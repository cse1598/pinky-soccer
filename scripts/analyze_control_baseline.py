#!/usr/bin/env python3
"""Offline Phase-0/1 plots on recorder monotonic receipt time; never publishes ROS."""
import argparse
import csv
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from control_baseline_metrics import metrics, stats, wrap


def load_run(directory):
    with gzip.open(directory/'samples.jsonl.gz','rt') as stream:
        rows=[json.loads(line) for line in stream]
    return rows,json.loads((directory/'episodes.json').read_text()),json.loads((directory/'metadata.json').read_text())


def cpu_rates(samples):
    previous={};rates=[]
    for row in samples:
        old=previous.get(row['pid']);previous[row['pid']]=row
        if old and row['wall']>old['wall'] and row['cpu_s']>=old['cpu_s']:
            rates.append(dict(wall=row['wall'],pid=row['pid'],name=row['name'],
                cpu_one_core_percent=100*(row['cpu_s']-old['cpu_s'])/(row['wall']-old['wall']),rss=row['rss']))
    return rates


def save_csv(path,rows,keys):
    with path.open('w') as stream:
        writer=csv.DictWriter(stream,keys,extrasaction='ignore');writer.writeheader();writer.writerows(rows)


def analyze(directory,output):
    rows,episodes,meta=load_run(directory)
    output.mkdir(parents=True,exist_ok=True)
    grouped={k:sorted((r for r in rows if r['role']==k),key=lambda r:r['wall']) for k in {r['role'] for r in rows}}
    report=dict(episodes=[metrics(rows,e) for e in episodes],
        evidence=dict(truth_use='evaluation only',time_axis='monotonic receipt wall seconds',
            source_age='ROS receipt clock minus source stamp; separate clock from wall',
            zero_intervals='Coincidence only; initial, terminal, and between-nonzero intervals separated.',
            timing='Only picked-up solve IDs; canceled generations are missing. Sticky activation flags/jitter are not independent slot events.',
            watchdog='drive output is observed; no direct watchdog timeout counter exists in this capture.',
            cpu='One-core percent from cumulative process time; recorder overhead included only in revised captures.',
            comparison='Sequential course legs have different actual starts. No controller ranking from these runs.'))
    rates=cpu_rates(meta.get('cpu',[]))
    report['cpu_by_process']={str(pid):dict(name=next(r['name'] for r in rates if r['pid']==pid),
        one_core_percent=stats(r['cpu_one_core_percent'] for r in rates if r['pid']==pid)) for pid in {r['pid'] for r in rates}}
    (output/'summary.json').write_text(json.dumps(report,indent=2))
    save_csv(output/'episodes.csv',report['episodes'],['case','result','elapsed_wall_s','truth_final_position_error_m',
        'truth_final_heading_error_rad','zero_run_starts','zero_intervals_between_nonzero','success_after_first_truth_region_wall_s'])
    save_csv(output/'commands.csv',[r for r in rows if r['role'] in ('cmd','mpc_command','drive','plant_command','odom')],
        ['wall','bag_ns','stamp_ns','role','v','w'])
    save_csv(output/'poses.csv',[r for r in rows if r['role'] in ('truth','world','localization','tf')],
        ['wall','bag_ns','stamp_ns','role','x','y','yaw','confidence'])
    unique={}
    for r in grouped.get('status',[]):
        d=r['data'];key=(d.get('goal_id'),d.get('solve_id'))
        if key[1] is not None and key not in unique:unique[key]=dict(d,wall=r['wall'])
    solves=list(unique.values())
    traced_jobs={};traced_slots={}
    for r in grouped.get('status',[]):
        trace=r['data'].get('timing_trace') or {}
        job=trace.get('last_job');slot=trace.get('last_slot')
        if job:traced_jobs.setdefault(job['seq'],dict(job,wall=r['wall']))
        if slot:traced_slots.setdefault((slot['goal_id'],slot['seq']),dict(slot,wall=r['wall']))
    save_csv(output/'jobs.csv',list(traced_jobs.values()),['wall','seq','solve_id','goal_id','canceled','status','ok',
        'submitted_sim_s','submitted_wall_s','pickup_tick_wall_s','scheduled_due_sim_s','worker_run_ms','worker_wait_ms','result_pickup_ms','end_to_pickup_ms','acados_ms'])
    save_csv(output/'slots.csv',list(traced_slots.values()),['wall','seq','goal_id','due_sim_s','actual_sim_s',
        'jitter_sim_s','activation_tick_wall_s','accepted','had_offer','tracking_before_activation','pending_solve_id'])
    save_csv(output/'solves.csv',solves,['wall','goal_id','solve_id','status','ms','acados_ms','worker_wait_ms','result_pickup_ms'])
    save_csv(output/'cpu.csv',rates,['wall','pid','name','cpu_one_core_percent','rss'])
    fig,axes=plt.subplots(4,1,figsize=(13,14),layout='constrained')
    for role in ('truth','world','localization'):
        data=grouped.get(role,[])
        if data:axes[0].plot([r['x'] for r in data],[r['y'] for r in data],label=role,lw=1)
    for i,e in enumerate(episodes):
        if e.get('goal'):axes[0].plot(*e['goal'][:2],'kx');axes[0].annotate(f"{i+1}:{e['case']}",e['goal'][:2])
    axes[0].set(xlabel='map x [m]',ylabel='map y [m]',title=directory.name);axes[0].axis('equal')
    for role in ('cmd','mpc_command','drive','plant_command','odom'):
        data=grouped.get(role,[])
        for ax,key in zip(axes[1:3],('v','w')):
            if data:ax.plot([r['wall'] for r in data],[r.get(key,np.nan) for r in data],label=role,lw=1)
    data=grouped.get('joints',[])
    for key in ('raw_l','raw_r'):
        if data:axes[3].plot([r['wall'] for r in data],[r.get(key,np.nan) for r in data],label=key,lw=.8)
    axes[1].set_ylabel('linear velocity [m/s]');axes[2].set_ylabel('angular velocity [rad/s]')
    axes[3].set(ylabel='measured wheel [rad/s]',xlabel='monotonic receipt [s]')
    for ax in axes[1:]:ax.sharex(axes[1])
    decorate(axes,episodes,skip_first=True)
    fig.savefig(output/'motion.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(7,1,sharex=True,figsize=(14,18),layout='constrained')
    for key in ('ms','acados_ms','worker_wait_ms','result_pickup_ms'):
        data=[r for r in solves if r.get(key) is not None]
        if data:axes[0].plot([r['wall'] for r in data],[r[key] for r in data],'.-',label=key,lw=.6,ms=2)
    axes[0].axhline(100,color='red',ls='--',label='100 ms slot (not total end-to-end budget)')
    axes[0].set_ylabel('picked-up solve [ms]')
    canceled=[j for j in traced_jobs.values() if j['canceled'] and j.get('worker_run_ms') is not None]
    if canceled:axes[0].plot([j['wall'] for j in canceled],[j['worker_run_ms'] for j in canceled],'rx',label='canceled worker run [ms]')
    status=grouped.get('status',[])
    for key,scale in (('activation_jitter_sim_s',1000),):
        data=[r for r in status if r['data'].get(key) is not None]
        if data:axes[1].plot([r['wall'] for r in data],[0. if abs(scale*r['data'][key])<1e-9 else scale*r['data'][key] for r in data],label='sticky activation jitter [sim ms] (display epsilon 1e-9)')
    flags=[r for r in status if 'activation_missed' in r['data']]
    if flags:axes[2].step([r['wall'] for r in flags],[int(r['data']['activation_missed']) for r in flags],where='post',label='sticky missed flag')
    missed=[s for s in traced_slots.values() if not s['accepted']]
    if missed:axes[2].plot([s['wall'] for s in missed],np.ones(len(missed)),'rx',label='unique observed missed slots')
    axes[2].set(ylabel='missed flag',ylim=(-.1,1.1))
    truth=grouped.get('truth',[]);world=grouped.get('world',[])
    if len(truth)>1 and world:
        times=np.array([r['wall'] for r in truth]);selected=[r for r in world if times[0]<=r['wall']<=times[-1]]
        ts=np.array([r['wall'] for r in selected]);values=np.array([[r[k] for k in ('x','y','yaw')] for r in truth])
        xy=np.array([[r[k] for k in ('x','y')] for r in selected]).reshape(-1,2)
        if len(ts):
            reference=np.stack([np.interp(ts,times,values[:,i]) for i in (0,1)],axis=1)
            axes[3].plot(ts,1000*np.linalg.norm(xy-reference,axis=1),label='estimate-truth position [mm] (receipt alignment)')
            axes[3].plot(ts,np.degrees(wrap(np.array([r['yaw'] for r in selected])-np.interp(ts,times,np.unwrap(values[:,2])))),label='yaw error [deg]')
    tf=grouped.get('tf',[])
    if len(tf)>1:
        xy=np.array([[r['x'],r['y']] for r in tf]);yaw=np.array([r['yaw'] for r in tf]);ts=[r['wall'] for r in tf[1:]]
        axes[4].plot(ts,1000*np.linalg.norm(np.diff(xy,axis=0),axis=1),label='map->odom translation step [mm]')
        axes[4].plot(ts,np.degrees(wrap(np.diff(yaw))),label='map->odom yaw step [deg]')
    for role in ('joints','odom','localization','world'):
        data=[r for r in grouped.get(role,[]) if r.get('stamp_ns',0)>0]
        if data:axes[5].plot([r['wall'] for r in data],[(r['bag_ns']-r['stamp_ns'])/1e6 for r in data],label=role,lw=.7)
    axes[5].set_ylabel('source age [ROS ms]')
    # Keep the recorder separate from nodes; process names are stored in cpu.csv.
    for pid in {r['pid'] for r in rates}:
        data=[r for r in rates if r['pid']==pid]
        if data and max(r['cpu_one_core_percent'] for r in data)>5:
            axes[6].plot([r['wall'] for r in data],[r['cpu_one_core_percent'] for r in data],label=f"{data[0]['name']}:{pid}",lw=.7)
    axes[6].set(ylabel='CPU [% one core]',xlabel='monotonic receipt [s]')
    decorate(axes,episodes)
    fig.savefig(output/'timing.png',dpi=140);plt.close(fig)
    return report


def decorate(axes,episodes,skip_first=False):
    for i,ax in enumerate(axes):
        ax.grid(alpha=.2)
        if ax.get_legend_handles_labels()[0]:ax.legend(loc='upper left',fontsize=7,ncol=3)
        if skip_first and i==0:continue
        for e in episodes:
            ax.axvline(e['start_wall'],color='gray',alpha=.3,lw=.6)
            ax.axvline(e['end_wall'],color='red' if e.get('result')!='success' else 'green',alpha=.3,lw=.6)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=analyze(args.directory,args.output or args.directory/'analysis')
    print(json.dumps([{k:e.get(k) for k in ('case','result','elapsed_wall_s','zero_intervals_between_nonzero')} for e in result['episodes']],indent=2))
