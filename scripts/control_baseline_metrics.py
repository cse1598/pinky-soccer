"""Phase-0/1 measured metrics; keeps command discontinuity separate from plant response."""
import math
import numpy as np


def wrap(x):
    return np.arctan2(np.sin(x),np.cos(x))


def stats(values):
    a=np.asarray([v for v in values if v is not None and math.isfinite(v)],float)
    if not len(a):return dict(n=0)
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),
        p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),maximum=float(a.max()))


def within(rows,start,end,role):
    return [r for r in rows if r['role']==role and start<=r['wall']<=end]


def metrics(rows,episode):
    start,end=episode['start_wall'],episode['end_wall']
    data=dict(episode)
    commands=within(rows,start,end,'cmd')
    status=within(rows,start,end,'status')
    truth=within(rows,start,end,'truth')
    odom=within(rows,start,end,'odom')
    results=within(rows,start,end,'result')
    goal=episode.get('goal')
    if goal:
        matching=[r for r in within(rows,start,end,'goal') if r.get('action')=='GoToPose'
                  and math.hypot(r['x']-goal[0],r['y']-goal[1])<1e-6
                  and abs(float(wrap(r['yaw']-goal[2])))<1e-6]
        goal_ids={str(r['goal_id']) for r in matching}
        data['goal_ids']=sorted(goal_ids)
        # Receipt of an old status can overlap a new request. Never attribute it to this trial.
        status=[r for r in status if str(r['data'].get('goal_id')) in goal_ids]
        results=[r for r in results if str(r.get('goal_id')) in goal_ids]
    data['samples']={key:len(within(rows,start,end,key)) for key in
        ('cmd','mpc_command','drive','plant_command','joints','odom','world','status','result','truth')}
    for role in ('cmd','mpc_command','drive','joints','world'):
        times=np.asarray([r['wall'] for r in within(rows,start,end,role)])
        data[role+'_receipt_period_ms']=stats(np.diff(times)*1000)
    if commands:
        x=np.asarray([[r['v'],r['w']] for r in commands])
        zero=np.all(abs(x)<1e-6,axis=1)
        data['zero_command_samples']=int(zero.sum())
        data['zero_run_starts']=int(np.sum(zero & np.r_[True,~zero[:-1]]))
        data['nonzero_command_samples']=int((~zero).sum())
        starts=np.flatnonzero(zero & np.r_[True,~zero[:-1]])
        ends=np.flatnonzero(zero & np.r_[~zero[1:],True])
        data['zero_intervals']=[]
        for a,b in zip(starts,ends):
            nearby=[r for r in status if abs(r['wall']-commands[a]['wall'])<=.1]
            nearest=min(nearby,key=lambda r:abs(r['wall']-commands[a]['wall'])) if nearby else None
            data['zero_intervals'].append(dict(start_wall=commands[a]['wall'],last_sample_wall=commands[b]['wall'],
                between_nonzero_commands=bool(a>0 and b<len(zero)-1),
                nearby_reason=nearest['data'].get('reason') if nearest else None,
                nearby_activation_missed=nearest['data'].get('activation_missed') if nearest else None))
        data['zero_intervals_between_nonzero']=sum(z['between_nonzero_commands'] for z in data['zero_intervals'])
        data['command_delta_v']=stats(abs(np.diff(x[:,0])))
        data['command_delta_w']=stats(abs(np.diff(x[:,1])))
        # Sign changes through a deadband are command facts, not 'unnecessary' motion labels.
        for i,k in enumerate(('v','w')):
            significant=np.sign(x[:,i][abs(x[:,i])>(.01 if i==0 else .05)])
            data['command_'+k+'_sign_changes']=int(np.sum(np.diff(significant)!=0))
    unique={}
    for r in status:
        d=r['data'];key=(d.get('goal_id'),d.get('solve_id'))
        if key[1] is not None and key not in unique:unique[key]=d
    data['completed_solver_metrics']={k:stats(d.get(k) for d in unique.values())
        for k in ('ms','acados_ms','worker_wait_ms','result_pickup_ms')}
    data['solver_status_counts']={}
    for d in unique.values():
        key=str(d.get('status'));data['solver_status_counts'][key]=data['solver_status_counts'].get(key,0)+1
    data['missed_slot_count']=None
    data['miss_note']='Legacy activation_missed is sticky; canceled jobs may be unobserved. No exact count inferred.'
    traces=[r['data']['timing_trace'] for r in status if r['data'].get('timing_trace')
            and str(r['data']['timing_trace'].get('goal_id'))==str(r['data'].get('goal_id'))]
    if traces:
        # Per-goal counters start at zero in the controller, rather than at the first received flag.
        data['observed_slot_totals']={k:max(t[k] for t in traces) for k in ('slots','missed','init_missed','track_missed')}
        data['missed_slot_count']=data['observed_slot_totals']['missed']
        data['miss_note']='Exact per-goal counter through the latest received trace; capture may end before the goal.'
        jobs={t['last_job']['seq']:t['last_job'] for t in traces if t.get('last_job')
              and str(t['last_job'].get('goal_id'))==str(t['goal_id'])}
        data['observed_canceled_jobs']=sum(j['canceled'] for j in jobs.values())
        data['all_observed_jobs_metrics']={k:stats(j.get(k) for j in jobs.values())
            for k in ('worker_run_ms','worker_wait_ms','result_pickup_ms','end_to_pickup_ms')}
        data['canceled_jobs']=[j for j in jobs.values() if j['canceled']]
    data['activation_jitter_sim_ms']=stats(r['data']['activation_jitter_sim_s']*1000 for r in status
        if r['data'].get('activation_jitter_sim_s') is not None)
    data['status_reason_samples']={}
    for r in status:
        reason=r['data'].get('reason','unknown')
        data['status_reason_samples'][reason]=data['status_reason_samples'].get(reason,0)+1
    data['source_age_ms']={}
    for role in ('joints','odom','localization','world','truth'):
        data['source_age_ms'][role]=stats((r['bag_ns']-r['stamp_ns'])/1e6
            for r in within(rows,start,end,role) if r.get('stamp_ns',0)>0)
    # Raw map->odom correction increments are diagnostic, not automatically localization errors.
    tf=within(rows,start,end,'tf')
    data['map_odom_translation_step_m']=stats(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(tf,tf[1:]))
    data['map_odom_yaw_step_rad']=stats(abs(float(wrap(b['yaw']-a['yaw']))) for a,b in zip(tf,tf[1:]))
    if truth:
        data['truth_displacement_m']=float(np.linalg.norm(np.array([truth[-1]['x'],truth[-1]['y']])-
                                                        [truth[0]['x'],truth[0]['y']]))
        data['truth_yaw_change_rad']=float(wrap(truth[-1]['yaw']-truth[0]['yaw']))
        if len(truth)>1:
            xy=np.array([[r['x'],r['y']] for r in truth])
            data['truth_path_length_m']=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
        if goal:
            dist=np.array([math.hypot(r['x']-goal[0],r['y']-goal[1]) for r in truth])
            ang=np.array([abs(float(wrap(r['yaw']-goal[2]))) for r in truth])
            inside=np.flatnonzero((dist<=.02)&(ang<=math.radians(3)))
            data['truth_final_position_error_m']=float(dist[-1])
            data['truth_final_heading_error_rad']=float(ang[-1])
            data['truth_first_legacy_pose_region_wall_s']=truth[inside[0]]['wall']-start if len(inside) else None
            data['success_after_first_truth_region_wall_s']=(end-truth[inside[0]]['wall']) if len(inside) and episode.get('result')=='success' else None
        stamped=[r for r in truth if r.get('stamp_ns',0)>0]
        data['pose_accuracy_source_aligned']={}
        if len(stamped)>1:
            origin=stamped[0]['stamp_ns']
            ts=np.array([(r['stamp_ns']-origin)/1e9 for r in stamped])
            ref=np.array([[r[k] for k in ('x','y','yaw')] for r in stamped])
            for role in ('world','localization'):
                estimates=[r for r in within(rows,start,end,role) if r.get('stamp_ns',0)>0
                    and stamped[0]['stamp_ns']<=r['stamp_ns']<=stamped[-1]['stamp_ns']]
                if not estimates:continue
                at=np.array([(r['stamp_ns']-origin)/1e9 for r in estimates])
                values=np.array([[r[k] for k in ('x','y','yaw')] for r in estimates])
                error=values-np.stack([np.interp(at,ts,ref[:,i] if i<2 else np.unwrap(ref[:,i])) for i in range(3)],axis=1)
                error[:,2]=wrap(error[:,2])
                data['pose_accuracy_source_aligned'][role]=dict(n=len(estimates),
                    x_rmse_m=float(np.sqrt(np.mean(error[:,0]**2))),y_rmse_m=float(np.sqrt(np.mean(error[:,1]**2))),
                    xy_rmse_m=float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
                    yaw_rmse_rad=float(np.sqrt(np.mean(error[:,2]**2))))
    if goal:
        data['motion_results']=[dict(state=r['state'],reason=r['reason']) for r in results[-3:]]
    data['odom_v_abs']=stats(abs(r['v']) for r in odom)
    data['odom_w_abs']=stats(abs(r['w']) for r in odom)
    data['elapsed_wall_s']=end-start
    return data
