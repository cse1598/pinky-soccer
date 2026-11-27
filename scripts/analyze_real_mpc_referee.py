#!/usr/bin/env python3
"""Read-only comparison of real GoToPose topic logs; never publishes commands.

The referee topic already contains its Kalman estimate. The trailing mean below
is ONLY an assessment display, never a new localization/control measurement.
"""
import argparse
from bisect import bisect_right
from collections import defaultdict
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from analyze_gotopose_bag import read_bag, stats


def smooth(t, p):
    """Five sample causal mean; reset on gaps, never interpolate missing data."""
    result, history, previous = [], [], None
    for stamp, point in zip(t, p):
        if previous is None or not 0 < stamp-previous < .2:
            history = []
        history = (history+[point])[-5:]
        result.append(np.mean(history, axis=0))
        previous = stamp
    return np.asarray(result)


def interior_zero_bouts(rows):
    """Exact zero Twist bouts between active commands, including settle phases."""
    active = [i for i, r in enumerate(rows) if abs(r['v'])+abs(r['w']) > 1e-6]
    if not active:
        return []
    bouts, start = [], None
    for i in range(active[0], active[-1]+1):
        r = rows[i]
        if abs(r['v'])+abs(r['w']) <= 1e-6:
            if start is None:
                start = r['bag_ns']/1e9
        elif start is not None:
            end = r['bag_ns']/1e9
            if end-start >= .08:
                bouts.append(dict(start_ros=start, duration_s=end-start))
            start = None
    return bouts


def path_quality(points, start, target, rotation=False):
    """Geometric assessment, independent of the BT terminal result.

    Cross track refers to the start-target chord, not the solver's planned path.
    Referee target errors retain its body-center/base-frame offset.
    """
    p, start, target = np.asarray(points), np.asarray(start), np.asarray(target)
    result = dict(max_displacement_from_start_m=float(np.linalg.norm(p-start, axis=1).max()))
    delta = target-start
    length = float(np.linalg.norm(delta))
    if not rotation and length > .025:
        along = (p-start)@(delta/length)
        cross = (p-start)@np.array([-delta[1], delta[0]])/length
        result.update(chord_length_m=length,
            max_overshoot_along_chord_m=float(max(0, along.max()-length)),
            max_cross_track_to_chord_m=float(np.abs(cross).max()))
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--bag', type=Path, required=True)
    args = ap.parse_args()
    records = [json.loads(l) for l in (args.run/'topics.jsonl').open()]
    results = json.loads((args.run/'results.json').read_text())
    bag, inventory, errors = read_bag(args.bag, {
        '/cmd_vel': 'cmd', '/drive/cmd_vel': 'drive', '/odom': 'odom',
        '/wheel_joint_states_raw': 'raw_joints'}, 'wheel_left_joint', 'wheel_right_joint')
    grouped = defaultdict(list)
    for r in records:
        grouped[r['case'], r['episode'], r['topic']].append(r)
    output = args.run/'analysis'
    output.mkdir(exist_ok=True)
    summary = dict(bag=str(args.bag), decode_errors=errors, cases=[])
    for case in results:
        name = case['case']
        job, slot = {}, {}
        for r in records:
            if r['case'] != name or r['topic'] != 'mpc/status':
                continue
            trace = r['data'].get('timing_trace', {})
            for key, dest in [('last_job', job), ('last_slot', slot)]:
                d = trace.get(key, {})
                if d:
                    dest[d.get('goal_id', trace.get('goal_id')), d['seq']] = d
        jobs = [d for d in job.values() if not d.get('canceled')]
        report = dict(case=name, preparation=case['preparation']['state'],
            preparation_reason=case['preparation']['reason'], slots=len(slot),
            missed=sum(not s['accepted'] for s in slot.values()),
            timing_ms={k: stats([d.get(k) for d in jobs]) for k in
                       ('ms', 'worker_wait_ms', 'result_pickup_ms', 'end_to_pickup_ms')},
            jitter_ms=stats([s['jitter_sim_s']*1000 for s in slot.values()]), goals=[])
        summary['cases'].append(report)
        if jobs and slot:
            fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
            origin = min(s['actual_sim_s'] for s in slot.values())
            for field in ('ms', 'worker_wait_ms', 'result_pickup_ms'):
                jj = [d for d in jobs if d.get(field) is not None]
                axes[0].plot([d['submitted_sim_s']-origin for d in jj],
                             [d[field] for d in jj], '.-', label=field)
            axes[0].set(ylabel='time [ms]', title='Unique completed, non-canceled jobs')
            axes[0].legend()
            for accepted, label in [(True, 'accepted'), (False, 'missed')]:
                ss = [s for s in slot.values() if s['accepted'] == accepted]
                axes[1].scatter([s['actual_sim_s']-origin for s in ss],
                                [s['jitter_sim_s']*1000 for s in ss], label=label)
            axes[1].set(xlabel='ROS time relative to first slot [s]', ylabel='activation jitter [ms]')
            axes[1].legend()
            for ax in axes:
                ax.grid(alpha=.25)
            fig.suptitle(name); fig.tight_layout()
            fig.savefig(output/f'{name}_timing.png', dpi=130); plt.close(fig)
        for goal in case['goals']:
            episode = goal['name']
            ref = grouped[name, episode, 'referee/state']
            world = grouped[name, episode, 'world_model']
            status = grouped[name, episode, 'mpc/status']
            if not ref or not status:
                continue
            # Episode labels switch before each new request. The first BT
            # message can still be the PREVIOUS request's terminal status.
            # Use the complete labelled interval, including final stationary
            # samples, rather than truncating at that unrelated terminal event.
            t0 = min(r['receipt_ros'] for r in ref+status)
            t1 = max(r['receipt_ros'] for r in ref)
            selected = {role: [r for r in bag if r['role'] == role and
                        t0 <= r['bag_ns']/1e9 <= t1] for role in ('cmd', 'drive', 'odom')}
            ref = [r for r in ref if t0 <= r['receipt_ros'] <= t1 and r['data']['robots']]
            rt = np.array([r['data']['stamp'] for r in ref])
            rp = np.array([r['data']['robots'][0]['p'] for r in ref])
            smoothed = smooth(rt, rp)
            world = [r for r in world if r['receipt_ros'] <= t1]
            wt = np.array([r['receipt_ros'] for r in world])
            wp = np.array([r['data']['pose'] for r in world])
            reasons = sorted({r['data']['reason'] for r in status if r['data'].get('state') == 'failed'})
            drive = selected['drive']
            coverage = {role: dict(n=len(rr),
                span_s=(rr[-1]['bag_ns']-rr[0]['bag_ns'])/1e9 if len(rr)>1 else 0,
                starts_after_episode_s=rr[0]['bag_ns']/1e9-t0 if rr else None,
                max_gap_s=max(np.diff([r['bag_ns']/1e9 for r in rr]), default=0))
                for role, rr in selected.items()}
            covered = bool(drive and drive[0]['bag_ns']/1e9-t0 < .15 and
                           coverage['drive']['span_s'] >= .7*goal['duration'])
            bouts = interior_zero_bouts(drive)
            st = [r['receipt_ros'] for r in status]
            for bout in bouts:
                idx = max(0, bisect_right(st, bout['start_ros'])-1)
                bout['mpc_state_at_start'] = status[idx]['data'].get('state')
                bout['mpc_reason_at_start'] = status[idx]['data'].get('reason')
            sign = np.sign([r['v'] for r in drive if abs(r['v']) > .01])
            settle = rp[rt >= rt[-1]-.2]
            target = np.array(goal['target'])
            rotation = episode in ('rotate', 'half_turn')
            world_quality = path_quality(wp[:, :2], goal['world_start'][:2], target[:2], rotation)
            ref_quality = path_quality(rp, goal['referee_start'], target[:2], rotation)
            unwrapped_yaw = np.unwrap(wp[:, 2])
            target_yaw = target[2]+2*np.pi*round((unwrapped_yaw[-1]-target[2])/(2*np.pi))
            if rotation:
                direction = np.sign(target_yaw-unwrapped_yaw[0])
                world_quality['yaw_overshoot_rad'] = float(max(0, np.max(direction*(unwrapped_yaw-target_yaw))))
            item = dict(goal, failure_reasons=reasons, topic_coverage=coverage,
                outcome_meaning='BT terminal result only; not a motion quality pass',
                world_path_quality=world_quality, referee_path_quality=ref_quality,
                drive_interval_covered=covered, drive_zero_bouts_ge80ms=bouts,
                drive_sign_changes_over_1cms=int(np.sum(sign[1:] != sign[:-1])),
                peak_drive_mps=max([abs(r['v']) for r in drive], default=0),
                peak_odom_mps=max([abs(r['v']) for r in selected['odom']], default=0),
                referee_settled_mean=settle.mean(axis=0).tolist(),
                referee_settled_error_m=float(np.linalg.norm(settle.mean(axis=0)-target[:2])),
                referee_y_std_m=float(np.std(rp[:, 1])),
                referee_missing_frames=sum(not r['data']['robots'] for r in grouped[name, episode, 'referee/state']))
            report['goals'].append(item)
            fig, axes = plt.subplots(2, 2, figsize=(12, 8))
            ax = axes[0, 0]
            ax.plot(rp[:, 0], rp[:, 1], '.', alpha=.3, label='Referee KF topic')
            ax.plot(smoothed[:, 0], smoothed[:, 1], label='Trailing mean 5 (assessment only)')
            ax.plot(wp[:, 0], wp[:, 1], label='Onboard world pose')
            start = goal['world_start']
            ax.plot([start[0], target[0]], [start[1], target[1]], 'k--', label='Straight chord (not planned path)')
            ax.plot(target[0], target[1], 'rx', ms=10)
            ax.set(xlabel='x [m]', ylabel='y [m]', title='Path; no reference alignment')
            ax.axis('equal'); ax.legend(fontsize=7)
            for role in ('cmd', 'drive', 'odom'):
                rr = selected[role]
                axes[0, 1].step([r['bag_ns']/1e9-t0 for r in rr], [r['v'] for r in rr], where='post', label=role)
                axes[1, 0].step([r['bag_ns']/1e9-t0 for r in rr], [r['w'] for r in rr], where='post', label=role)
            axes[0, 1].set(ylabel='v [m/s]', xlabel='receipt time [s]'); axes[0, 1].legend()
            axes[1, 0].set(ylabel='omega [rad/s]', xlabel='receipt time [s]')
            axes[1, 1].plot(wt-t0, np.linalg.norm(wp[:, :2]-target[:2], axis=1)*1000, label='World target error')
            axes[1, 1].plot(rt-t0, np.linalg.norm(rp-target[:2], axis=1)*1000, alpha=.4, label='Referee target error')
            axes[1, 1].set(ylabel='position error [mm]', xlabel='time [s]'); axes[1, 1].legend(fontsize=8)
            for ax in axes.flat:
                ax.grid(alpha=.25)
            fig.suptitle(f'{name} / {episode} / {goal["outcome"]}')
            fig.tight_layout(); fig.savefig(output/f'{name}_{episode}.png', dpi=130); plt.close(fig)
            fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
            axes[0].plot(wt-t0, wp[:, 0], label='World x')
            axes[0].plot(wt-t0, wp[:, 1], label='World y')
            axes[0].plot(rt-t0, smoothed[:, 0], label='Referee x / mean5', alpha=.7)
            axes[0].plot(rt-t0, smoothed[:, 1], label='Referee y / mean5', alpha=.7)
            axes[0].set(ylabel='position [m]'); axes[0].legend(fontsize=8)
            axes[1].plot(wt-t0, np.rad2deg(unwrapped_yaw), label='World yaw (unwrapped)')
            axes[1].axhline(np.rad2deg(target_yaw), color='red', linestyle='--', label='Target yaw')
            axes[1].set(ylabel='yaw [deg]'); axes[1].legend(fontsize=8)
            for role in ('cmd', 'drive', 'odom'):
                rr = selected[role]
                axes[2].step([r['bag_ns']/1e9-t0 for r in rr], [r['w'] for r in rr], where='post', label=role)
            axes[2].set(ylabel='omega [rad/s]', xlabel='receipt time [s]'); axes[2].legend()
            for ax in axes:
                ax.grid(alpha=.25)
            fig.suptitle(f'{name} / {episode}: completion != quality pass')
            fig.tight_layout(); fig.savefig(output/f'{name}_{episode}_quality.png', dpi=130); plt.close(fig)
    (output/'summary.json').write_text(json.dumps(summary, indent=2))
    for c in summary['cases']:
        print(c['case'], c['preparation'], 'slots/missed', c['slots'], c['missed'])
        for g in c['goals']:
            print(' ', g['name'], g['outcome'], 'rest error mm', round(g['referee_settled_error_m']*1000, 1),
                  'zero bouts', len(g['drive_zero_bouts_ge80ms']), 'reversals', g['drive_sign_changes_over_1cms'])


if __name__ == '__main__':
    main()
