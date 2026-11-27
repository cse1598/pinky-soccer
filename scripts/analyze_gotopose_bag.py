#!/usr/bin/env python3
"""Read-only ROS 2 GoToPose diagnostics. Never creates ROS nodes or publishes.

All correlations use recorder receipt timestamps. Source stamps remain separate;
neither clock offsets nor unique command identities are inferred from a Twist.
"""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import subprocess

import numpy as np


TOPICS = dict(cmd='cmd_vel', drive='drive/cmd_vel', joints='joint_states',
              raw_joints='wheel_joint_states_raw', wheel_joints='wheel_joint_states',
              status='mpc/status', odom='odom', amcl='amcl_pose', world='world_model',
              goal='behavior_goal', result='motion/result', motor='motor/diagnostics',
              motor_info='motor/info', tf='tf', sync='calibration/time_sync_status')
ROOT = Path(__file__).resolve().parents[1]
BASELINE_FILES = [
    'src/pinky_soccer_mpc/pinky_soccer_mpc/mpc_controller.py',
    'src/pinky_soccer_mpc/pinky_soccer_mpc/scheduled_input.py',
    'src/pinky_soccer_mpc/pinky_soccer_mpc/acados_time_optimal.py',
    'src/pinky_soccer_dynamics/config/dynamics.yaml',
    'src/pinky_soccer_dynamics/pinky_soccer_dynamics/profile_model.py',
    'src/pinky_soccer_bringup/pinky_soccer_bringup/command_watchdog.py',
]


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def stats(values):
    a = np.asarray([v for v in values if finite(v)], dtype=float)
    if not len(a):
        return {'n': 0}
    return dict(n=len(a), min=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


def clean_json(value):
    """Solver failure diagnostics may legitimately contain Infinity/NaN."""
    if isinstance(value, dict):
        return {k: clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def normalize(role, msg, left, right):
    """Extract measurements without applying sign/clock/geometry corrections."""
    d = {}
    if hasattr(msg, 'header'):
        d.update(stamp_ns=stamp_ns(msg.header.stamp), frame=msg.header.frame_id)
    if role in ('cmd', 'drive'):
        d.update(v=msg.linear.x, w=msg.angular.z, tag=msg.linear.z)
    elif role in ('joints', 'raw_joints', 'wheel_joints'):
        vel = dict(zip(msg.name, msg.velocity))
        d.update(raw_l=vel.get(left), raw_r=vel.get(right))
    elif role in ('status', 'motor', 'motor_info', 'sync'):
        payload = json.loads(msg.data)
        if not isinstance(payload, dict):
            raise ValueError('JSON message is not an object')
        d['data'] = payload
    elif role in ('odom', 'amcl'):
        p = msg.pose.pose
        d.update(x=p.position.x, y=p.position.y, yaw=yaw(p.orientation),
                 cov_x=msg.pose.covariance[0], cov_y=msg.pose.covariance[7],
                 cov_yaw=msg.pose.covariance[35])
        if role == 'odom':
            d.update(v=msg.twist.twist.linear.x, w=msg.twist.twist.angular.z)
    elif role == 'world':
        p = msg.self.pose
        d.update(x=p.x, y=p.y, yaw=p.theta, confidence=msg.self.confidence,
                 state=msg.self.state, last_seen_ns=stamp_ns(msg.self.last_seen),
                 v=math.hypot(msg.self.velocity.x, msg.self.velocity.y))
    elif role == 'goal':
        d.update(goal_id=str(msg.goal_id), action=msg.action, mode=msg.mode,
                 x=msg.target.x, y=msg.target.y, yaw=msg.target.theta)
    elif role == 'result':
        d.update(goal_id=str(msg.goal_id), state=msg.state, reason=msg.reason)
    elif role == 'tf':
        # Keep map->odom discontinuities separate from odom->base motion.
        for tr in msg.transforms:
            if tr.header.frame_id.strip('/') == 'map' and tr.child_frame_id.strip('/') == 'odom':
                d.update(stamp_ns=stamp_ns(tr.header.stamp), frame='map->odom',
                         x=tr.transform.translation.x, y=tr.transform.translation.y,
                         yaw=yaw(tr.transform.rotation))
                return d
        return None
    return d


def read_bag(path, mapping, left, right):
    """Support a read-only single sqlite file, or rosbag2 sqlite/MCAP directories."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rows, errors, inventory = [], Counter(), {}
    path = Path(path)
    if path.suffix == '.db3':
        connection = sqlite3.connect(f'{path.resolve().as_uri()}?mode=ro', uri=True)
        inventory = dict(connection.execute('SELECT name,type FROM topics').fetchall())
        ids = {i: (n, t) for i, n, t in connection.execute('SELECT id,name,type FROM topics')}
        selected = [i for i, (n, _) in ids.items() if n in mapping]
        placeholders = ','.join('?' for _ in selected)
        query = ('SELECT topic_id,data,timestamp FROM messages WHERE topic_id IN (' +
                 placeholders + ') ORDER BY timestamp,id')
        iterator = ((ids[i][0], data, ts) for i, data, ts in
                    connection.execute(query, selected)) if selected else iter(())
    else:
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        # Empty storage_id permits rosbag2 metadata autodetection.
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        inventory = {x.name: x.type for x in reader.get_all_topics_and_types()}
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(mapping)))

        def messages():
            while reader.has_next():
                yield reader.read_next()
        iterator = messages()
        connection = None
    classes = {}
    try:
        for topic, data, ts in iterator:
            if topic not in mapping:
                continue
            role = mapping[topic]
            try:
                if topic not in classes:
                    classes[topic] = get_message(inventory[topic])
                names = ('left_wheel_joint', 'right_wheel_joint') if role == 'raw_joints' else (left, right)
                d = normalize(role, deserialize_message(data, classes[topic]), *names)
                if d is not None:
                    rows.append(dict(d, bag_ns=int(ts), role=role, topic=topic))
            except (ValueError, TypeError, KeyError, AttributeError, ImportError, RuntimeError) as exc:
                errors[f'{topic}: {type(exc).__name__}: {exc}'] += 1
    finally:
        if connection is not None:
            connection.close()
    return rows, inventory, dict(errors)


def prepare(rows):
    """Use integer subtraction before float conversion to retain ns precision."""
    rows = sorted(rows, key=lambda x: x['bag_ns'])
    origin = rows[0]['bag_ns'] if rows else 0
    grouped = defaultdict(list)
    for original in rows:
        r = dict(original, t=(original['bag_ns']-origin)/1e9)
        source = r.get('stamp_ns')
        r['age_ms'] = (r['bag_ns']-source)/1e6 if source and source > 0 else None
        grouped[r['role']].append(r)
    return grouped, origin


def previous(rows, t, max_age, times=None):
    if not rows:
        return None
    i = bisect_right(times if times is not None else [r['t'] for r in rows], t)-1
    return rows[i] if i >= 0 and t-rows[i]['t'] <= max_age else None


def summarize(g, errors=None):
    out = dict(topics={}, decode_errors=errors or {}, conclusions=[])
    for role, rows in g.items():
        times = [r['t'] for r in rows]
        gaps = np.diff(times)
        stamps = [r['stamp_ns'] for r in rows if r.get('stamp_ns', 0) > 0]
        ds = np.diff(stamps)/1e6
        out['topics'][role] = dict(count=len(rows), receipt_period_ms=stats(gaps*1000),
            mean_hz=(len(rows)-1)/(times[-1]-times[0]) if len(rows)>1 and times[-1]>times[0] else None,
            gaps_over_200ms=int(sum(gaps > .2)), source_period_ms=stats(ds),
            nonincreasing_source_stamps=int(sum(ds <= 0)),
            missing_or_zero_source_stamp=sum(not r.get('stamp_ns') for r in rows),
            receipt_minus_source_ms=stats(r.get('age_ms') for r in rows))
    statuses = g.get('status', [])
    # solve metrics persist on subsequent 50 Hz reports; use first observation only.
    solves, seen = [], set()
    missed_runs = 0
    last_missed = False
    for r in statuses:
        d = r['data']
        key = (d.get('goal_id'), d.get('solve_id'))
        if key[1] is not None and key not in seen:
            seen.add(key)
            solves.append(r)
        missed = d.get('activation_missed') is True
        if missed and not last_missed:
            missed_runs += 1
        last_missed = missed
    out['timing'] = dict(observed_completed_solves=len(solves),
        missed_flag_true_runs_lower_bound=missed_runs,
        exact_missed_slot_count=None,
        reason='activation_missed is sticky; canceled generation results have no completion metrics',
        metrics={k: stats(r['data'].get(k) for r in solves) for k in
                 ('ms', 'acados_ms', 'worker_wait_ms', 'result_pickup_ms',
                  'solve_input_age_wall_s', 'solve_input_age_sim_s')},
        status_reasons=dict(Counter(r['data'].get('reason', '') for r in statuses)))
    out['timing']['sticky_jitter_status_weighted_ms'] = stats(
        r['data'].get('activation_jitter_sim_s')*1000 for r in statuses
        if finite(r['data'].get('activation_jitter_sim_s')))
    # Receipt-side reconstruction of 20 Hz sample-and-hold, NOT transport latency.
    comparisons = []
    cmd_times = [r['t'] for r in g.get('cmd', [])]
    for r in g.get('drive', []):
        c = previous(g.get('cmd', []), r['t'], .5, cmd_times)
        if c and all(finite(x) for x in (c.get('v'), c.get('w'), r.get('v'), r.get('w'))):
            comparisons.append(dict(t=r['t'], age_ms=1000*(r['t']-c['t']),
                dv=r['v']-c['v'], dw=r['w']-c['w']))
    out['command_chain'] = dict(comparable_samples=len(comparisons),
        receipt_latest_cmd_age_ms=stats(c['age_ms'] for c in comparisons),
        mismatches=sum(abs(c['dv'])>1e-5 or abs(c['dw'])>1e-5 for c in comparisons),
        transport_latency_ms=None,
        reason='Twist has no stamp/sequence; PC recorder order is not watchdog callback order')
    zero_runs, was_zero = [], False
    status_times = [r['t'] for r in statuses]
    prior_cmd_t = -math.inf
    for r in g.get('cmd', []):
        zero = finite(r.get('v')) and finite(r.get('w')) and abs(r['v'])<=1e-6 and abs(r['w'])<=1e-6
        if zero and (not was_zero or r['t']-prior_cmd_t > .2):
            # report follows send(); nearest status is more appropriate than causal hold.
            index = bisect_right(status_times, r['t'])
            candidates = statuses[max(0, index-1):index+1]
            near = min(candidates, key=lambda s: abs(s['t']-r['t'])) if candidates else None
            near = near if near and abs(near['t']-r['t'])<=.05 else None
            d = near['data'] if near else {}
            zero_runs.append(dict(t=r['t'], status_state=d.get('state'), reason=d.get('reason'),
                                  activation_missed=d.get('activation_missed'),
                                  goal_id=d.get('goal_id')))
        was_zero = zero
        prior_cmd_t = r['t']
    out['zero_command_starts'] = zero_runs
    out['missing_roles'] = [k for k in ('cmd', 'drive', 'status', 'joints', 'odom', 'amcl', 'world') if not g.get(k)]
    diagnostics = [r['data'] for r in g.get('motor', [])]
    commands = [c for d in diagnostics for c in d.get('commands', [])]
    out['driver_diagnostics'] = dict(
        serial_write_ms=stats((c['written_unix_ns']-c['recv_unix_ns'])/1e6 for c in commands
                             if c.get('written_unix_ns') is not None and c.get('recv_unix_ns') is not None),
        feedback_bulk_read_ms=stats((d['feedback']['read_done_unix_ns']-d['feedback']['read_start_unix_ns'])/1e6
                                   for d in diagnostics if d.get('feedback') and
                                   'read_done_unix_ns' in d['feedback'] and 'read_start_unix_ns' in d['feedback']),
        info=[r['data'] for r in g.get('motor_info', [])],
        note='Diagnostic driver must already be enabled; this tool does not enable it.')
    out['conclusions'] = [
        'No root cause is established automatically; use time-aligned evidence and trial context.',
        'A missed flag is not a unique activation event. True-run count is only a lower bound.',
        'Completed-solve metrics omit canceled generations: short observed solves do not rule out deadline misses.',
        'Receipt minus source stamp includes clock offset and buffering, not just sensor/network delay.',
        'AMCL/TF/world steps can be legitimate localization corrections; wheel odom is not independent ground truth.',
        'Low commanded speed with zero wheel feedback is a candidate deadzone/stiction observation, not proof.',
        'Real bringup merges wheel feedback through joint_state_publisher at 20 Hz and restamps it. Record raw_joints to assess physical measurement freshness.',
    ]
    return out, solves, comparisons


def pose_steps(rows):
    result = []
    for a, b in zip(rows, rows[1:]):
        if a.get('frame') != b.get('frame'):
            continue
        if not all(finite(r.get(k)) for r in (a, b) for k in ('x', 'y', 'yaw')):
            continue
        angle = math.atan2(math.sin(b['yaw']-a['yaw']), math.cos(b['yaw']-a['yaw']))
        result.append(dict(t=b['t'], dt=b['t']-a['t'],
                           distance=math.hypot(b['x']-a['x'], b['y']-a['y']),
                           angle=angle))
    return result


def motor_comparison(g, params, max_age=.2):
    """One-feedback-step replay from measured wheels, with latent profile memory.

    Integration occurs only over continuous receipt history. No fitted parameters,
    inferred delay or automatic controller updates. Missing drive commands are
    never substituted with PC commands.
    """
    from pinky_soccer_dynamics.command_adapter import CommandAdapter
    from pinky_soccer_dynamics.profile_model import motor_step
    adapter = CommandAdapter(params)
    drive = g.get('drive', [])
    dtimes = [r['t'] for r in drive]
    held_since = []
    held_start = None
    for i, r in enumerate(drive):
        if i == 0 or r['t']-drive[i-1]['t'] > max_age or any(
                abs(r[k]-drive[i-1][k]) > .00001 for k in ('v', 'w')):
            held_start = r['t']
        held_since.append(held_start)
    result = []
    ref = None
    prev = None
    segment_start = None
    for r in g.get('joints', []):
        if not all(finite(r.get(k)) for k in ('raw_l', 'raw_r')):
            ref = prev = None
            continue
        measured = np.array([params.sign_left*r['raw_l'], params.sign_right*r['raw_r']])
        c = previous(drive, r['t'], max_age, dtimes)
        if c is None or not all(finite(c.get(k)) for k in ('v', 'w')):
            ref = prev = None
            continue
        u = adapter.driver_applied(c['v'], c['w'])[:2]
        pred = np.full(2, np.nan)
        if prev is not None and 0 < r['t']-prev['t'] <= max_age:
            # Integrate every command transition between wheel samples.
            start = prev['t']
            wheel = prev['wheel'].copy()
            indices = range(bisect_right(dtimes, start), bisect_right(dtimes, r['t']))
            current = previous(drive, start, max_age, dtimes)
            covered = current is not None
            for index in list(indices)+[None]:
                end = r['t'] if index is None else drive[index]['t']
                if (current is None or end-current['t'] > max_age or
                        not all(finite(current.get(k)) for k in ('v', 'w'))):
                    covered = False
                    break
                command = adapter.driver_applied(current['v'], current['w'])[:2]
                wheel, ref = motor_step(params, wheel, ref, command, end-start)
                start = end
                if index is not None:
                    current = drive[index]
            if covered:
                pred = wheel
            else:
                ref = None
        else:
            ref = None
        if ref is None:
            ref = measured.copy()
            segment_start = r['t']
        result.append(dict(t=r['t'], l=float(measured[0]), r=float(measured[1]),
                           cmd_l=float(u[0]), cmd_r=float(u[1]),
                           pred_l=float(pred[0]), pred_r=float(pred[1]),
                           warmup=r['t']-segment_start < .5,
                           steady_hold_s=r['t']-held_since[bisect_right(dtimes, r['t'])-1]))
        prev = dict(t=r['t'], wheel=measured)
    return result


def baseline():
    def git(*args):
        return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()
    files = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in BASELINE_FILES}
    unchanged = {p: hashlib.sha256(subprocess.check_output(
        ['git', '-C', str(ROOT), 'show', 'HEAD:'+p])).hexdigest() == files[p] for p in BASELINE_FILES}
    return dict(head=git('rev-parse', 'HEAD'), status=git('status', '--short'),
                files=files, unchanged_from_head=unchanged,
                analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def plots(g, solves, motor, output, context=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def line(ax, rows, key, label, scale=1., step=False):
        valid = [(r['t'], r.get(key)) for r in rows]
        if valid:
            x, y = zip(*valid)
            ax.plot(x, [v*scale if finite(v) else np.nan for v in y],
                    label=label, drawstyle='steps-post' if step else 'default', linewidth=.9)

    def finish(fig, axes, name):
        for ax in axes:
            ax.grid(alpha=.25)
            if ax.get_legend_handles_labels()[0]:
                ax.legend(loc='upper right', fontsize=7, ncol=2)
        axes[-1].set_xlabel('Recorder receipt time from first selected message [s]')
        fig.text(.01, .005, context[:180], fontsize=8)
        fig.tight_layout(rect=(0, .02, 1, .97))
        fig.savefig(output/name, dpi=150)
        plt.close(fig)

    fig, axes = plt.subplots(6, 1, figsize=(15, 15), sharex=True)
    fig.suptitle('GoToPose receipt timeline (missing topics are left blank)')
    for role in ('cmd', 'drive', 'odom'):
        line(axes[0], g.get(role, []), 'v', role, step=role!='odom')
        line(axes[1], g.get(role, []), 'w', role, step=role!='odom')
    axes[0].set_ylabel('v [m/s]')
    axes[1].set_ylabel('yaw rate [rad/s]')
    for k in ('l', 'r', 'cmd_l', 'cmd_r', 'pred_l', 'pred_r'):
        line(axes[2], motor, k, k)
    if not motor:
        for k in ('raw_l', 'raw_r'):
            line(axes[2], g.get('joints', []), k, k+' (unadjusted sign)')
    axes[2].set_ylabel('wheels [rad/s]')
    timed = [dict(r['data'], t=r['t']) for r in solves]
    for k in ('ms', 'acados_ms', 'worker_wait_ms', 'result_pickup_ms'):
        line(axes[3], timed, k, k)
    axes[3].axhline(100, color='red', ls=':', label='slot duration (not compute budget)')
    axes[3].set_ylabel('completed solves [ms]')
    status = [dict(r['data'], t=r['t']) for r in g.get('status', [])]
    line(axes[4], status, 'activation_missed', 'sticky activation_missed', step=True)
    line(axes[4], status, 'activation_jitter_sim_s', 'sticky jitter [ms]', 1000)
    axes[4].set_ylabel('flag / jitter [ms]')
    for role in ('raw_joints', 'wheel_joints', 'joints', 'odom', 'amcl', 'world'):
        line(axes[5], g.get(role, []), 'age_ms', role)
    line(axes[5], status, 'feedback_age', 'controller feedback receipt age', 1000)
    line(axes[5], status, 'wheel_source_age_s', 'controller wheel source age', 1000)
    axes[5].set_ylabel('receipt-source [ms]\nclock offset included')
    finish(fig, axes, 'timeline.png')

    fig, axes = plt.subplots(4, 1, figsize=(15, 11), sharex=True)
    fig.suptitle('Pose changes: each series retains its own frame; odom != map')
    for role in ('odom', 'amcl', 'world', 'tf'):
        rows = g.get(role, [])
        for index, key in enumerate(('x', 'y')):
            line(axes[index], rows, key, role)
        increments = pose_steps(rows)
        line(axes[2], increments, 'distance', role+' distance increment')
        line(axes[3], increments, 'angle', role+' wrapped yaw increment')
    for ax, label in zip(axes, ('x [m]', 'y [m]', 'step [m]', 'yaw step [rad]')):
        ax.set_ylabel(label)
    finish(fig, axes, 'localization.png')

    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    for role in ('cmd', 'drive', 'raw_joints', 'joints', 'world', 'status'):
        rows = g.get(role, [])
        periods = [dict(t=b['t'], period=(b['t']-a['t'])*1000) for a, b in zip(rows, rows[1:])]
        line(axes[0], periods, 'period', role)
    axes[0].set_ylabel('receipt period [ms]')
    for k in ('solve_input_age_wall_s', 'command_state_age_wall_s', 'pose_source_age_s'):
        line(axes[1], status, k, k, 1000)
    axes[1].set_ylabel('age [ms]')
    state_names = sorted({r.get('state', '') for r in status})
    for i, state in enumerate(state_names):
        rows = [dict(t=r['t'], value=i if r.get('state')==state else None) for r in status]
        line(axes[2], rows, 'value', state, step=True)
    axes[2].set_yticks(range(len(state_names)), state_names)
    axes[2].set_ylabel('controller state')
    finish(fig, axes, 'timing.png')

    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig.suptitle('Feedback chain: raw motor read -> renamed wheels -> 20 Hz restamped joint_states')
    for role in ('raw_joints', 'wheel_joints', 'joints'):
        for side in ('raw_l', 'raw_r'):
            line(axes[0], g.get(role, []), side, role+'/'+side, step=True)
        rows = g.get(role, [])
        periods = [dict(t=b['t'], period=(b['t']-a['t'])*1000) for a, b in zip(rows, rows[1:])]
        line(axes[1], periods, 'period', role)
        line(axes[2], rows, 'age_ms', role)
    axes[0].set_ylabel('raw sign speed [rad/s]')
    axes[1].set_ylabel('receipt period [ms]')
    axes[2].set_ylabel('receipt-source [ms]')
    finish(fig, axes, 'feedback.png')

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(context[:100] or 'Motor command/feedback comparison')
    for ax, side in zip(axes, ('l', 'r')):
        rows = [r for r in motor if not r['warmup']]
        ax.scatter([r['cmd_'+side] for r in rows], [r[side] for r in rows], s=3, alpha=.4)
        ax.set(xlabel='Latest reconstructed drive wheel command [rad/s]',
               ylabel='Measured wheel speed [rad/s]', title=side+' wheel (closed-loop; not a friction fit)')
        ax.grid(alpha=.25)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(output/'motor_scatter.png', dpi=150)
    plt.close(fig)


def write_report(summary, output):
    lines = ['# GoToPose offline evidence', '',
             'Root cause: **not established automatically**. Input context and completeness matter.', '',
             'All plots use recorder receipt time. Source clocks are not shifted or interpolated.', '',
             '| Topic role | Samples | Mean Hz | Receipt p95 ms | Source p95 ms |',
             '|---|---:|---:|---:|---:|']
    for role, s in summary['topics'].items():
        lines.append(f"| {role} | {s['count']} | {s['mean_hz']} | "
                     f"{s['receipt_period_ms'].get('p95')} | {s['source_period_ms'].get('p95')} |")
    lines += ['', 'Missing roles: '+', '.join(summary['missing_roles']), '',
              '## Interpretation limits', '']
    lines += ['- '+x for x in summary['conclusions']]
    lines += ['', '## Timing evidence', '', '```json', json.dumps(summary['timing'], indent=2),
              '```', '', '## Zero-command onset associations (not causality)', '',
              '```json', json.dumps(summary['zero_command_starts'][:100], indent=2), '```', '',
              'All events are retained in summary.json; normalized raw data are in samples.csv.']
    (output/'report.md').write_text('\n'.join(lines)+'\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--bag', type=Path)
    source.add_argument('--jsonl', type=Path, help='Normalized records for repeatable offline tests')
    ap.add_argument('--output', required=True, type=Path, help='New directory; refuses overwrite')
    ap.add_argument('--namespace', default='')
    ap.add_argument('--topic', action='append', default=[], metavar='ROLE=/TOPIC')
    ap.add_argument('--target', choices=('real', 'mujoco', 'gazebo'), default='real')
    ap.add_argument('--profile', help='Actual run profile; read-only; no inferred selection')
    ap.add_argument('--identification', default='',
                    help='Current real MPC response snapshot by default; none for the original 0.05 s model')
    ap.add_argument('--motor-feedback-role', choices=('joints', 'raw_joints', 'wheel_joints'),
                    default='joints', help='Default matches controller input; raw_joints avoids merger restamping')
    ap.add_argument('--left-joint', default='l_wheel_joint')
    ap.add_argument('--right-joint', default='r_wheel_joint')
    ap.add_argument('--clock-synchronized', action='store_true',
                    help='Operator confirms clocks; does not remove acquisition/buffering latency')
    ap.add_argument('--context', default='', help='Run command, floor, battery, symptom, synthetic label')
    args = ap.parse_args(argv)
    mapping = {role: '/'+('/'.join(x for x in (args.namespace.strip('/'), topic) if x))
               for role, topic in TOPICS.items()}
    for override in args.topic:
        role, topic = override.split('=', 1)
        if role not in TOPICS or not topic.startswith('/'):
            ap.error('topic override must be a known ROLE=/absolute/topic')
        mapping[role] = topic
    if len(set(mapping.values())) != len(mapping):
        ap.error('topic roles must have distinct names')
    if args.output.exists():
        ap.error('output already exists; choose a new directory')
    if args.bag:
        rows, inventory, errors = read_bag(args.bag, {v: k for k, v in mapping.items()},
                                            args.left_joint, args.right_joint)
    else:
        rows = [json.loads(line) for line in args.jsonl.read_text().splitlines() if line.strip()]
        inventory, errors = {}, {}
    if not rows:
        ap.error('no selected messages found; check namespace/topic overrides and message installation')
    g, origin = prepare(rows)
    summary, solves, comparisons = summarize(g, errors)
    summary.update(input=str(args.bag or args.jsonl), context=args.context,
                   origin_bag_ns=origin, clock_synchronized_asserted=args.clock_synchronized,
                   requested_topics=mapping, inventory=inventory, code_baseline=baseline())
    motor = []
    try:
        from pinky_soccer_dynamics import params as dyn_params
        from pinky_soccer_mpc.response_calibration import apply_response
        params = dyn_params.load(args.target, profile=args.profile)
        params, response = apply_response(params, args.identification)
        summary['analysis_model'] = params.as_dict()
        summary['analysis_response'] = response
        summary['analysis_model_note'] = 'Offline comparison assumptions; not proof of the running robot parameters'
        motor = motor_comparison(dict(g, joints=g.get(args.motor_feedback_role, [])), params)
        summary['motor_feedback_role'] = args.motor_feedback_role
        summary['motor'] = {side: dict(
            one_step_error_rad_s=stats(r[side]-r['pred_'+side] for r in motor if not r['warmup']),
            low_command_stationary_samples=sum(.02 < abs(r['cmd_'+side]) < 1 and abs(r[side]) < .05
                                               and r['steady_hold_s'] >= .3
                                               for r in motor if not r['warmup'])) for side in ('l', 'r')}
    except ImportError as exc:
        summary['motor_unavailable'] = str(exc)+'; source install/setup.bash to enable model replay'
    summary['pose_increments'] = {role: dict(
        distance_m=stats(x['distance'] for x in pose_steps(g.get(role, []))),
        wrapped_yaw_rad=stats(abs(x['angle']) for x in pose_steps(g.get(role, []))))
        for role in ('odom', 'amcl', 'world', 'tf')}
    args.output.mkdir(parents=True, exist_ok=False)
    # CSV stores one row per message, no artificial common-rate resampling.
    fields = sorted({key for rows_ in g.values() for r in rows_ for key in r})
    with (args.output/'samples.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        for r in sorted((r for rows_ in g.values() for r in rows_), key=lambda r: r['bag_ns']):
            writer.writerow({k: json.dumps(clean_json(v), allow_nan=False) if isinstance(v, dict) else v
                             for k, v in r.items()})
    # No JSON NaN: missing model predictions are represented by null.
    (args.output/'summary.json').write_text(json.dumps(clean_json(summary), indent=2, allow_nan=False)+'\n')
    write_report(summary, args.output)
    plots(g, solves, motor, args.output, args.context or str(args.bag or args.jsonl))
    print(f'Wrote {args.output}: {sum(map(len, g.values()))} messages; missing: {summary["missing_roles"]}')
    if errors:
        print('Decode errors recorded in summary.json; affected topics are incomplete.')


if __name__ == '__main__':
    main()
