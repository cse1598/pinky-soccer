#!/usr/bin/env python3
"""Isolated native simulator check. Evaluator truth never enters perception/control.

Launches a separate localhost DDS domain; calibration can rotate the simulated robot.
--drive also sends existing GoToPose test requests. No hardware mode is provided.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import PoseStamped
from pinky_soccer_msgs.msg import WorldModel
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from benchmark_localization import LOCAL_DDS


def evaluation_metrics(rows, truth):
    """Evaluation only: compare predicted surface location to axle truth (biased)."""
    if len(truth) < 3:
        return {}
    gt = np.asarray([[t, *p] for t, p in truth], float)
    errors, velocities, sign_ok = [], [], []
    for row in rows:
        t = row['t']
        if not row['opponents'] or t < gt[0, 0]+3 or t > gt[-1, 0]:
            continue
        window = gt[abs(gt[:, 0]-t) <= .25]
        if len(window) < 4 or np.ptp(window[:, 0]) < .1:
            continue
        o = row['opponents'][0]
        position = np.array([np.interp(t, gt[:, 0], gt[:, j]) for j in (1, 2)])
        velocity = np.linalg.lstsq(np.column_stack((window[:, 0]-t,
            np.ones(len(window)))), window[:, 1:], rcond=None)[0][0]
        estimated = np.asarray(o['velocity'])
        errors.append(np.sum((np.asarray(o['position'])-position)**2))
        velocities.append(np.sum((estimated-velocity)**2))
        if np.linalg.norm(velocity) > .03 and np.linalg.norm(estimated) > .02:
            sign_ok.append(float(np.dot(estimated, velocity) > 0))
    return dict(axle_centre_rmse_including_surface_bias_m=float(np.sqrt(np.mean(errors)))
                if errors else None,
                velocity_rmse_mps=float(np.sqrt(np.mean(velocities))) if velocities else None,
                movement_sign_agreement=float(np.mean(sign_ok)) if sign_ok else None,
                metric_samples=len(errors))


def plot_evaluation(rows, truth, output):
    """Evaluation artifact only; plotting is outside the timed detector."""
    if len(truth) < 3:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    gt = np.asarray([[t, *p] for t, p in truth], float)
    observed = [r for r in rows if r['opponents']]
    if not observed:
        return
    t = np.array([r['t'] for r in observed])
    y = np.array([r['opponents'][0]['position'][1] for r in observed])
    vy = np.array([r['opponents'][0]['velocity'][1] for r in observed])
    # Centred 0.5 s local linear slope of evaluation truth, also used for RMSE.
    truth_v = []
    for stamp in t:
        window = gt[abs(gt[:, 0]-stamp) <= .25]
        truth_v.append(np.linalg.lstsq(np.column_stack((window[:, 0]-stamp,
            np.ones(len(window)))), window[:, 2], rcond=None)[0][0] if len(window) > 3
            else float('nan'))
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
    ax[0].plot(gt[:, 0], gt[:, 2], label='Axle ground truth (evaluation only)')
    ax[0].plot(t, y, label='Tracked observed surface')
    ax[0].set(ylabel='Map y [m]')
    ax[0].legend()
    ax[1].plot(t, truth_v, label='Ground truth local velocity')
    ax[1].plot(t, vy, label='Alpha-beta velocity')
    ax[1].set(xlabel='ROS time [s]', ylabel='Map vy [m/s]')
    ax[1].legend()
    fig.savefig(output, dpi=140)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sim', choices=['mujoco', 'gazebo'], required=True)
    p.add_argument('--opponent', choices=['none', 'static', 'lateral', 'longitudinal'], default='none')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--domain', type=int, default=218)
    p.add_argument('--duration', type=float, default=10.)
    p.add_argument('--drive', action='store_true')
    p.add_argument('--initial-yaw', type=float, default=None)
    args = p.parse_args()
    if not 1 <= args.domain <= 232 or args.duration <= 0:
        p.error('invalid domain or duration')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), CYCLONEDDS_URI=LOCAL_DDS,
                      GZ_PARTITION='pinky_opp_'+uuid.uuid4().hex,
                      ROS_LOG_DIR=str(args.output/'ros_logs'))
    os.environ['AMENT_PREFIX_PATH'] = ':'.join(x for x in os.environ.get('AMENT_PREFIX_PATH', '').split(':')
                                             if not x.endswith('/install/pinky_soccer_skills'))
    command = ['ros2', 'launch', 'pinky_soccer_viz', 'game.launch.py', f'mode:={args.sim}',
        'headless:=true', 'verify:=true', 'preparation:=false', 'field:=hardboard',
        f'heading_record:={args.output.resolve()}/records/heading.yaml']
    if args.initial_yaw is not None:
        command += [f'initial_yaw:={args.initial_yaw}']
    multi = args.opponent != 'none'
    if multi:
        command += ['robots:=2', 'robot_name:=observer', 'opponent_name:=blocker',
            f'opponent_mode:={args.opponent}', 'opponent_x:=0.2', 'opponent_y:=0.0',
            'opponent_speed:=0.08', 'opponent_travel:=0.30']
    log = (args.output/'launch.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('opponent_evaluator', namespace='/observer' if multi else '/')
    truth, self_truth = deque(maxlen=3000), deque(maxlen=3000)
    rows, raw, states, scans, behavior, cpu, processing, mpc = [], [], [], [], [], [], [], []

    def timestamp(m):
        return m.header.stamp.sec+m.header.stamp.nanosec/1e9

    def on_truth(m, store):
        store.append((timestamp(m), [m.pose.position.x, m.pose.position.y]))

    def on_world(m):
        stamp = timestamp(m)
        row = dict(t=stamp, opponents=[dict(id=o.id, position=[o.pose.x, o.pose.y],
            velocity=[o.velocity.x, o.velocity.y], state=o.state,
            last_seen=o.last_seen.sec+o.last_seen.nanosec/1e9) for o in m.opponents])
        if truth:
            nearest = min(truth, key=lambda v: abs(v[0]-stamp))
            if abs(nearest[0]-stamp) < .15:
                row['truth'] = nearest
        rows.append(row)

    node.create_subscription(WorldModel, 'world_model', on_world, 10)
    node.create_subscription(String, 'opponents/status',
                             lambda m: processing.append(json.loads(m.data)), 5)
    node.create_subscription(WorldModel, 'world_model/obstacles', lambda m: raw.append(
        [timestamp(m), len(m.opponents)]), 10)
    node.create_subscription(PoseStamped, 'ground_truth', lambda m: on_truth(m, self_truth),
                             qos_profile_sensor_data)
    if multi:
        node.create_subscription(PoseStamped, '/blocker/ground_truth', lambda m: on_truth(m, truth),
                                 qos_profile_sensor_data)
    node.create_subscription(String, 'mpc/status', lambda m: mpc.append(json.loads(m.data)), 10)
    node.create_subscription(String, 'localization/status', lambda m: states.append(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, 'behavior_test/status', lambda m: behavior.append(json.loads(m.data)), 10)
    node.create_subscription(LaserScan, 'scan', lambda m: scans.append(timestamp(m)), qos_profile_sensor_data)
    calibrate = node.create_client(Trigger, 'localization/calibrate')
    request = node.create_publisher(String, 'behavior_test/request', 10)
    start, ready, signalled, leg, cpu_time = time.monotonic(), None, False, 0, 0.
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=.02)
            elapsed = time.monotonic()-start
            if process.poll() is not None or (ready is None and elapsed > 45):
                break
            if not signalled and len(scans) > 3 and self_truth and calibrate.service_is_ready():
                calibrate.call_async(Trigger.Request())
                signalled = True
            if ready is None and states and states[-1].get('ready'):
                ready = elapsed
            if elapsed-cpu_time > 1:
                cpu_time = elapsed
                for child in psutil.Process(process.pid).children(recursive=True):
                    try:
                        argv = child.cmdline()
                        if any(Path(x).name == 'world_model_node' for x in argv[:2]):
                            cpu.append(dict(t=elapsed, pid=child.pid, cpu=sum(child.cpu_times()[:2])))
                    except psutil.Error:
                        pass
            if ready is not None:
                dt = elapsed-ready
                if args.drive and ((leg == 0 and dt > 2) or (leg == 1 and dt > 16)):
                    goal = [-.25, 0., 0.] if leg == 0 else [-.25, 0., 1.]
                    request.publish(String(data=json.dumps(dict(command='run', id=9201+leg,
                        target='GoToPose', ports={**dict(zip(('x', 'y', 'theta'), map(str, goal))),
                        'avoid_walls': 'true', 'avoid_opponents': 'true'}, timeout_sec=20.))))
                    leg += 1
                if dt > args.duration:
                    break
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.try_shutdown()
        log.close()
    steady = [x for x in rows if rows and x['t'] > rows[0]['t']+3]
    counted = [x for x in steady if x['opponents']]
    unique = np.unique(scans)
    summary = dict(sim=args.sim, mode=args.opponent, ready_wall_s=ready, scan_count=len(unique),
        scan_hz=float(1/np.median(np.diff(unique))) if len(unique) > 1 else None,
        world_samples=len(rows), confirmed_fraction=len(counted)/max(1, len(steady)),
        multi_opponent_count=sum(len(x['opponents']) > 1 for x in rows),
        max_raw_count=max((x[1] for x in raw), default=0),
        speed_p95_mps=float(np.percentile([math.hypot(*x['opponents'][0]['velocity'])
                                          for x in counted], 95)) if counted else None,
        last_status=states[-1] if states else None,
        behavior_results=list({b.get('request_id'): dict(request_id=b.get('request_id'),
            state=b['state'], elapsed=b.get('elapsed')) for b in behavior
            if b.get('state') in ('success', 'failure', 'timeout', 'error')}.values()),
        processing=processing[-1] if processing else None)
    if len(cpu) > 1:
        summary['world_model_cpu_percent_one_core'] = 100*(cpu[-1]['cpu']-cpu[0]['cpu'])/(cpu[-1]['t']-cpu[0]['t'])
    result = dict(summary=summary, rows=rows, raw=raw, states=states, behavior=behavior,
                  scans=scans, processing=processing, mpc=mpc, opponent_truth=list(truth), self_truth=list(self_truth), cpu=cpu,
                  launch_command=command)
    summary.update(evaluation_metrics(rows, list(truth)))
    (args.output/'result.json').write_text(json.dumps(result, indent=2))
    plot_evaluation(rows, list(truth), args.output/'timeline.png')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
