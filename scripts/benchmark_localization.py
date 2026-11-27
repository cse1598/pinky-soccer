#!/usr/bin/env python3
"""Isolated simulator evaluation; truth is read only by this evaluator, never fed back.

Run after sourcing ROS/workspace: .venv/bin/python scripts/benchmark_localization.py
--sim mujoco --localization rectangle --drive --output /tmp/loc-run
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import shlex
import subprocess
import time
import uuid

import numpy as np
import cv2
import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

LOCAL_DDS = ('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/>'
             '</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery>'
             '<ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="127.0.0.1"/>'
             '</Peers></Discovery></Domain></CycloneDDS>')


def is_localizer_command(arguments):
    names = {'amcl', 'goal_heading', 'rectangle_localization'}
    return bool(arguments and (Path(arguments[0]).name in names or (
        Path(arguments[0]).name.startswith('python') and len(arguments) > 1
        and Path(arguments[1]).name in names)))


def seconds(header):
    return header.stamp.sec+header.stamp.nanosec/1e9


def pose(message):
    q = message.orientation
    return [message.position.x, message.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


def summarize(result):
    summary = {'ready_wall_s': result['ready_wall_s'], 'sources': {}}
    for source in ('amcl', 'rectangle'):
        rows = [s for s in result['samples'] if s['source'] == source]
        if not rows:
            continue
        error = np.asarray([np.asarray(s['pose'])-s['truth'] for s in rows])
        error[:, 2] = np.arctan2(np.sin(error[:, 2]), np.cos(error[:, 2]))
        times = np.asarray([s['t'] for s in rows])
        summary['sources'][source] = dict(
            samples=len(rows), xy_rmse_m=np.sqrt(np.mean(error[:, :2]**2, axis=0)).tolist(),
            position_rmse_m=float(np.sqrt(np.mean(np.sum(error[:, :2]**2, axis=1)))),
            yaw_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(error[:, 2]**2)))),
            pose_hz=float((len(times)-1)/(times[-1]-times[0])) if times[-1] > times[0] else None)
    terminal = {}
    for row in result['behavior']:
        if row.get('state') in ('success', 'failure', 'error', 'timeout', 'world_model_stale'):
            terminal[row.get('request_id')] = row['state']
    summary['behavior_results'] = terminal
    per_pid = {}
    for row in result['cpu']:
        if is_localizer_command(row.get('argv', shlex.split(row['cmd']))):
            per_pid.setdefault(row['pid'], []).append(row)
    summary['localization_cpu_percent_one_core'] = sum(
        100*(entries[-1]['cpu']-entries[0]['cpu'])/(entries[-1]['elapsed']-entries[0]['elapsed'])
        for entries in per_pid.values() if len(entries) > 1)
    states = result['states']
    first_ready = next((row for row in states if row.get('ready')), None)
    summary['initial_rotation_imu_rad'] = first_ready['rotation_rad'] if first_ready else None
    summary['last_status'] = states[-1] if states else None
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', choices=['mujoco', 'gazebo'], default='mujoco')
    parser.add_argument('--localization', choices=['rectangle', 'lidar'], default='rectangle')
    parser.add_argument('--output', type=Path, required=True, help='new directory; never overwrite')
    parser.add_argument('--domain', type=int, default=198)
    parser.add_argument('--drive', action='store_true', help='two existing GoToPose requests')
    parser.add_argument('--goal1', nargs=3, type=float, default=[-.1, 0., 0.])
    parser.add_argument('--goal2', nargs=3, type=float, default=[-.1, 0., 1.])
    parser.add_argument('--opponent', action='store_true', help='static simulated opponent, sensor occlusion')
    parser.add_argument('--avoid-walls', action='store_true')
    parser.add_argument('--avoid-opponents', action='store_true')
    parser.add_argument('--duration', type=float, default=35., help='wall seconds after ready')
    parser.add_argument('--startup-timeout', type=float, default=40.)
    parser.add_argument('--extra', nargs='*', default=[], help='additional existing launch arguments')
    args = parser.parse_args()
    if not 1 <= args.domain <= 232 or args.duration < 0 or args.startup_timeout <= 0:
        parser.error('Invalid isolated domain or time budget')
    allowed = {'field', 'initial_x', 'initial_y', 'initial_yaw', 'localization_config',
               'localization_debug', 'ball_x', 'ball_y', 'max_lin', 'max_ang'}
    if any(':=' not in text or text.split(':=', 1)[0] not in allowed for text in args.extra):
        parser.error('--extra permits only field/start pose/localization/ball/speed settings')
    args.output.mkdir(parents=True, exist_ok=False)
    # Process-local environment. Do not edit the user's ROS/network configuration.
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), CYCLONEDDS_URI=LOCAL_DDS,
                      GZ_PARTITION='pinky_loc_'+uuid.uuid4().hex,
                      ROS_LOG_DIR=str(args.output/'ros_logs'))
    os.environ['AMENT_PREFIX_PATH'] = ':'.join(
        p for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':')
        if not p.endswith('/install/pinky_soccer_skills'))
    log = (args.output/'launch.log').open('w')
    command = ['ros2', 'launch', 'pinky_soccer_viz', 'game.launch.py', f'mode:={args.sim}',
               f'localization:={args.localization}', 'headless:=true', 'verify:=true', 'preparation:=false',
               'field:=hardboard', f'heading_record:={args.output.resolve()}/records/heading.yaml',
               *args.extra]
    if args.opponent:
        command += ['robots:=2', 'robot_name:=loc_eval', 'opponent_name:=blocker',
                    'opponent_mode:=static', 'opponent_x:=0.2', 'opponent_y:=0.0']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('localization_evaluator', namespace='/loc_eval' if args.opponent else '/')
    truth, rows, states, statuses, cpu, commands, truths = deque(maxlen=1000), [], [], [], [], {}, []
    saved_at = {}

    def save_debug(message, name):
        now = time.monotonic()
        if now-saved_at.get(name, -math.inf) < 1.:
            return
        if message.encoding != 'bgr8':
            return
        pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        image = pixels[:, :message.width*3].reshape(message.height, message.width, 3)
        first = args.output/(name+'_first.png')
        if not first.exists():
            cv2.imwrite(str(first), image)
        cv2.imwrite(str(args.output/(name+'_latest.png')), image)
        saved_at[name] = now

    def on_truth(message):
        t, p = seconds(message.header), pose(message.pose)
        truth.append((t, p))
        truths.append([t, *p])

    def sample(message, source):
        t, p = seconds(message.header), pose(message.pose.pose)
        if truth:
            gt = min(truth, key=lambda x: abs(x[0]-t))
            if abs(gt[0]-t) < .1:
                rows.append(dict(t=t, source=source, pose=p, truth=gt[1], sync_error=abs(gt[0]-t)))

    node.create_subscription(PoseStamped, 'ground_truth', on_truth, qos_profile_sensor_data)
    for name in ('map_image', 'goal_image'):
        node.create_subscription(Image, 'localization/'+name,
                                 lambda m, key=name: save_debug(m, key), qos_profile_sensor_data)
    for topic, source in [('amcl_pose', 'amcl'), ('localization/pose', 'rectangle')]:
        node.create_subscription(PoseWithCovarianceStamped, topic,
                                 lambda m, s=source: sample(m, s), qos_profile_sensor_data)
    node.create_subscription(String, 'localization/status', lambda m: states.append(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, 'behavior_test/status', lambda m: statuses.append(json.loads(m.data)), 10)
    for topic in ('cmd_vel', 'drive/cmd_vel', 'drive/motor_cmd_vel'):
        node.create_subscription(Twist, topic, lambda m, k=topic: commands.update(
            {k: [m.linear.x, m.angular.z]}), qos_profile_sensor_data)
    client = node.create_client(Trigger, 'localization/calibrate')
    request = node.create_publisher(String, 'behavior_test/request', 10)
    start, ready, calibrated, leg, last_cpu = time.monotonic(), None, False, 0, 0.
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=.02)
            elapsed = time.monotonic()-start
            if process.poll() is not None or (ready is None and elapsed > args.startup_timeout):
                break
            if ready is None and states and states[-1].get('state') in (
                    'GOAL_COLOR_FAILED', 'CALIBRATION_TIMEOUT', 'ABORTED'):
                break
            if elapsed-last_cpu > 1:
                (args.output/'live.json').write_text(json.dumps(dict(
                    elapsed=elapsed, state=states[-1] if states else None, commands=commands,
                    truth=truths[-1] if truths else None)))
                for child in psutil.Process(process.pid).children(recursive=True):
                    try:
                        arguments = child.cmdline()
                        if is_localizer_command(arguments):
                            cpu.append(dict(elapsed=elapsed, pid=child.pid, cmd=' '.join(arguments),
                                            argv=arguments,
                                            cpu=sum(child.cpu_times()[:2])))
                    except psutil.Error:
                        pass
                last_cpu = elapsed
            if args.localization == 'rectangle' and not calibrated and truth and client.service_is_ready():
                client.call_async(Trigger.Request())
                calibrated = True
            good = bool(states and states[-1].get('ready')) if args.localization == 'rectangle' else bool(rows and elapsed > 10)
            if ready is None and good:
                ready = elapsed
            if ready is not None:
                dt = elapsed-ready
                if args.drive and ((leg == 0 and dt > 2) or (leg == 1 and dt > 16)):
                    goal = args.goal1 if leg == 0 else args.goal2
                    ports = dict(zip(('x', 'y', 'theta'), (str(v) for v in goal)))
                    if args.avoid_walls:
                        ports['avoid_walls'] = 'true'
                    if args.avoid_opponents:
                        ports['avoid_opponents'] = 'true'
                    request.publish(String(data=json.dumps(dict(command='run', id=9101+leg,
                        target='GoToPose', ports=ports,
                        timeout_sec=20.))))
                    leg += 1
                if dt > args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()
    result = dict(ready_wall_s=ready, samples=rows, states=states, behavior=statuses,
                  cpu=cpu, commands=commands, truth_samples=truths, launch_command=command,
                  arguments={**vars(args), 'output': str(args.output)})
    (args.output/'raw.json').write_text(json.dumps(result))
    summary = summarize(result)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
