#!/usr/bin/env python3
"""Isolated simulator preparation test. Truth is evaluation only, never initialization."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
from std_srvs.srv import Trigger
from benchmark_localization import LOCAL_DDS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', choices=['mujoco', 'gazebo'], required=True)
    parser.add_argument('--strategy', action='store_true')
    parser.add_argument('--team', choices=['Y', 'B'], default='Y')
    parser.add_argument('--domain', type=int, default=222)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--initial-yaw', type=float)
    parser.add_argument('--initial-x', type=float)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), CYCLONEDDS_URI=LOCAL_DDS,
        GZ_PARTITION='pinky_prep_'+uuid.uuid4().hex, ROS_LOG_DIR=str(args.output/'ros_logs'))
    os.environ['AMENT_PREFIX_PATH'] = ':'.join(p for p in os.environ.get('AMENT_PREFIX_PATH','').split(':')
                                             if not p.endswith('/install/pinky_soccer_skills'))
    command = ['ros2', 'launch', 'pinky_soccer_viz', 'game.launch.py', f'mode:={args.sim}',
        'headless:=true', f'verify:={str(not args.strategy).lower()}', 'field:=hardboard', f'team:={args.team}',
        f'heading_record:={args.output.resolve()}/records/heading.yaml']
    if args.initial_x is not None:
        command += [f'initial_x:={args.initial_x}']
    if args.initial_yaw is not None:
        command += [f'initial_yaw:={args.initial_yaw}']
    log = (args.output/'launch.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('preparation_evaluator')
    states, leds, commands, truth, results, odometry, goals, world = [], [], [], [], [], [], [], []
    sensors = dict(scan=0, camera=0, imu=0, odom=0)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    start = time.monotonic()
    node.create_subscription(String, 'preparation/status', lambda m: states.append(
        dict(wall=time.monotonic()-start, **json.loads(m.data))), latched)
    node.create_subscription(String, 'preparation/led', lambda m: leds.append(
        dict(wall=time.monotonic()-start, **json.loads(m.data))), latched)
    node.create_subscription(BehaviorGoal, 'behavior_goal', lambda m: goals.append(
        [time.monotonic()-start, m.branch, m.action, m.mode]),10)
    node.create_subscription(WorldModel, 'world_model', lambda m: world.append(
        [time.monotonic()-start, m.playing]),10)
    node.create_subscription(Odometry, 'odom', lambda m: odometry.append(
        [time.monotonic()-start, m.twist.twist.linear.x, m.twist.twist.angular.z]),qos_profile_sensor_data)
    node.create_subscription(Twist, 'cmd_vel', lambda m: commands.append(
        [time.monotonic()-start, m.linear.x, m.angular.z]), 10)
    node.create_subscription(PoseStamped, 'ground_truth', lambda m: truth.append(
        [time.monotonic()-start, m.pose.position.x, m.pose.position.y]), qos_profile_sensor_data)
    for key, msg, topic in [('scan', LaserScan, 'scan'), ('camera', Image, 'camera/image_raw'),
                            ('imu', Imu, 'imu_raw'), ('odom', Odometry, 'odom')]:
        node.create_subscription(msg, topic, lambda m, k=key: sensors.update({k:sensors[k]+1}),
                                 qos_profile_sensor_data)
    clients = {k:node.create_client(Trigger,'preparation/'+k) for k in ('prepare','start','stop')}
    prepared = None
    started = stopped = False
    early_sent = False
    ready_time = None
    try:
        while time.monotonic()-start < 100:
            rclpy.spin_once(node, timeout_sec=.02)
            elapsed = time.monotonic()-start
            if process.poll() is not None:
                break
            state = states[-1]['state'] if states else None
            if not early_sent and clients['start'].service_is_ready():
                future = clients['start'].call_async(Trigger.Request())
                future.add_done_callback(lambda f: results.append(['early_start', f.result().success]))
                early_sent = True
            if prepared is None and elapsed > 6 and all(sensors.values()) and clients['prepare'].service_is_ready():
                clients['prepare'].call_async(Trigger.Request())
                prepared = elapsed
                (args.output/'idle.json').write_text(json.dumps(dict(sensors=sensors,
                    commands=commands, states=states, results=results)))
            if state == 'ERROR':
                break
            if state == 'READY' and ready_time is None:
                ready_time = elapsed
            if state == 'READY' and not started and elapsed-ready_time > 1:
                clients['start'].call_async(Trigger.Request())
                started = True
            if state == 'PLAYING' and started and elapsed-ready_time > 3 and not stopped:
                clients['stop'].call_async(Trigger.Request())
                stopped = True
            if state == 'STOPPED' and stopped and elapsed-ready_time > 4:
                break
            (args.output/'live.json').write_text(json.dumps(dict(elapsed=elapsed,
                state=states[-1] if states else None, sensors=sensors)))
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.try_shutdown()
        log.close()
    summary = dict(sim=args.sim, team=args.team, prepared_signal_wall_s=prepared,
        observed_states=list(dict.fromkeys(s['state'] for s in states)),
        early_start_result=results,
        idle_nonzero=sum(abs(v)+abs(w)>1e-9 for t,v,w in commands if prepared is None or t<prepared),
        ready_reached=ready_time is not None,
        last_status=states[-1] if states else None, sensors=sensors)
    (args.output/'result.json').write_text(json.dumps(dict(summary=summary,states=states,leds=leds,
        commands=commands,truth=truth,odom=odometry,goals=goals,world=world,launch_command=command),indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
