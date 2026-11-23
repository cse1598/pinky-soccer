#!/usr/bin/env python3
r"""
Odometry calibration of a running simulator, with the calibration UI's own procedure.

The robot is driven open-loop exactly like the UI wheel experiments (constant
MOTION_LINEAR_SPEED / MOTION_ANGULAR_SPEED until /odom reports 1 m or 360 deg, 60 s limit),
the wheel encoder change comes from /joint_states, and the "tape measure" is the simulator
ground truth: the start-to-end chord of the bumper front centre for straight runs and the
accumulated true yaw for turns. Trials, fit (distance_rotation_fit) and export are the
pinky_soccer_calibration Session code, so the profile has the same format as a real one
(environment: rehearsal, robot_id: <simulator>-sim).

The result changes only drive constants (cmd_vel conversion and encoder odometry); the
simulated wheel geometry stays as modelled. No feedback control is involved.

    ros2 launch pinky_soccer_sim mujoco_soccer.launch.py field:=hardboard viewer:=false \\
        camera:=false spawn_ball:=false calibration_profile:=none
    ros2 run pinky_soccer_sim calibrate_odometry.py --simulator mujoco --install

`--verify` drives the same runs and only reports odometry vs. truth (no export).
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

TARGET_DISTANCE = 1.0          # m, as the calibration UI
TARGET_ANGLE = 2.0 * math.pi   # rad, as the calibration UI
TIME_LIMIT = 60.0              # s, as the calibration UI
SETTLE = 1.0                   # s stationary before and after each run


def yaw_of(q):
    """Yaw of a geometry_msgs quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def bumper_chord(start, end, front):
    """Signed start-to-end distance of the bumper front centre (pose = x, y, yaw)."""
    def point(p):
        c, s = math.cos(p[2]), math.sin(p[2])
        return (p[0] + c * front[0] - s * front[1], p[1] + s * front[0] + c * front[1])
    a, b = point(start), point(end)
    dx, dy = b[0] - a[0], b[1] - a[1]
    sign = 1.0 if dx * math.cos(start[2]) + dy * math.sin(start[2]) >= 0 else -1.0
    return sign * math.hypot(dx, dy)


def true_turn(odom_angle, truth_start_yaw, truth_end_yaw):
    """Accumulated true yaw, unwrapped with the odometry turn (valid while |error| < pi)."""
    return odom_angle + wrap((truth_end_yaw - truth_start_yaw) - odom_angle)


def gazebo_pose(model):
    """Return (x, y, yaw) of a Gazebo model via the gz CLI."""
    out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True,
                         timeout=20).stdout
    rows = re.findall(r'\[\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s*\]', out)
    if len(rows) < 2:
        raise RuntimeError(f'gz model -m {model} -p: no pose in output')
    return float(rows[0][0]), float(rows[0][1]), float(rows[1][2])


class Runner(Node):
    """Open-loop calibration runs against simulator ground truth."""

    def __init__(self, simulator, robot_name):
        super().__init__('calibrate_odometry')
        from pinky_soccer_calibration.suite_ros import MOTION_ANGULAR_SPEED, MOTION_LINEAR_SPEED
        self.linear, self.angular = MOTION_LINEAR_SPEED, MOTION_ANGULAR_SPEED
        self.simulator, self.robot_name = simulator, robot_name
        self.odom = self.joints = self.truth_msg = self.description = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(PoseStamped, '/ground_truth',
                                 lambda m: setattr(self, 'truth_msg', m), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description',
                                 lambda m: setattr(self, 'description', m.data), latched)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if 'l_wheel_joint' in pos and 'r_wheel_joint' in pos:
            self.joints = (float(pos['l_wheel_joint']), float(pos['r_wheel_joint']))

    def spin_for(self, seconds, twist=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if twist is not None:
                self.cmd.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout=60.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            truth_ok = self.simulator == 'gazebo' or self.truth_msg is not None
            if self.odom and self.joints and self.description and truth_ok:
                return
        raise RuntimeError('no /odom, /joint_states, /robot_description or ground truth')

    def truth(self):
        if self.simulator == 'gazebo':
            return gazebo_pose(self.robot_name)
        self.spin_for(0.1)
        p = self.truth_msg.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def odom_pose(self):
        p = self.odom.pose.pose
        return [p.position.x, p.position.y, yaw_of(p.orientation)]

    def stop(self):
        self.spin_for(SETTLE, Twist())

    def drive(self, kind, distance=TARGET_DISTANCE):
        """UI-identical motion; returns the trial dict like suite_ros.stop() leaves it."""
        start = self.odom_pose()
        m = {'kind': kind, 'start': start, 'last_yaw': start[2], 'angle': 0.0,
             'joints': list(self.joints), 'started': time.time()}
        rotating = kind in ('left', 'right')
        msg = Twist()
        if rotating:
            msg.angular.z = self.angular if kind == 'left' else -self.angular
        else:
            msg.linear.x = self.linear if kind == 'forward' else -self.linear
        t0 = time.monotonic()
        reason = None
        while reason is None:
            rclpy.spin_once(self, timeout_sec=0.02)
            pose = self.odom_pose()
            m['angle'] += wrap(pose[2] - m['last_yaw'])
            m['last_yaw'] = pose[2]
            moved = math.hypot(pose[0] - start[0], pose[1] - start[1])
            if (abs(m['angle']) >= TARGET_ANGLE) if rotating else (moved >= distance):
                reason = '목표 도달'
            elif time.monotonic() - t0 > TIME_LIMIT:
                reason = '60초 제한'
            else:
                self.cmd.publish(msg)
        for _ in range(3):
            self.cmd.publish(Twist())
        m['reason'], m['complete'] = reason, reason == '목표 도달'
        self.stop()
        end = self.joints
        m['left_rad'], m['right_rad'] = end[0] - m['joints'][0], end[1] - m['joints'][1]
        m['odom_distance_m'] = math.hypot(self.odom_pose()[0] - start[0],
                                          self.odom_pose()[1] - start[1])
        return m


def run(args):
    from pinky_soccer_calibration.measurement_geometry import MeasurementGeometry
    from pinky_soccer_calibration.wheel_measurement import (distance_measurement,
                                                            rotation_measurement)
    rclpy.init()
    node = Runner(args.simulator, args.robot_name)
    try:
        node.wait_ready()
        reference = MeasurementGeometry(node.description).wheel_reference()
        front = reference['bumper_front_base_xyz_m']
        node.stop()
        # Positioning move (not recorded): back up so the 1 m runs stay inside the field.
        node.drive('backward', args.start_offset)
        trials, report = [], []
        for _ in range(args.repeats):
            for kind in ('forward', 'backward', 'left', 'right'):
                truth0 = node.truth()
                t = node.drive(kind)
                truth1 = node.truth()
                if not t['complete']:
                    raise RuntimeError(f'{kind} run stopped: {t["reason"]}')
                t['measurement_source'] = 'simulator_ground_truth'
                t['ground_truth_start'], t['ground_truth_end'] = list(truth0), list(truth1)
                if kind in ('forward', 'backward'):
                    t['measurement_geometry'] = reference
                    measured = bumper_chord(truth0, truth1, front)
                    t.update(distance_measurement(kind, measured, reference))
                    report.append({'kind': kind, 'odom_m': round(math.copysign(
                        t['odom_distance_m'], measured), 4), 'truth_m': round(measured, 4)})
                else:
                    angle = true_turn(t['angle'], truth0[2], truth1[2])
                    t.update(rotation_measurement(kind, math.degrees(angle)))
                    report.append({'kind': kind, 'odom_deg': round(math.degrees(t['angle']), 2),
                                   'truth_deg': round(math.degrees(angle), 2)})
                t.update(id=uuid.uuid4().hex,
                         recorded_utc=datetime.now(timezone.utc).isoformat())
                trials.append(t)
        summary = summarize(report)
        print(json.dumps({'runs': report, 'summary': summary}, indent=1, ensure_ascii=False))
        if args.verify:
            return 0
        return export(args, trials, summary)
    finally:
        for _ in range(3):
            node.cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


def summarize(report):
    """Mean relative odometry error per experiment kind (positive = odom over-reports)."""
    out = {}
    for kind in ('forward', 'backward', 'left', 'right'):
        rows = [r for r in report if r['kind'] == kind]
        if not rows:
            continue
        if 'odom_m' in rows[0]:
            err = [r['odom_m'] / r['truth_m'] - 1 for r in rows]
        else:
            err = [r['odom_deg'] / r['truth_deg'] - 1 for r in rows]
        out[kind] = {'mean_relative_error': round(sum(err) / len(err), 4),
                     'max_abs_relative_error': round(max(abs(e) for e in err), 4)}
    return out


def export(args, trials, summary):
    """Store trials with the UI Session and export profile.yaml in the UI format."""
    from pinky_soccer_calibration.suite_model import Session
    session = Session(Path(args.out_dir).expanduser(), 'rehearsal', f'{args.simulator}-sim')
    session.meta['measurement_source'] = 'simulator_ground_truth'
    session.meta['simulator'] = args.simulator
    session.meta['calibration_scope'] = ('drive constants only (cmd_vel conversion and '
                                         'encoder odometry); simulated wheel geometry unchanged')
    session.meta['pre_calibration_odometry_error'] = summary
    session.trials.extend(trials)
    session.write('wheel_straight_trials.yaml',
                  [t for t in trials if t['kind'] in ('forward', 'backward')])
    session.write('wheel_rotation_trials.yaml',
                  [t for t in trials if t['kind'] in ('left', 'right')])
    session.write('wheel_trials.yaml', session.trials)
    session.recalculate('wheels')
    profile = session.export(True)
    print(f'profile: {profile}')
    print(json.dumps({k: session.results['wheels'][k] for k in
                      ('effective_radius_m', 'separation_m', 'trials')}, indent=1))
    if args.install:
        target = Path(args.install_dir).expanduser() / f'{args.simulator}_profile.yaml'
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(profile, target)
        print(f'installed: {target}')
    return 0


def main(argv=None):
    """CLI."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--simulator', required=True, choices=['mujoco', 'gazebo'])
    ap.add_argument('--robot-name', default='pinky', help='Gazebo model name')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--start-offset', type=float, default=0.5,
                    help='unrecorded backward positioning move [m]')
    ap.add_argument('--out-dir', default='~/pinky-soccer/calibration_results/simulator')
    ap.add_argument('--install', action='store_true',
                    help='copy the profile to --install-dir as <simulator>_profile.yaml')
    ap.add_argument('--install-dir', default=str(
        Path(__file__).resolve().parents[1] / 'config' / 'calibration'))
    ap.add_argument('--verify', action='store_true', help='report odometry error only')
    args = ap.parse_args(argv)
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
