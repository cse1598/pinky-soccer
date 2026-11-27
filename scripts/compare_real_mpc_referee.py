#!/usr/bin/env python3
"""Explicit real-robot comparison; overhead topics are evaluation only, no images.

Uses existing preparation/BT interfaces and unchanged robot MPC constraints.
Only its own PC launch children are started/stopped. Robot/sensor/referee nodes
must already be running. Invalid overhead measurements abort the entire run.
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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Path as NavPath
from pinky_soccer_msgs.msg import WorldModel
from pinky_soccer_referee.msg import RefereeState


class DataUnavailable(RuntimeError):
    pass


class Trial(Node):
    def __init__(self, output):
        super().__init__('real_mpc_referee_comparison')
        self.output = output
        self.file = (output/'topics.jsonl').open('w')
        self.latest = {}
        self.seen = {}
        self.references = deque(maxlen=120)
        self.last_visible_reference = None
        self.robot_id = None
        self.problem = None
        self.armed = False
        self.case = None
        self.episode = None
        self.request_id = 41000
        self.launch = None
        self.launch_file = None
        self.request = self.create_publisher(String, '/behavior_test/request', 10)
        self.action_clients = {a: self.create_client(Trigger, '/preparation/'+a)
                         for a in ('prepare', 'start', 'stop')}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = [self.create_subscription(RefereeState, '/referee/state', self.referee, latched),
                     self.create_subscription(WorldModel, '/world_model', self.world, 10),
                     self.create_subscription(NavPath, '/mpc/predicted_path', self.predicted_path, 10)]
        for name in ('preparation/status', 'localization/status', 'mpc/status', 'behavior_test/status'):
            self.subs.append(self.create_subscription(String, '/'+name,
                lambda msg, name=name: self.status(name, msg),
                latched if name in ('preparation/status', 'localization/status') else 10))
        self.create_timer(.15, lambda: self.send({'command': 'ping'}))

    def write(self, topic, data):
        self.file.write(json.dumps(dict(receipt_ros=self.get_clock().now().nanoseconds/1e9,
            receipt_wall=time.monotonic(), case=self.case, episode=self.episode,
            topic=topic, data=data), allow_nan=True)+'\n')
        self.latest[topic] = data
        self.seen[topic] = time.monotonic()

    def status(self, name, msg):
        try:
            self.write(name, json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def predicted_path(self, msg):
        self.write('mpc/predicted_path', dict(
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            frame=msg.header.frame_id,
            xy=[[p.pose.position.x, p.pose.position.y] for p in msg.poses]))

    def world(self, msg):
        s = msg.self
        self.write('world_model', dict(pose=[s.pose.x, s.pose.y, s.pose.theta],
            confidence=s.confidence, state=s.state))
        reference = self.latest.get('referee/state', {})
        if self.armed and s.confidence > 0 and reference.get('valid') and len(reference.get('robots', [])) == 1:
            error = np.linalg.norm(np.array([s.pose.x, s.pose.y])-reference['robots'][0]['p'])
            if error > .15:
                self.problem = 'Onboard/referee positions diverged: %.3f m' % error

    def referee(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        robots = [dict(id=o.id, p=[o.position.x, o.position.y],
                       v=[o.velocity.x, o.velocity.y], reacquired=o.reacquired)
                  for o in msg.robots if o.visible]
        data = dict(stamp=stamp, valid=msg.metric_valid, robots=robots,
                    ball=[msg.ball.position.x, msg.ball.position.y] if msg.ball.visible else None)
        self.write('referee/state', data)
        if msg.metric_valid and len(robots) == 1 and np.isfinite(robots[0]['p']).all():
            self.last_visible_reference = dict(stamp=stamp, robot=robots[0])
        if not self.armed:
            return
        if not msg.metric_valid or len(robots) > 1:
            self.problem = 'Overhead position invalid or robot identity ambiguous'
            return
        if not robots:
            return  # Short gaps are logged, never supplied as MPC measurements.
        o = robots[0]
        if o['id'] != self.robot_id or not np.isfinite(o['p']).all():
            self.problem = 'Overhead robot ID changed or position invalid'
            return
        if self.references:
            previous = self.references[-1]
            dt = stamp-previous['stamp']
            jump = np.linalg.norm(np.asarray(o['p'])-previous['p'])
            if dt <= 0 or jump > .08+.35*dt:
                self.problem = 'Overhead position discontinuity: %.3f m / %.3f s' % (jump, dt)
        self.references.append(dict(stamp=stamp, p=o['p']))

    def send(self, data):
        self.request.publish(String(data=json.dumps(data)))

    def reference(self):
        d = self.latest.get('referee/state', {})
        age = self.get_clock().now().nanoseconds/1e9-d.get('stamp', 0)
        if (not d.get('valid') or len(d.get('robots', [])) > 1 or
                time.monotonic()-self.seen.get('referee/state', -math.inf) > .5 or not 0 <= age <= .5):
            raise DataUnavailable('Fresh unambiguous referee position unavailable')
        if d.get('robots'):
            robot = d['robots'][0]
        else:
            last = self.last_visible_reference
            if not last or self.get_clock().now().nanoseconds/1e9-last['stamp'] > .15:
                raise DataUnavailable('Referee robot missing for over 150ms')
            robot = last['robot']
        p = np.array(robot['p'])
        if not np.isfinite(p).all() or abs(p[0]) > .91 or abs(p[1]) > .61:
            raise DataUnavailable('Referee robot coordinate outside expected field')
        return p

    def spin(self, duration=.02):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.01)
            if self.armed:
                self.reference()
                if self.problem:
                    raise DataUnavailable(self.problem)

    def wait(self, condition, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.spin()
            if condition():
                return True
        return False

    def call(self, action):
        client = self.action_clients[action]
        if not client.wait_for_service(timeout_sec=3):
            raise DataUnavailable('Preparation '+action+' service unavailable')
        future = client.call_async(Trigger.Request())
        if not self.wait(future.done, 4):
            raise DataUnavailable('Preparation '+action+' service response timed out')
        result = future.result()
        if not result.success:
            raise RuntimeError(action+': '+result.message)

    def stop(self):
        self.armed = False
        self.send({'command': 'stop'})
        client = self.action_clients['stop']
        if client.service_is_ready():
            future = client.call_async(Trigger.Request())
            end = time.monotonic()+3
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.02)
        self.file.flush()

    def close_launch(self):
        if self.launch and self.launch.poll() is None:
            os.killpg(self.launch.pid, signal.SIGINT)
            try:
                self.launch.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(self.launch.pid, signal.SIGTERM)
                self.launch.wait(timeout=5)
        if self.launch_file:
            self.launch_file.close()
        self.launch = self.launch_file = None

    def arm(self):
        if not self.wait(lambda: len(self.latest.get('referee/state', {}).get('robots', [])) == 1, .5):
            raise DataUnavailable('Expected exactly one visible referee robot at trial start; got '+
                                  str(self.latest.get('referee/state', {}).get('robots', [])))
        self.reference()
        self.robot_id = self.latest['referee/state']['robots'][0]['id']
        self.references.clear()
        self.problem = None
        self.armed = True

    def pose_pair(self):
        world = self.latest.get('world_model', {})
        if time.monotonic()-self.seen.get('world_model', -math.inf) > .5 or world.get('confidence', 0) <= 0:
            raise DataUnavailable('Onboard localization is stale/invalid')
        return np.array(world['pose']), self.reference()

    def goal(self, name, target):
        self.episode = name
        self.request_id += 1
        world0, ref0 = self.pose_pair()
        start = time.monotonic()
        self.arm()
        self.send(dict(command='run', id=self.request_id, target='GoToPose', timeout_sec=15.,
            ports=dict(x=str(target[0]), y=str(target[1]), theta=str(target[2]),
                avoid_walls='false', avoid_opponents='false', avoid_ball='false',
                completion_mode='STRICT_STOP')))
        terminal = ('success', 'failure', 'timeout', 'error', 'stopped', 'world_model_stale')
        completed = self.wait(lambda: self.latest.get('behavior_test/status', {}).get('request_id') == self.request_id
            and self.latest['behavior_test/status'].get('state') in terminal, 17)
        status = self.latest.get('behavior_test/status', {})
        self.send({'command': 'stop'})
        self.spin(.5)
        world1, ref1 = self.pose_pair()
        row = dict(name=name, target=target, outcome=status.get('state') if completed else 'timeout',
            message=status.get('message'), duration=time.monotonic()-start,
            world_start=world0.tolist(), referee_start=ref0.tolist(),
            world_end=world1.tolist(), referee_end=ref1.tolist(),
            world_error_m=float(np.linalg.norm(world1[:2]-target[:2])),
            referee_error_unaligned_m=float(np.linalg.norm(ref1-target[:2])),
            displacement_disagreement_m=float(np.linalg.norm((ref1-ref0)-(world1[:2]-world0[:2]))))
        print('RESULT', self.case, json.dumps(row), flush=True)
        self.armed = False
        return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--profile', required=True)
    ap.add_argument('--camera', required=True)
    ap.add_argument('--identification', required=True)
    ap.add_argument('--configs', default='2:align,0:align,0:off,2:off,2:align')
    ap.add_argument('--course', choices=('line', 'poses'), default='line',
                    help='poses adds short motion, rotations and a curved GoToPose')
    ap.add_argument('--goals', help='Comma-separated existing course goals to resume, in course order')
    args = ap.parse_args()
    configs = [(float(p.split(':')[0]), p.split(':')[1]) for p in args.configs.split(',')]
    if any(d not in (0., 2.) or o not in ('off', 'align', 'ndob') for d, o in configs):
        ap.error('Use existing damping 0/2 and observer off/align/ndob options')
    selected_goals = set(args.goals.split(',')) if args.goals else None
    allowed = {'setup', 'forward', 'reverse'} if args.course == 'line' else {
        'setup', 'short', 'rotate', 'half_turn', 'curve', 'reverse'}
    if selected_goals is not None and not selected_goals <= allowed:
        ap.error('Unknown course goal: '+str(selected_goals-allowed))
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Trial(args.output)
    results = []
    try:
        if not node.wait(lambda: node.latest.get('referee/state', {}).get('valid'), 8):
            raise DataUnavailable('Referee not ready')
        for index, (damping, observer) in enumerate(configs):
            node.case = '%02d_damping%g_%s' % (index, damping, observer)
            node.episode = 'startup'
            print('CASE', node.case, flush=True)
            node.latest.pop('preparation/status', None)
            node.latest.pop('world_model', None)
            node.latest.pop('mpc/status', None)
            node.latest.pop('behavior_test/status', None)
            launch = ['ros2', 'launch', 'pinky_soccer_viz', 'game.launch.py', 'mode:=real',
                'field:=hardboard', 'team:=Y', 'controller:=mpc', 'mpc_backend:=acados',
                'mpc_damping_scale:='+str(damping), 'mpc_observer_mode:='+observer,
                'max_lin:=0', 'max_ang:=0', 'verify:=true', 'preparation:=true', 'referee:=false',
                'headless:=true', 'localization:=rectangle', 'localization_debug:=false',
                'wheel_feedback:=true', 'calibration_profile:='+args.profile,
                'camera_yaml:='+args.camera, 'ball_offset_source:='+args.profile,
                'identification:='+args.identification]
            node.launch_file = (args.output/(node.case+'.log')).open('w')
            node.launch = subprocess.Popen(launch, stdout=node.launch_file, stderr=subprocess.STDOUT,
                                           start_new_session=True)
            if not node.wait(lambda: node.latest.get('preparation/status', {}).get('state') == 'IDLE', 15):
                raise DataUnavailable('Robot PC stack did not become IDLE')
            # IDLE can precede MPC startup (model loading/code generation). A
            # preparation failure at that point is not a controller comparison.
            if not node.wait(lambda: 'mpc/status' in node.latest and
                             'behavior_test/status' in node.latest and
                             time.monotonic()-node.seen.get('mpc/status', 0) < .3, 20):
                raise DataUnavailable('MPC/BT did not finish startup')
            client = AsyncParameterClient(node, '/mpc_controller')
            if client.wait_for_services(timeout_sec=4):
                f = client.set_parameters([Parameter('timing_trace', value=True)])
                node.wait(f.done, 3)
            node.arm()
            node.episode = 'preparation'
            node.call('prepare')
            if not node.wait(lambda: node.latest.get('preparation/status', {}).get('state') in ('READY', 'ERROR'), 35):
                raise DataUnavailable('Preparation timed out')
            prep = node.latest['preparation/status']
            result = dict(case=node.case, damping=damping, observer=observer, preparation=prep, goals=[])
            results.append(result)
            if prep['state'] == 'READY':
                world, ref = node.pose_pair()
                if np.linalg.norm(world[:2]-ref) > .15:
                    raise DataUnavailable('Onboard/referee positions disagree by more than 15cm')
                node.call('start')
                course = [('setup', [-.50, 0., 0.]), ('forward', [-.25, 0., 0.]),
                          ('reverse', [-.50, 0., 0.])]
                if args.course == 'poses':
                    course = [('setup', [-.45, 0., 0.]), ('short', [-.40, 0., 0.]),
                              ('rotate', [-.40, 0., math.pi/2]),
                              ('half_turn', [-.40, 0., -math.pi/2]),
                              ('curve', [-.20, .15, math.pi/2]),
                              ('reverse', [-.20, -.10, math.pi/2])]
                for name, target in course:
                    if selected_goals is not None and name not in selected_goals:
                        continue
                    ball = node.latest['referee/state'].get('ball')
                    if ball is not None and -.70 < ball[0] < -.05 and abs(ball[1]) < .30:
                        raise DataUnavailable('Ball is in the planned test corridor; ask user to clear it')
                    row = node.goal(name, target)
                    result['goals'].append(row)
                    if row['outcome'] != 'success':
                        break
            node.stop()
            node.close_launch()
            (args.output/'results.json').write_text(json.dumps(results, indent=2))
    except Exception as exc:
        print('HALTED', type(exc).__name__, str(exc), flush=True)
        (args.output/'halt.json').write_text(json.dumps(dict(type=type(exc).__name__, reason=str(exc))))
    finally:
        node.stop()
        node.close_launch()
        (args.output/'results.json').write_text(json.dumps(results, indent=2))
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
