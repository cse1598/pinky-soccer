#!/usr/bin/env python3
"""Local MuJoCo acceptance harness. Never use against a real ROS domain.

Ground-truth subscriptions are recorded for evaluation only. Teleport commands
are limited to ball-placement rule tests, never robot localization or control.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_srvs.srv import Trigger
from pinky_soccer_msgs.msg import WorldModel, BehaviorGoal, MotionResult
from pinky_soccer_referee.msg import RefereeState, RefereeEvent


class Trial(Node):
    def __init__(self, output, namespace="", opponent=""):
        super().__init__('attack_sim_trial', namespace=namespace, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.prepare_seen = False
        self.latest = {}; self.goals = Counter(); self.moves = []; self.events = []
        self.file = (output/'topics.jsonl').open('w')
        self.output = output
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for t in ('preparation/status', 'motion/status', 'localization/status'):
            self.create_subscription(String, t, lambda m,t=t:self.record(t,json.loads(m.data)),
                                     q if t!='motion/status' else 10)
        self.create_subscription(RefereeState, '/referee/state', self.referee, q)
        self.create_subscription(RefereeEvent, '/referee/events', self.event, q)
        self.create_subscription(WorldModel, 'world_model', self.world, 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_subscription(PoseStamped, 'ground_truth', lambda m:self.record('truth',
            [m.pose.position.x,m.pose.position.y,m.pose.orientation.z,m.pose.orientation.w]), 10)
        if opponent:
            self.create_subscription(PoseStamped, '/'+opponent+'/ground_truth',
                lambda m:self.record('opponent_truth',[m.pose.position.x,m.pose.position.y]),10)
        self.motion_result = None
        self.manual_goal = None
        self.strategy = self.create_publisher(BehaviorGoal,'preparation/strategy_goal',10)
        self.create_subscription(MotionResult,'motion/result',lambda m:setattr(self,'motion_result',m),10)
        self.create_timer(.05,self.publish_manual_goal)
        self.ball_pub = self.create_publisher(Point, '/sim/set_ball', 10)
        self.clients_by_action = {a:self.create_client(Trigger,'preparation/'+a) for a in ('prepare','start','stop')}

    def publish_manual_goal(self):
        if self.manual_goal is not None:
            self.manual_goal.header.stamp=self.get_clock().now().to_msg()
            self.strategy.publish(self.manual_goal)

    def record(self, topic, data):
        if topic=='preparation/status' and data.get('state') in ('CALIBRATING','MOVING_TO_START'):
            self.prepare_seen=True
        self.latest[topic] = data
        self.file.write(json.dumps(dict(t=self.get_clock().now().nanoseconds/1e9,
                                       wall=time.monotonic(),topic=topic,data=data))+'\n')

    def referee(self,m):
        self.record('referee',dict(valid=m.metric_valid,calibrated=m.calibrated,armed=m.armed,
            state=m.status,ball=[m.ball.position.x,m.ball.position.y] if m.ball.visible else None,
            robots=[[r.id,r.position.x,r.position.y] for r in m.robots if r.visible]))

    def event(self,m):
        d=dict(sequence=m.sequence,event=m.event,color=m.goal_color)
        self.events.append(d);self.record('event',d);print('EVENT',d,flush=True)

    def world(self,m):
        self.record('world',dict(pose=[m.self.pose.x,m.self.pose.y,m.self.pose.theta],
            ball=[m.ball.pose.x,m.ball.pose.y],ball_state=m.ball.state,playing=m.playing))

    def command(self,m):
        self.moves.append((m.linear.x,m.angular.z))
        self.record('cmd',self.moves[-1])

    def wait(self, condition, timeout, label):
        until=time.monotonic()+timeout;last=0
        while time.monotonic()<until:
            rclpy.spin_once(self,timeout_sec=.02)
            if condition():return True
            if time.monotonic()-last>5:
                print(label,self.latest.get('preparation/status',{}).get('state'),
                    self.latest.get('motion/status',{}),flush=True);last=time.monotonic()
        return False

    def call(self, action):
        c=self.clients_by_action[action]
        if not c.wait_for_service(timeout_sec=4):raise RuntimeError(action+' unavailable')
        f=c.call_async(Trigger.Request())
        if not self.wait(f.done,4,action):raise RuntimeError(action+' timed out')
        r=f.result();print('SERVICE',action,r.success,r.message,flush=True)
        if not r.success:raise RuntimeError(r.message)

    def place(self,x,y):
        self.ball_pub.publish(Point(x=float(x),y=float(y)))

    def move_ball(self, points):
        for x,y in points:
            self.place(x,y)
            if not self.wait(lambda:self.latest.get('referee',{}).get('ball') is not None and
                abs(self.latest['referee']['ball'][0]-x)<.035 and
                abs(self.latest['referee']['ball'][1]-y)<.035,8,'ball crossing'):
                raise RuntimeError('Referee lost moving ball')
            stamp=self.get_clock().now().nanoseconds/1e9
            self.wait(lambda:self.get_clock().now().nanoseconds/1e9-stamp>.25,4,'observe crossing')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--scenario',choices=('observe','rules','prepare','moving'),default='observe')
    ap.add_argument('--robot-name',default='')
    ap.add_argument('--opponent-name',default='')
    args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='193':ap.error('Isolated simulation domain 193 required')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=Trial(args.output,args.robot_name,args.opponent_name);report={'scenario':args.scenario}
    try:
        if not n.wait(lambda:n.latest.get('referee',{}).get('valid') and
                      n.latest.get('preparation/status',{}).get('state') in ('IDLE','STOPPED','READY'),20,'startup'):
            raise RuntimeError('Simulated image referee not armed or stack not idle')
        assert not any(abs(v)+abs(w)>1e-8 for v,w in n.moves),'Moved before prepare'
        report['idle_zero']=True
        n.prepare_seen=False
        n.call('prepare')
        if not n.wait(lambda:n.prepare_seen and n.latest.get('preparation/status',{}).get('state') in ('READY','ERROR','STOPPED'),70,'prepare'):
            raise RuntimeError('Preparation timed out')
        prep=n.latest['preparation/status'];report['preparation']=prep
        if prep['state']!='READY':raise RuntimeError('Preparation: '+prep.get('reason',''))
        if args.scenario=='prepare':
            report['peak_cmd_v']=max((abs(v) for v,w in n.moves),default=0)
            report['prepared_only']=True
            return
        if args.scenario=='moving':
            # Test-owned BT is halted only for this independent GoToPose trial.
            # Never override a running user match or publish velocity commands.
            import psutil,signal
            stopped=0
            for process in psutil.process_iter(['pid','cmdline']):
                command=process.info['cmdline'] or []
                if (any('/.worktrees/attack/install/pinky_soccer_bt/' in c for c in command)
                        and process.environ().get('ROS_DOMAIN_ID')=='193'):
                    process.send_signal(signal.SIGINT);stopped+=1
            if stopped!=1:raise RuntimeError('Expected one test-owned BT for moving trial')
        n.call('start')
        if not n.wait(lambda:n.latest.get('preparation/status',{}).get('state')=='PLAYING',10,'start'):
            raise RuntimeError('Start failed: '+str(n.latest.get('preparation/status')))
        if args.scenario=='moving':
            g=BehaviorGoal();g.header.frame_id='map';g.mode=g.NAVIGATE
            g.branch='TEST';g.action='GoToPose';g.goal_id=time.time_ns()
            g.target.x,g.target.y,g.target.theta=.4,-.1,0.
            n.manual_goal=g
            if not n.wait(lambda:n.motion_result is not None and n.motion_result.goal_id==g.goal_id
                and n.motion_result.state in (MotionResult.SUCCESS,MotionResult.FAILURE),65,'moving obstacle'):
                raise RuntimeError('Moving obstacle GoToPose timed out')
            report['motion_result']=dict(state=n.motion_result.state,reason=n.motion_result.reason,
                position_error=n.motion_result.position_error,heading_error=n.motion_result.heading_error)
            if n.motion_result.state!=MotionResult.SUCCESS:raise RuntimeError('Moving obstacle GoToPose failed')
            report['moving_trial']=True
        elif args.scenario=='observe':
            start=n.get_clock().now().nanoseconds/1e9
            n.wait(lambda:bool(n.events) or n.get_clock().now().nanoseconds/1e9-start>25 or
                   n.latest.get('preparation/status',{}).get('state') in ('ERROR','STOPPED'),90,'attack')
            report['autonomous_goal']=any(e['event']=='GOAL' for e in n.events)
            if report['autonomous_goal']:
                if not n.wait(lambda:n.latest.get('preparation/status',{}).get('state') in ('READY','ERROR'),70,'goal return'):
                    raise RuntimeError('Autonomous goal return timed out')
                report['goal_return']=n.latest['preparation/status']
                if report['goal_return']['state']!='READY':raise RuntimeError('Autonomous goal return failed')
        elif args.scenario=='rules':
            n.move_ball([(0,.25),(0,.45),(0,.52),(0,.575)])
            if not n.wait(lambda:n.latest.get('preparation/status',{}).get('state')=='STOPPED',15,'out'):
                raise RuntimeError('OUT did not stop')
            report['out_stop']=n.latest.get('preparation/status')
            n.move_ball([(0,.3),(0,0)])
            if not n.wait(lambda:n.latest.get('referee',{}).get('ball') is not None and
                max(abs(v) for v in n.latest['referee']['ball'])<.04,10,'replace'):
                raise RuntimeError('Centre replacement not observed')
            n.call('start')
            if not n.wait(lambda:n.latest.get('preparation/status',{}).get('state')=='PLAYING',10,'restart'):
                raise RuntimeError('Restart after OUT failed')
            n.move_ball([(.25,0),(.5,0),(.72,0),(.80,0),(.86,0)])
            if not n.wait(lambda:any(e['event']=='GOAL' for e in n.events),15,'goal'):
                raise RuntimeError('Image referee did not detect goal')
            n.place(0,0)
            if not n.wait(lambda:n.latest.get('preparation/status',{}).get('state') in ('READY','ERROR'),70,'return'):
                raise RuntimeError('Return did not finish')
            report['goal_return']=n.latest.get('preparation/status')
            if report['goal_return']['state']!='READY':raise RuntimeError('Goal return failed')
        report['final']=n.latest
        report['events']=n.events
        report['peak_cmd_v']=max((abs(v) for v,w in n.moves),default=0)
    except Exception as e:
        report['error']=str(e);print('FAIL',str(e),flush=True)
    finally:
        n.manual_goal=None
        try:n.call('stop')
        except Exception:pass
        n.file.close()
        (args.output/'result.json').write_text(json.dumps(report,indent=2))
        n.destroy_node();rclpy.shutdown()
    print(json.dumps({k:v for k,v in report.items() if k not in ('preparation','final','out_stop','goal_return')},indent=2),flush=True)


if __name__=='__main__':main()
