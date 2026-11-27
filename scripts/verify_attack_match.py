#!/usr/bin/env python3
"""Two sensor-driven robots, first to three; only the test driver presses START."""
import argparse,json,os,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from pinky_soccer_referee.msg import RefereeState,RefereeEvent
from pinky_soccer_msgs.msg import WorldModel
from geometry_msgs.msg import PoseStamped

class Match(Node):
 def __init__(self,out):
  super().__init__('attack_match_trial',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.file=out.open('w');self.states={};self.motion={};self.ref=None;self.events=[];self.last_score=0.;self.count={};self.starts=0;self.play_started=None
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.service_clients={}
  self.reward={name:dict(total=0.,stationary_s=0.,far_ball_s=0.,distance_m=0.,recoveries=0,goals=0,conceded=0,outs=0,playing_s=0.,stagnant_ball_s=0.) for name in ('yellow','blue')}
  self.ball_anchor={};self.last_world={};self.last_moved={};self.poses={};self.seen_recoveries={};self.last_reward_log=0.
  for name in ('yellow','blue'):
   for suffix in ('preparation/status','motion/status','localization/status'):
    t=f'/{name}/{suffix}';self.create_subscription(String,t,lambda m,t=t:self.status(t,json.loads(m.data)),10)
   for action in ('prepare','start','stop'):self.service_clients[name,action]=self.create_client(Trigger,f'/{name}/preparation/{action}')
   self.create_subscription(PoseStamped,f'/{name}/ground_truth',lambda m,n=name:self.log(n+'/truth',[m.pose.position.x,m.pose.position.y]),10)
   self.create_subscription(WorldModel,f'/{name}/world_model',lambda m,n=name:self.world(n,m),10)
  self.create_subscription(RefereeState,'/referee/state',self.referee,q)
  self.create_subscription(RefereeEvent,'/referee/events',self.event,q)
 def world(self,name,m):
  now=self.now();pose=(m.self.pose.x,m.self.pose.y,m.self.pose.theta)
  previous=self.last_world.get(name);self.last_world[name]=(now,pose)
  self.poses[name]=pose
  if not previous or self.states.get(name,{}).get('state')!='PLAYING':
   self.last_moved[name]=(now,pose);self.ball_anchor.pop(name,None);return
  dt=max(0.,min(.2,now-previous[0]));distance=math.hypot(pose[0]-previous[1][0],pose[1]-previous[1][1])
  score=self.reward[name];score['playing_s']+=dt;score['total']-=.2*dt;score['distance_m']+=distance;score['total']-=.5*distance
  moved_at,anchor=self.last_moved.get(name,(now,pose))
  if math.hypot(pose[0]-anchor[0],pose[1]-anchor[1])>.008 or abs(math.atan2(math.sin(pose[2]-anchor[2]),math.cos(pose[2]-anchor[2])))>.06:self.last_moved[name]=(now,pose)
  elif now-moved_at>=1.:score['stationary_s']+=dt;score['total']-=5*dt
  if self.ref and self.ref.ball.visible:
   goal=.83 if name=='yellow' else -.83
   distance_goal=math.hypot(goal-self.ref.ball.position.x,self.ref.ball.position.y)
   stamp,anchor=self.ball_anchor.get(name,(now,distance_goal))
   if anchor-distance_goal>.05:stamp,anchor=now,distance_goal
   self.ball_anchor[name]=(stamp,anchor)
   if now-stamp>5.:score['stagnant_ball_s']+=dt;score['total']-=dt
   distance_ball=math.hypot(pose[0]-self.ref.ball.position.x,pose[1]-self.ref.ball.position.y)
   if distance_ball>.45:score['far_ball_s']+=dt;score['total']-=.2*dt
   elif distance_ball<.20:score['total']+=.1*dt
  self.log(name+'/world',dict(pose=pose,ball=[m.ball.pose.x,m.ball.pose.y],ball_state=m.ball.state))
  if now-self.last_reward_log>1.:
   self.log('reward',self.reward);self.last_reward_log=now
 def now(self):return self.get_clock().now().nanoseconds*1e-9
 def log(self,t,d):self.file.write(json.dumps(dict(t=self.now(),topic=t,data=d))+'\n')
 def status(self,t,d):
  self.log(t,d)
  if t.endswith('preparation/status'):self.states[t.split('/')[1]]=d
  if t.endswith('motion/status'):
   name=t.split('/')[1];self.motion[name]=d
   attempts=d.get('recovery_attempts',0);old=self.seen_recoveries.get(name,0)
   self.reward[name]['recoveries']+=max(0,attempts-old);self.reward[name]['total']-=2*max(0,attempts-old);self.seen_recoveries[name]=attempts
 def referee(self,m):
  self.ref=m;self.log('referee',dict(status=m.status,armed=m.armed,ball_visible=m.ball.visible,ball=[m.ball.position.x,m.ball.position.y],yellow=m.yellow_goal_entries,blue=m.blue_goal_entries))
 def event(self,m):
  if m.sequence in self.count:return
  if m.event in ('GOAL','OUT','UNVERIFIED_EXIT'):self.play_started=None
  if m.event=='GOAL':
   scorer='blue' if m.goal_color=='yellow' else 'yellow';other='yellow' if scorer=='blue' else 'blue'
   self.reward[scorer]['total']+=100;self.reward[scorer]['goals']+=1
   self.reward[other]['total']-=100;self.reward[other]['conceded']+=1
  elif m.event in ('OUT','UNVERIFIED_EXIT'):
   for score in self.reward.values():score['total']-=10;score['outs']+=1
  self.count[m.sequence]=True;self.events.append((self.now(),m.event,m.goal_color));self.log('event',dict(sequence=m.sequence,event=m.event,color=m.goal_color));print('EVENT',self.events[-1],flush=True)
 def spin(self,s=.1):rclpy.spin_once(self,timeout_sec=s)
 def call(self,action):
  futures=[]
  for name in ('yellow','blue'):
   c=self.service_clients[name,action]
   if c.service_is_ready():futures.append((name,c.call_async(Trigger.Request())))
  end=time.monotonic()+5
  while any(not f.done() for _,f in futures) and time.monotonic()<end:self.spin()
  result=[(n,f.result().success,f.result().message) for n,f in futures if f.done() and f.result()]
  self.log('service/'+action,result);print(action,result,flush=True);return len(result)==2 and all(r[1] for r in result)
 def play(self,limit):
  end=time.monotonic()+45
  while (len(self.states)<2 or not self.ref or not self.ref.calibrated) and time.monotonic()<end:self.spin()
  if not self.call('prepare'):return 'prepare rejected'
  started=time.monotonic();center_since=None;lastprint=0.;self.play_started=None
  while time.monotonic()-started<limit:
   self.spin(.03);now=self.now()
   if any(s['state']=='ERROR' for s in self.states.values()):return 'robot ERROR: '+json.dumps(self.states)
   if self.ref and max(self.ref.yellow_goal_entries,self.ref.blue_goal_entries)>=3:return 'SUCCESS'
   waiting=len(self.states)==2 and all(s['state'] in ('READY','STOPPED') and s.get('referee_start_allowed') for s in self.states.values())
   if waiting:
    center_since=now if center_since is None else center_since
    if now-center_since>=1.:
     if self.call('start'):self.starts+=1;self.play_started=now
     center_since=None
   else:center_since=None
   if now-lastprint>5:
    print('STATE',round(now,2),{n:s['state'] for n,s in self.states.items()},{n:(s.get('state'),s.get('reason')) for n,s in self.motion.items()},flush=True);lastprint=now
   # An unproductive rally is a failure, not a manufactured score.
   if self.play_started is not None and now-self.play_started>75:return 'FAIL: no completed rally within 75 simulation seconds'
  return 'FAIL: wall time limit'

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--limit',type=float,default=240);a=p.parse_args()
 if os.environ.get('ROS_DOMAIN_ID')!='193':raise SystemExit('test domain 193 only')
 out=Path(a.output);out.mkdir(parents=True,exist_ok=True);rclpy.init();n=Match(out/'topics.jsonl')
 try:
  result=n.play(a.limit);print('RESULT',result,flush=True)
  (out/'result.json').write_text(json.dumps(dict(result=result,score_version=2,events=n.events,starts=n.starts,scores=[n.ref.yellow_goal_entries,n.ref.blue_goal_entries] if n.ref else None,reward=n.reward),ensure_ascii=False,indent=2))
 finally:n.call('stop');n.file.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
