"""ROS/MPC adapter for the reusable PART B policies; a single command authority remains."""
import json
import math
import time
import numpy as np
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from .motion_management import ContinuousState,Completion,CompletionPolicy,TrajectoryBuffer,ConvergenceGuard,pose_error,should_replan
from .collision_monitor import CollisionMonitor
from . import collision_geometry as cg
from .delay_observer import FORECAST_TAU


class MotionRuntime:
    def __init__(self,node):
        self.node=node;p=node.declare_parameter
        self.enabled=bool(p('motion_management',False).value)
        self.default=p('motion_completion','STRICT_STOP').value
        self.default_policy=CompletionPolicy.parse({},self.default)
        self.filter=ContinuousState(float(p('motion_pose_tau',.12).value))
        self.completion=Completion(node.arrival);self.buffer=TrajectoryBuffer(node.dt)
        self.guard=ConvergenceGuard();self.oscillating=False
        self.monitor=CollisionMonitor(node.params,latency=float(p('motion_latency',.15).value),
            deceleration=float(p('motion_deceleration',.3).value),margin=float(p('motion_collision_margin',.01).value))
        self.monitor_enabled=bool(p('motion_collision_monitor',True).value)
        self.policies={};self.invalid_policies={};self.policy=self.default_policy;self.raw=None;self.confidence=0.;self.sigma=0.
        self.gyro=None;self.odom=None;self.scan=None;self.pending_scan=None;self.pending_target=None;self.handoff_at=None
        self.handoff_wall=None;self.handoff_target=None;self.last_fallback=None;self.replan_reason='initial'
        self.error=None;self.last_policy=None
        self.goal_anchor=self.goal_candidate=None;self.goal_candidate_at=None
        self.successor_prepared=False;self.last_eligible=False
        self.owner_goal_id=None
        if not self.enabled:return
        self.tf=Buffer();self.listener=TransformListener(self.tf,node)
        node.create_subscription(String,'motion/policy',self.on_policy,10)
        node.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        node.create_subscription(Odometry,'odom',self.on_odom,qos_profile_sensor_data)
        node.create_subscription(Imu,'imu_raw',self.on_imu,qos_profile_sensor_data)

    @property
    def active(self):
        return self.enabled and self.node.goal is not None and self.node.goal.action=='GoToPose' and self.policy.mode!='LEGACY'

    def on_policy(self,msg):
        data={}
        try:
            if len(msg.data)>4096:raise ValueError('policy too large')
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('policy must be a JSON object')
            gid=int(data['goal_id']);policy=CompletionPolicy.parse(data,self.default)
            if gid<=0:raise ValueError('goal_id required')
            self.policies[gid]=policy
            self.invalid_policies.pop(gid,None)
            if self.owner_goal_id==gid and policy!=self.policy:
                if self.pending_target is not None and (policy.next_target!=self.pending_target or not policy.next_ready):
                    self.node.invalidate();self.pending_target=None;self.successor_prepared=False
                    self.handoff_at=self.handoff_wall=None
                    self.error='successor contract changed while planning or transferring'
                self.policy=policy;self.completion.reset()
            while len(self.policies)>16:del self.policies[next(iter(self.policies))]
        except (ValueError,KeyError,TypeError,json.JSONDecodeError) as e:
            try:
                gid=int(data.get('goal_id',0))
                if gid>0:
                    self.invalid_policies[gid]='invalid completion contract: '+str(e)
                    while len(self.invalid_policies)>16:del self.invalid_policies[next(iter(self.invalid_policies))]
                    if self.owner_goal_id==gid:
                        self.error=self.invalid_policies[gid]
            except (ValueError,TypeError,AttributeError):pass
            self.node.get_logger().warning('Rejected motion policy: '+str(e))

    def on_imu(self,msg):
        now=self.node.get_clock().now().nanoseconds*1e-9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.angular_velocity_covariance[0]<0:return
        # Angular velocity is a vector: honor the actual IMU mounting transform.
        try:
            from scipy.spatial.transform import Rotation
            q=self.tf.lookup_transform('base_footprint',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform.rotation
            r=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
            if 0<=now-stamp<=.2 and np.all(np.isfinite(r)):self.gyro=(float(r[2]),now)
        except Exception:pass  # Wheel yaw prediction remains available.

    def on_odom(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        v,w=msg.twist.twist.linear.x,msg.twist.twist.angular.z
        if math.isfinite(v) and math.isfinite(w):self.odom=(v,w,stamp)

    def on_scan(self,msg):
        self.pending_scan=msg
        self.consume_scan()

    def consume_scan(self):
        msg=self.pending_scan
        if msg is None:return
        now=self.node.get_clock().now().nanoseconds*1e-9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not 0<=now-stamp<=.3:self.pending_scan=None;return
        try:
            from scipy.spatial.transform import Rotation
            t=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            ranges=np.asarray(msg.ranges);angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
            valid=np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=msg.range_max)
            if valid.sum()<8:return  # Empty/partial scan is not proof of free space.
            xyz=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())))
            q=t.rotation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(xyz)
            self.scan=(r[:,:2]+[t.translation.x,t.translation.y],stamp)
            self.pending_scan=None
        except Exception:pass  # No latest-TF fallback; stale safety data causes a stop.

    def on_world(self,msg,now):
        s=msg.self;self.raw=(s.pose.x,s.pose.y,s.pose.theta);self.confidence=float(s.confidence)
        self.sigma=math.sqrt(max(0.,s.covariance[0],s.covariance[4]))
        if self.enabled:
            stamp=s.last_seen.sec+s.last_seen.nanosec*1e-9
            was_valid=self.filter.valid
            self.filter.correct(self.raw,stamp if stamp>0 else now,now)
            if not self.filter.valid or not was_valid:
                self.scan=None;self.pending_scan=None  # Never reuse a pre-recovery map-frame scan.

    def predict(self,now):
        if not self.enabled:return
        self.consume_scan()
        n=self.node;l,r=n.omega
        v=(n.mpc.rl*l+n.mpc.rr*r)/2;w=(n.mpc.rr*r-n.mpc.rl*l)/n.mpc.b
        if self.odom is not None and 0<=now-self.odom[2]<.2:v,w=self.odom[:2]
        if self.gyro is not None and 0<=now-self.gyro[1]<.15:w=self.gyro[0]
        self.filter.predict(now,v,w)
        if self.active:
            if self.filter.pose is not None:n.pose=tuple(self.filter.pose)
            n.pose_valid=n.pose_valid and self.filter.valid

    def select(self,goal,now):
        n=self.node;policy=self.policies.get(int(n.goal.goal_id),self.default_policy)
        fresh_plan=(self.buffer.active is not None and 0<=now-self.buffer.origin<=self.buffer.max_age
                    and 0<=time.monotonic()-self.buffer.wall<=self.buffer.max_age)
        adopted=(self.active and self.handoff_at is not None and 0<=now-self.handoff_at<=.2
                 and 0<=time.monotonic()-self.handoff_wall<=.2 and pose_error(goal,self.handoff_target)[0]<1e-6
                 and pose_error(goal,self.handoff_target)[1]<1e-6 and policy.mode!='LEGACY' and fresh_plan)
        self.policy=policy;self.completion.reset();self.guard.reset();self.pending_target=None;self.handoff_at=None;self.handoff_wall=None
        self.owner_goal_id=int(n.goal.goal_id)
        if not adopted:self.buffer.clear()
        else:self.buffer.version=tuple(goal)
        self.last_fallback=None;self.error=self.invalid_policies.get(int(n.goal.goal_id))
        if policy.mode in ('PASS_THROUGH','HANDOFF') and getattr(n,'time_backend','acados')!='acados':
            self.error='selected backend has no prepared timed trajectory adapter'
        self.successor_prepared=False;self.last_eligible=False
        self.goal_anchor=tuple(goal);self.goal_candidate=None
        return adopted

    def validate(self,us,terminal_target=None):
        n=self.node
        if not n.pose_valid or self.raw is None:return False
        vb,wb=n.caps();vb=min(n.time_mpc.v_limit,vb) if vb>0 else n.time_mpc.v_limit
        wb=min(n.time_mpc.w_limit,wb) if wb>0 else n.time_mpc.w_limit
        terminal=terminal_target is not None
        target=np.array(terminal_target,float) if terminal else np.zeros(3)
        if terminal:
            from .arrival import wrap
            target[2]=n.pose[2]+wrap(target[2]-n.pose[2])
        us=np.asarray(us,float);m=n.time_mpc
        if us.ndim!=2 or us.shape[1]!=2 or not len(us) or not np.all(np.isfinite(us)):return False
        z=us/m.wmax;vw=z@m.B.T
        du=np.diff(np.vstack((np.array(n.last_cmd)/m.wmax,z)),axis=0)
        forward=n.goal.action in n.forward_only
        lower=np.array([0. if forward else -vb,-wb]);upper=np.array([vb,wb])
        violation=max(0.,float(np.max(abs(z)-1)),float(np.max(lower-vw)),
                      float(np.max(vw-upper)),float(np.max(abs(du)-m.slew)))
        xs=self.replay(us)
        if not np.all(np.isfinite(xs)):return False
        if terminal:
            error=xs[-1,:3]-target
            g=np.r_[np.sum((error[:2]/m.pos_tol)**2),error[2]/m.ang_tol,
                    xs[-1,3:7]/m.wheel_tol,us[-1]/m.wheel_tol]
            violation=max(violation,float(np.max(abs(g)-1)))
        if violation>1e-6:return False
        if n.avoid.any:
            _,clear=n.avoid_snapshot(n.last_tick).clearances(cg.interpolate_poses(xs))
            if clear.size and not np.all(clear>=n.avoid_config.margin-1e-6):return False
        return True

    def replay(self,us):
        n=self.node;xs=[np.array([*n.pose,*n.omega,*n.ref],float)]
        # A private predictor and immutable disturbance snapshot, never worker solver state.
        disturbance=np.array(n.observer_disturbance,copy=True)
        for k,u in enumerate(us):
            xs.append(self.monitor.model.step(xs[-1],u,n.dt,disturbance*math.exp(-k*n.dt/FORECAST_TAU)))
        return np.asarray(xs)

    def safety(self,u):
        n=self.node
        if not self.active or not self.monitor_enabled:return tuple(u)
        if n.pose is None or self.raw is None:return (0.,0.)
        if self.sigma>.05:
            return self.protect((0.,0.),dict(state='STOP',reason='localization uncertainty exceeds monitored envelope',replan=True),u)
        now=n.get_clock().now().nanoseconds*1e-9
        if self.scan is None:
            return self.protect((0.,0.),dict(state='STOP',reason='waiting for measurement-time scan TF',replan=True),u)
        objects=[];world=n.avoidance_world()
        if world is not None:
            for o in world.opponents:
                if o.state!=o.LOST:
                    inflation=min(.1,2*math.sqrt(max(0.,o.covariance[0],o.covariance[4])))
                    objects.append((o.pose.x,o.pose.y,n.avoid_config.opponent_radius+inflation,
                                    o.velocity.x,o.velocity.y))
        # Collision monitor is in the raw map frame, never the smoothed control pose.
        state=[*(self.raw or n.pose),*n.omega,*n.ref]
        command,diag=self.monitor.check(state,n.last_cmd,u,self.scan[0],now-self.scan[1],
                                       2*self.sigma,objects)
        return self.protect(command,diag,u)

    def protect(self,command,diag,requested):
        n=self.node;n.diag['collision_monitor']=diag
        if diag['replan']:self.replan_reason='collision monitor'
        elif self.replan_reason=='collision monitor':self.replan_reason='retain safe trajectory'
        if command!=tuple(requested):
            self.buffer.clear();self.successor_prepared=False
            n.last_solution=None;n.active_solve_id=None
            n.generation+=1;n.schedule.clear()
        if diag['state']=='STOP' and self.handoff_at is not None:
            self.handoff_at=self.handoff_wall=None
            self.error='prepared handoff invalidated by collision monitor'
        return command

    def arrival(self,now,goal,v,w,fresh):
        if self.policy.mode=='PASS_THROUGH':
            n=self.node;signed=(n.mpc.rl*n.omega[0]+n.mpc.rr*n.omega[1])/2
            v=signed
        can_brake=True
        if self.policy.mode in ('STRICT_STOP','PRACTICAL_STOP'):
            n=self.node
            position=n.arrival.pos_tol if self.policy.mode=='STRICT_STOP' else self.policy.position
            heading=n.arrival.ang_tol if self.policy.mode=='STRICT_STOP' else self.policy.heading
            d,a=pose_error(n.pose,goal)
            if d<=position and a<=heading:
                xs,_=self.monitor.rollout([*(self.raw or n.pose),*n.omega,*n.ref],n.last_cmd,(0.,0.),0.,0.)
                sd,sa=pose_error(xs[-1,:3],goal)
                can_brake=sd<=position and sa<=heading
        out=self.completion.update(now,self.node.pose,self.raw or self.node.pose,goal,v,w,self.node.omega,fresh,self.policy,can_brake)
        if self.policy.mode in ('PASS_THROUGH','HANDOFF') and not (self.policy.next_ready and self.policy.next_target):
            self.error='continuous completion requires a ready successor contract'
        cv,cw=self.node.adapter.to_twist(*self.node.last_cmd)
        self.oscillating=self.guard.update(now,out['distance'],out['angle'],cv,cw)
        if self.oscillating and not out['eligible'] and self.pending_target is None:
            self.error='near-goal command reversals without convergence or ready successor'
        self.last_eligible=out['eligible']
        if out['prepare'] and self.pending_target is None:
            self.pending_target=self.policy.next_target
            # Invalidate only the obsolete pending calculation, not the verified active plan.
            self.node.generation+=1;self.node.schedule.clear()
        fresh_plan=(self.buffer.active is not None and self.buffer.version==self.pending_target
                    and 0<=now-self.buffer.origin<=self.buffer.max_age-.2
                    and 0<=time.monotonic()-self.buffer.wall<=self.buffer.max_age-.2)
        if out['eligible'] and self.successor_prepared and fresh_plan and self.handoff_at is None:
            self.activated(now,self.pending_target)
        if self.handoff_at is not None and (now-self.handoff_at>.2 or time.monotonic()-self.handoff_wall>.2):
            self.error='successor did not acquire control before handoff expiry'
        return out

    def target(self,goal):
        if self.pending_target is not None:return self.pending_target
        now=self.node.last_tick
        if self.goal_anchor is None:self.goal_anchor=tuple(goal)
        d,a=pose_error(goal,self.goal_anchor)
        change,_=should_replan(True,d,0.,0.,0.)
        if d<1e-6 and a<1e-6:self.goal_candidate=None;return self.goal_anchor
        if self.goal_candidate is None or max(pose_error(goal,self.goal_candidate))>.002:
            self.goal_candidate=tuple(goal);self.goal_candidate_at=now
        if change or a>.05 or now-self.goal_candidate_at>.3:
            self.goal_anchor=tuple(goal);self.goal_candidate=None
            self.node.generation+=1;self.node.schedule.clear()
            self.replan_reason='goal changed'
        return self.goal_anchor

    def retain_direction(self,result,now,wall,goal):
        b=self.buffer
        if b.active is None or b.version!=tuple(goal) or self.pending_target is not None:return result
        old=np.asarray(b.result.get('xs',[]));new=np.asarray(result.get('xs',[]))
        if len(old)<3 or len(new)<3:return result
        direction=np.array(goal[:2])-np.asarray(self.node.pose[:2])
        if np.linalg.norm(direction)<.05:return result
        def side(xs):
            return float(np.cross(direction,xs[len(xs)//2,:2]-np.asarray(self.node.pose[:2])))
        k=int((now-b.origin+1e-8)/b.dt)
        count=min(len(old)-k,len(new))
        if count<2:return result
        path_delta=float(np.max(np.linalg.norm(old[k:k+count,:2]-new[:count,:2],axis=1)))
        tracking_error=pose_error(old[min(k,len(old)-1),:3],self.node.pose)[0]
        different_side=side(old)*side(new)<-1e-5
        if not different_side and path_delta>.02:return result
        old_time=max(0.,len(b.active)*self.node.dt-(now-b.origin))
        gain=old_time-len(result['us'])*self.node.dt
        safe=self.buffer.continuation(now,wall,b.version,self.validate)
        switch,reason=should_replan(safe is not None,0.,0.,tracking_error,gain)
        if safe is None:reason='active trajectory expired or invalid'
        if self.replan_reason=='collision monitor':switch=True;reason='collision monitor'
        self.replan_reason=reason
        if switch:return result
        us=b.active[k:]
        xs=self.replay(us)
        return dict(result,u=safe,us=us,xs=xs,_retain_active=True,source_solve_id=b.result.get('source_solve_id'))

    def activated(self,now,goal):
        if self.pending_target is not None and tuple(goal)==tuple(self.pending_target) and self.handoff_at is None:
            self.successor_prepared=True
            if self.last_eligible:
                self.handoff_at=now;self.handoff_wall=time.monotonic();self.handoff_target=tuple(goal)

    def state(self,reason,dist,ang):
        n=self.node
        if not self.enabled or n.goal is None:return None
        data=dict(goal_id=str(n.goal.goal_id),stamp=n.get_clock().now().nanoseconds*1e-9,
            pose=list(n.pose) if n.pose is not None else None,raw_pose=self.raw,
            v=(n.mpc.rl*n.omega[0]+n.mpc.rr*n.omega[1])/2,
            omega=(n.mpc.rr*n.omega[1]-n.mpc.rl*n.omega[0])/n.mpc.b,
            confidence=self.confidence,position_error=dist,heading_error=ang,reason=reason,
            mode=self.policy.mode,handoff_ready=self.handoff_at is not None,next_target=self.handoff_target,
            oscillating=self.oscillating,
            replan_reason=self.replan_reason)
        return data
