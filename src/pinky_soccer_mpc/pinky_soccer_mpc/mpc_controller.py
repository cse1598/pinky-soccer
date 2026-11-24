"""One GoToPose authority: NMPC tracking, zero-command settling, typed result.

The solver worker owns its warm start. The ROS thread owns input callbacks,
request identity, the 50 Hz command heartbeat and all safety/result decisions.
"""
import json
import math
import time
import numpy as np
from concurrent.futures import ThreadPoolExecutor

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from pinky_soccer_dynamics import params as dyn_params
from pinky_soccer_dynamics.command_adapter import CommandAdapter, wheels_from_joint_state
from pinky_soccer_dynamics.profile_model import motor_step
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel, MotionResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .arrival import Arrival, wrap
from .pose_mpc import PoseMpc
from .time_optimal import TimeOptimalMpc
from .acados_time_optimal import AcadosTimeOptimalMpc
from .scheduled_input import ScheduledInput


class MpcController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        p=self.declare_parameter
        target=p('target','real').value
        profile=p('calibration_profile','').value
        identification=p('identification','').value
        if identification:
            raise ValueError('Legacy identification not validated for phase-1 model')
        self.params=dyn_params.load(target,profile=profile or None)
        if self.params.m('delay') > 0.:
            raise ValueError('Phase-1 predictor does not compensate transport delay')
        self.max_lin=float(p('max_lin',0.).value)
        self.max_ang=float(p('max_ang',0.).value)
        self.goal_timeout=float(p('goal_timeout',.5).value)
        self.pose_timeout=float(p('pose_timeout',.5).value)
        self.rate=float(p('rate_hz',10.).value)
        self.command_rate=float(p('command_rate_hz',50.).value)
        self.dt=1/self.rate
        self.mpc=PoseMpc(self.params,p('horizon',2.).value,int(p('steps',20).value))
        if abs(self.dt-self.mpc.dt)>1e-6:
            raise ValueError('horizon/steps must match optimization period')
        self.time_backend=p('mpc_backend','acados').value
        if self.time_backend not in ('acados','slsqp'):
            raise ValueError('mpc_backend must be acados or slsqp')
        solver_cls=AcadosTimeOptimalMpc if self.time_backend=='acados' else TimeOptimalMpc
        self.time_mpc=solver_cls(self.params, dt=self.dt,
             max_steps=int(p('time_optimal_max_steps',80).value),
             budget=float(p('time_optimal_budget_sec',.075).value))
        self.schedule=ScheduledInput(self.dt, min(.025,.45*self.dt))
        self.future_scheduled=False
        self.future_predicted=None
        self.active_predicted=None
        self.active_solve_id=None
        self.pose_receipt_sim=None
        self.adapter=CommandAdapter(self.params)
        self.arrival=Arrival(float(p('pos_tol',.02).value),
                             float(p('ang_tol',math.radians(3)).value),
                             float(p('settle_sec',.3).value))
        self.wheel_feedback=bool(p('wheel_feedback',False).value)
        self.forward_only=set(p('forward_only_actions',[
            'GoBehindBall','PushBall','GoToBlockPoint','FaceBall','TurnToLastSeen']).value)
        self.goal=self.pose=self.measured=None
        self.goal_time=self.pose_time=self.measured_time=None
        self.pose_valid=False
        self.world_v=0.
        self.omega=self.ref=self.last_cmd=(0.,0.)
        self.last_tick=None
        self.key=None
        self.generation=0
        self.worker_generation=None
        self.future=None
        self.future_generation=None
        self.future_wall=None
        self.future_sim=None
        self.solve_id=0
        self.last_solution_input_sim=None
        self.last_solution_input_wall=None
        self.next_solve=0.
        self.last_solution=None
        self.last_solution_sim=None
        self.last_solution_wall=None
        self.failures=0
        self.failed_reason=None
        self.feedback_age=None
        self.diag={}
        self.progress_time=None
        self.progress_best=None
        self.executor_pool=ThreadPoolExecutor(max_workers=1)
        self.pub=self.create_publisher(Twist,'cmd_vel',10)
        self.path_pub=self.create_publisher(Path,'mpc/predicted_path',10)
        self.status_pub=self.create_publisher(String,'mpc/status',10)
        self.result_pub=self.create_publisher(MotionResult,'motion/result',10)
        self.create_subscription(BehaviorGoal,'behavior_goal',self.on_goal,10)
        self.create_subscription(WorldModel,'world_model',self.on_world,10)
        if self.wheel_feedback:
            self.create_subscription(JointState,'joint_states',self.on_joints,qos_profile_sensor_data)
        self.create_timer(1/self.command_rate,self.tick)
        self.get_logger().info(f'Fixed-step time-optimal NMPC {target}/{self.time_backend}: {self.rate:g} Hz solve, '
                               f'{self.command_rate:g} Hz command; up to {self.time_mpc.max_steps} stages; '
                               f'{self.mpc.wmax*30/math.pi:.1f} RPM')

    def on_goal(self,msg):
        self.goal,self.goal_time=msg,time.monotonic()

    def on_world(self,msg):
        s=msg.self
        self.pose=(s.pose.x,s.pose.y,s.pose.theta)
        self.pose_time=time.monotonic()
        self.pose_receipt_sim=self.get_clock().now().nanoseconds*1e-9
        self.world_v=math.hypot(s.velocity.x,s.velocity.y)
        self.pose_valid=(s.state!=s.LOST and s.confidence>0 and
                         all(math.isfinite(v) for v in (*self.pose,self.world_v)))

    def on_joints(self,msg):
        try:
            wheel=wheels_from_joint_state(self.params,msg.name,msg.velocity)
            if all(math.isfinite(v) for v in wheel):
                self.measured,self.measured_time=wheel,time.monotonic()
        except (KeyError,ValueError,IndexError):
            pass

    def caps(self):
        v,w=self.max_lin,self.max_ang
        if self.goal.max_speed>0:
            if self.goal.action=='SpinInPlace':
                w=min(w,self.goal.max_speed) if w else self.goal.max_speed
            else:
                v=min(v,self.goal.max_speed) if v else self.goal.max_speed
        return v,w

    def send(self,u):
        self.last_cmd=tuple(float(q) for q in u)
        v,w=self.adapter.to_twist(*self.last_cmd)
        msg=Twist()
        msg.linear.x,msg.angular.z=float(v),float(w)
        self.pub.publish(msg)
        return v,w

    def invalidate(self):
        self.generation+=1
        self.last_solution=self.last_solution_sim=self.last_solution_wall=None
        self.last_solution_input_sim=self.last_solution_input_wall=None
        self.next_solve=0.
        self.schedule.clear()
        self.active_predicted=None
        self.active_solve_id=None

    def job(self,generation,x,goal,v,w,forward,is_pose):
        if self.worker_generation!=generation:
            self.mpc.reset()
            self.time_mpc.reset()
            self.worker_generation=generation
        solver=self.time_mpc if is_pose else self.mpc
        return solver.solve(x,goal,v,w,forward_only=forward)

    def report(self,state,reason='',result=MotionResult.RUNNING,dist=0.,ang=0.,**extra):
        gid=int(self.goal.goal_id) if self.goal is not None else 0
        data=dict(self.diag)
        data.update(extra)
        data.update(state=state,reason=reason,goal_id=str(gid),
                    wheel_l=float(self.omega[0]),wheel_r=float(self.omega[1]),
                    profile_l=float(self.ref[0]),profile_r=float(self.ref[1]),
                    wheel_cmd_l=self.last_cmd[0],wheel_cmd_r=self.last_cmd[1],
                    feedback_age=self.feedback_age,feedback_enabled=self.wheel_feedback,
                    position_error=dist,heading_error=ang,
                    backend=data.get('backend') or (f'discrete_time_optimal_{self.time_backend}' if
                        self.goal is not None and self.goal.action=='GoToPose' else 'slsqp'))
        data['command_state_age_sim_s'] = (self.last_tick-self.last_solution_input_sim
            if state=='track' and self.last_solution_input_sim is not None else None)
        data['command_state_age_wall_s'] = (time.monotonic()-self.last_solution_input_wall
            if state=='track' and self.last_solution_input_wall is not None else None)
        data['active_solve_id']=self.active_solve_id if state=='track' else None
        data['compensated_state_age_sim_s']=(self.last_tick-self.last_solution_input_sim-self.dt
            if state=='track' and self.active_solve_id is not None else None)
        data['pose_receipt_age_sim_s'] = (max(0.,self.last_tick-self.pose_receipt_sim)
            if self.last_tick is not None and self.pose_receipt_sim is not None else None)
        self.status_pub.publish(String(data=json.dumps(data)))
        if gid and self.goal.action=='GoToPose':
            msg=MotionResult()
            msg.header.stamp=self.get_clock().now().to_msg()
            msg.header.frame_id='map'
            msg.goal_id,msg.state,msg.reason=gid,result,reason
            msg.position_error,msg.heading_error=float(dist),float(ang)
            self.result_pub.publish(msg)

    def stop(self,reason,result=MotionResult.RUNNING,dist=0.,ang=0.):
        self.send((0.,0.))
        self.report('failed' if result==MotionResult.FAILURE else 'stop',
                    reason,result,dist,ang)

    def tick(self):
        wall=time.monotonic()
        now=self.get_clock().now().nanoseconds*1e-9
        elapsed=1/self.command_rate if self.last_tick is None else now-self.last_tick
        self.last_tick=now
        if elapsed<0 or elapsed>.5:
            self.invalidate()
            self.arrival.reset()
            self.omega=self.ref=self.last_cmd=(0.,0.)
            return self.stop('clock discontinuity')
        self.omega,self.ref=motor_step(self.params,self.omega,self.ref,self.last_cmd,elapsed)
        self.feedback_age=None if self.measured_time is None else wall-self.measured_time
        feedback=self.feedback_age is not None and self.feedback_age<.2
        if feedback:
            self.omega=self.measured
        # Drain a finished job even when cancelled or stopped. Never use an old generation.
        if self.future is not None and self.future.done():
            future=self.future
            generation=self.future_generation
            submitted=self.future_wall
            submitted_sim=self.future_sim
            completed_id=self.solve_id
            scheduled=self.future_scheduled
            predicted=self.future_predicted
            self.future=None
            try:
                res=future.result()
            except Exception as exc:
                res={'ok':False,'status':-1,'ms':(wall-submitted)*1000,
                     'message':str(exc),'residuals':[]}
            if generation==self.generation:
                activation_diag={k:v for k,v in self.diag.items() if k.startswith('activation_') or k=='prediction_lead_sim_s'}
                self.diag={k:res.get(k) for k in ('status','ms','direction','residuals',
                                                'iterations','v_bound','w_bound','message','backend',
                                                'horizon_steps','planned_time','terminal_feasible',
                                                'attempts','lower_steps','attempt_statuses','replay_violation')}
                self.diag.update(activation_diag)
                self.diag.update(solve_id=completed_id,
                                 solve_input_age_sim_s=now-submitted_sim,
                                 solve_input_age_wall_s=wall-submitted)
                if scheduled:
                    # Hold the existing stage until its scheduled boundary.
                    # The new solution is only eligible for that boundary.
                    if res['ok'] and wall-submitted<=.3:
                        self.schedule.offer(now,(res,predicted,submitted_sim,submitted,completed_id))
                elif res['ok'] and wall-submitted<=.3:
                    self.last_solution=tuple(res['u'])
                    self.last_solution_sim=now
                    self.last_solution_wall=wall
                    self.last_solution_input_sim=submitted_sim
                    self.last_solution_input_wall=submitted
                    self.failures=0
                    self.publish_path(res['xs'])
                else:
                    self.last_solution=None
                    self.failures+=1
        fresh=(self.goal is not None and wall-self.goal_time<=self.goal_timeout
               and self.pose is not None and wall-self.pose_time<=self.pose_timeout)
        if not fresh or self.goal.mode==BehaviorGoal.STOP:
            if self.key is not None:
                self.invalidate()
                self.key=None
            self.arrival.reset()
            self.diag={}
            return self.stop('STOP goal' if fresh else 'stale input')
        t=self.goal.target
        goal=(t.x,t.y,t.theta)
        is_pose=self.goal.action=='GoToPose'
        key=(self.goal.action,int(self.goal.goal_id))
        if key!=self.key:
            self.key=key
            self.invalidate()
            self.arrival.reset()
            self.diag={}
            self.failures=0
            self.failed_reason=None
            self.progress_time=now
            self.progress_best=None
        if not self.pose_valid or not all(math.isfinite(v) for v in goal):
            self.arrival.reset()
            self.invalidate()
            return self.stop('invalid localization or target')
        if is_pose and not self.wheel_feedback:
            return self.stop('GoToPose requires verified wheel_feedback:=true',MotionResult.FAILURE)
        if self.wheel_feedback and not feedback:
            self.arrival.reset()
            self.invalidate()
            return self.stop('stale wheel feedback')
        if self.failed_reason is not None:
            return self.stop(self.failed_reason,MotionResult.FAILURE)
        l,r=self.omega
        v_actual=(self.mpc.rl*l+self.mpc.rr*r)/2
        w_actual=(self.mpc.rr*r-self.mpc.rl*l)/self.mpc.b
        inside,done,dist,ang=self.arrival.update(now,self.pose,goal,
                 max(abs(v_actual),self.world_v),w_actual,self.omega,feedback)
        if is_pose and inside and self.arrival.since is not None:
            # Braking to zero is not a P alignment controller; if we drift out,
            # tracking resumes immediately with the same NMPC.
            if (self.last_solution is not None or self.schedule.due is not None or
                    (self.future is not None and self.future_generation==self.generation)):
                self.invalidate()
            self.send((0.,0.))
            return self.report('arrived' if done else 'settle','settled' if done else 'waiting for rest',
                               MotionResult.SUCCESS if done else MotionResult.RUNNING,dist,ang)
        if is_pose:
            merit=dist+.05*ang
            if self.progress_best is None or merit<self.progress_best-.005:
                self.progress_best,self.progress_time=merit,now
            if self.failures>=3:
                self.failed_reason='three consecutive optimizer failures'
            elif now-self.progress_time>4.:
                self.failed_reason='no pose progress for 4 s'
            if self.failed_reason:
                self.invalidate()
                return self.stop(self.failed_reason,MotionResult.FAILURE,dist,ang)
        if self.future is not None and wall-self.future_wall>.3:
            if self.future_generation==self.generation:
                self.invalidate()
                self.failures+=1
            return self.stop('solver deadline')
        if is_pose and self.time_backend=='acados':
            return self.scheduled_tick(now,wall,goal,dist,ang,elapsed)
        if self.future is None and now+1e-8>=self.next_solve:
            v,w=self.caps()
            x=(*self.pose,*self.omega,*self.last_cmd)
            self.future_generation=self.generation
            self.future_wall=wall
            self.future_sim=now
            self.future_scheduled=False
            self.future_predicted=None
            self.solve_id+=1
            self.future=self.executor_pool.submit(self.job,self.generation,x,goal,v,w,
                                                   self.goal.action in self.forward_only,is_pose)
            self.next_solve=now+self.dt
        valid=(self.last_solution is not None and now-self.last_solution_sim<=2*self.dt
               and wall-self.last_solution_wall<=.3)
        v,w=self.send(self.last_solution if valid else (0.,0.))
        self.report('track' if valid else 'waiting', 'optimized' if valid else 'no fresh optimized command',
                    dist=dist,ang=ang,v=v,w=w,period_s=elapsed)

    def scheduled_tick(self,now,wall,goal,dist,ang,elapsed):
        """Pipeline exactly one 100 ms stage; optimize the next activation state."""
        due,result,jitter=self.schedule.take(now)
        if due:
            self.diag['activation_jitter_sim_s']=float(jitter)
            if result is not None:
                res,predicted,submitted_sim,submitted,solve_id=result
                self.active_solve_id=solve_id
                self.last_solution=tuple(res['u'])
                self.last_solution_sim=now
                self.last_solution_wall=wall
                self.last_solution_input_sim=submitted_sim
                self.last_solution_input_wall=submitted
                self.active_predicted=predicted
                self.failures=0
                self.diag.update(activation_position_error_m=math.hypot(
                    self.pose[0]-predicted[0],self.pose[1]-predicted[1]),
                    activation_heading_error_rad=abs(wrap(self.pose[2]-predicted[2])),
                    activation_wheel_error_rad_s=float(np.max(np.abs(
                        np.asarray(self.omega)-predicted[3:5]))))
                self.publish_path(res['xs'])
            else:
                # No valid solution for this interval: brake, not an old first input.
                self.last_solution=None
                self.active_predicted=None
                self.active_solve_id=None
                self.failures+=1
                self.diag.update(activation_position_error_m=None,
                    activation_heading_error_rad=None,activation_wheel_error_rad_s=None,
                    activation_missed=True)
                # Invalidate a late worker so it cannot fill a subsequent slot.
                self.generation+=1
            self.diag['activation_missed']=result is None
        if self.failures>=3:
            self.failed_reason='three consecutive missed/failed activation slots'
            self.invalidate()
            return self.stop(self.failed_reason,MotionResult.FAILURE,dist,ang)
        command=self.last_solution if self.last_solution is not None else (0.,0.)
        # Send before snapshotting prediction: this is the known held command.
        v,w=self.send(command)
        if self.schedule.due is None and self.future is None:
            vb,wb=self.caps()
            physical=np.asarray((*self.pose,*self.omega,*self.ref),float)
            predicted=self.time_mpc.predictor.step(physical,command,self.dt)
            state=np.r_[predicted,command]
            self.schedule.begin(now)
            self.future_generation=self.generation
            self.future_wall=wall
            self.future_sim=now
            self.future_scheduled=True
            self.future_predicted=predicted
            self.solve_id+=1
            self.future=self.executor_pool.submit(self.job,self.generation,state,
                 goal,vb,wb,self.goal.action in self.forward_only,True)
            self.diag['prediction_lead_sim_s']=self.dt
        valid=self.last_solution is not None
        self.report('track' if valid else 'waiting',
                    'scheduled optimized input' if valid else 'waiting for scheduled solution',
                    dist=dist,ang=ang,v=v,w=w,period_s=elapsed)

    def publish_path(self,xs):
        path=Path()
        path.header.stamp=self.get_clock().now().to_msg()
        path.header.frame_id='map'
        for x in xs:
            ps=PoseStamped()
            ps.header=path.header
            ps.pose.position.x,ps.pose.position.y=float(x[0]),float(x[1])
            ps.pose.orientation.z,ps.pose.orientation.w=math.sin(x[2]/2),math.cos(x[2]/2)
            path.poses.append(ps)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node=MpcController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.send((0.,0.))
        node.executor_pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()
        rclpy.try_shutdown()
