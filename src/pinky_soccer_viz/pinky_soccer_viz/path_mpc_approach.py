"""Experimental scheduled path-MPC approach. AttackController remains cmd_vel owner."""
import math
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from pinky_soccer_mpc.collision_geometry import FieldObstacles, interpolate_poses
from pinky_soccer_mpc.obstacle_snapshot import AvoidanceOptions, AvoidanceConfig, take
from pinky_soccer_mpc.path_time_optimal import PathTimeOptimalMpc, PredictedScene, SEGMENTS, WIDTH, corridor_distance
from pinky_soccer_mpc.scheduled_input import ScheduledInput


class PathMpcApproach:
    def __init__(self,params,field,margin,v_cap,w_cap):
        self.dt=.1
        self.v_cap,self.w_cap=v_cap,w_cap
        self.config=AvoidanceConfig(margin=margin)
        self.field=FieldObstacles(field)
        self.solver=PathTimeOptimalMpc(params,dt=self.dt,max_steps=80,budget=.075)
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.future=None
        self.generation=0
        self.worker_generation=None
        self.schedule=ScheduledInput(self.dt,.025)
        self.reset()

    def reset(self):
        self.generation+=1
        self.schedule.clear()
        self.target=None
        self.active=np.zeros(2)
        self.status='path MPC waiting'

    def close(self):
        self.pool.shutdown(wait=True,cancel_futures=True)

    def scene(self,world,ball=True,walls=True):
        snap=take(AvoidanceOptions(walls,True,ball),self.config,self.field,world)
        tracks=[o for o in world.opponents if o.state!=o.LOST]
        velocity=[]
        for c in snap.circles:
            if c.kind=='opponent' and tracks:
                o=min(tracks,key=lambda o:math.hypot(o.pose.x-c.x,o.pose.y-c.y))
                v=np.array([o.velocity.x,o.velocity.y],float)
                if not np.isfinite(v).all():
                    v=np.zeros(2)
                v*=min(1.,.5/max(np.linalg.norm(v),1e-9))
            else:
                v=np.zeros(2)
            velocity.append(v)
        return PredictedScene(snap,velocity)

    def opponent_safe(self,world,poses,dt):
        clear=self.scene(world,ball=False,walls=False).clearances(
            poses,np.arange(len(poses))*dt)
        return not clear.size or bool(np.all(clear>=np.minimum(self.config.margin,clear[0])[None,:]-1e-6))

    def solve(self,generation,state,goal,route,scene,v):
        if self.worker_generation!=generation:
            self.solver.reset()
            self.worker_generation=generation
        return self.solver.solve_path(state,goal,route,scene,v,self.w_cap,self.dt)

    def command(self,now,wall,pose,wheels,profile,previous,target,route,world,speed_cap):
        route=np.asarray(route,float)
        target=np.asarray(target,float)
        if len(route)>SEGMENTS+1 or len(route)<2:
            self.reset()
            self.status='path MPC unsupported route length'
            return self.active.copy()
        if self.target is None or np.linalg.norm(target[:2]-self.target[:2])>.06 or abs(
            math.atan2(math.sin(target[2]-self.target[2]),math.cos(target[2]-self.target[2])))>.3:
            self.reset()
            self.target=target.copy()
        if np.linalg.norm(np.asarray(previous)-self.active)>1e-4:
            # Safety scaling changed the held input: invalidate the predicted activation.
            self.reset()
            self.target=target.copy()
        scene=self.scene(world)
        if self.future is not None and self.future.done():
            try:
                result=self.future.result()
                if self.future_generation==self.generation:
                    self.schedule.offer(now,result)
            except Exception as exc:
                self.status='path MPC solve error: '+str(exc)
            self.future=None
        due,result,error=self.schedule.take(now)
        if due:
            self.active=np.zeros(2)
            if result is not None and result.get('ok'):
                xs=np.asarray(result.get('xs',[]),float)
                if xs.ndim==2 and len(xs)>1:
                    xy=np.linalg.norm(pose[:2]-xs[0,:2])
                    yaw=abs(math.atan2(math.sin(pose[2]-xs[0,2]),math.cos(pose[2]-xs[0,2])))
                    predicted=interpolate_poses(xs)
                    clear=scene.clearances(predicted,np.arange(len(predicted))*self.dt/5)
                    safe=not clear.size or np.all(clear>=np.minimum(self.config.margin,clear[0])[None,:]-1e-6)
                    if xy<.035 and yaw<.15 and safe and np.max(corridor_distance(predicted,route))<=WIDTH+.001:
                        self.active=np.asarray(result['u'],float)
                        self.status='path MPC optimized approach'
            if not np.any(self.active):
                self.status='path MPC waiting / rejected activation'
        if self.future is None and self.schedule.due is None:
            physical=np.r_[pose,wheels,profile]
            predicted=self.solver.predictor.step(physical,self.active,self.dt)
            goal=target.copy()
            goal[:2]=route[-1]
            route=route.copy()
            route[0]=predicted[:2]
            v=self.v_cap
            if speed_cap>0:
                v=min(v,speed_cap) if v>0 else speed_cap
            self.schedule.begin(now)
            self.future_generation=self.generation
            self.future=self.pool.submit(self.solve,self.generation,np.r_[predicted,self.active],
                goal,route,scene,v)
        return self.active.copy()
