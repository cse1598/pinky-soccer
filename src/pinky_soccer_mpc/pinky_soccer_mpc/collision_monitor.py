"""Independent model-based braking monitor. It never waits for the path planner."""
import math
import numpy as np
from scipy.signal import lfilter
from .ramp_predictor import RampPredictor
from .collision_geometry import FOOTPRINT_RECTS


def clearance(poses,points,radii=0.):
    poses=np.asarray(poses,float);points=np.asarray(points,float)
    if not len(points):return np.full(len(poses),np.inf)
    # points may be static Mx2 or predicted NxMx2.
    if points.ndim==2:points=points[None,:,:]
    dx=points[...,0]-poses[:,None,0];dy=points[...,1]-poses[:,None,1]
    c=np.cos(poses[:,None,2]);s=np.sin(poses[:,None,2]);x=c*dx+s*dy;y=-s*dx+c*dy
    best=np.full(x.shape,np.inf)
    for x0,y0,x1,y1 in FOOTPRINT_RECTS:
        ox=np.maximum(np.maximum(x0-x,x-x1),0.);oy=np.maximum(np.maximum(y0-y,y-y1),0.)
        inside=-np.minimum(np.minimum(x-x0,x1-x),np.minimum(y-y0,y1-y))
        d=np.where((ox>0)|(oy>0),np.hypot(ox,oy),inside)
        best=np.minimum(best,d)
    return np.min(best-np.asarray(radii),axis=1)


class CollisionMonitor:
    def __init__(self,params,latency=.15,deceleration=.3,margin=.01):
        if not all(math.isfinite(v) and v>0 for v in (latency,deceleration,margin)):
            raise ValueError('invalid braking monitor parameters')
        self.model=RampPredictor(params,.02)
        self.latency=max(latency,3*max(self.model.tau_l,self.model.tau_r))
        self.deceleration=deceleration;self.margin=margin
        self.fast_enabled=True
        self.footprint_radius=max(math.hypot(x,y) for x0,y0,x1,y1 in FOOTPRINT_RECTS
                                  for x,y in ((x0,y0),(x0,y1),(x1,y0),(x1,y1)))

    def rollout(self,state,previous,command,hold,age):
        x=np.asarray(state,float);h=.005
        v=abs((self.model.rl*x[3]+self.model.rr*x[4])/2)
        delay=math.ceil((self.latency+age)/.02-1e-9)*.02
        hold=math.ceil(hold/.02-1e-9)*.02
        braking=min(3.-delay-hold,v/self.deceleration+.5)
        refs=[];ref=x[5:7].copy()
        # Exact bounded ramp at the shared 5 ms motor-model resolution.
        for u,duration in ((previous,delay),(command,hold),((0.,0.),braking)):
            count=max(0,math.ceil(duration/h-1e-9))
            t=np.arange(1,count+1)[:,None]*h
            segment=ref+np.clip(np.asarray(u)-ref,-self.model.accel*t,self.model.accel*t)
            if count:refs.append(segment);ref=segment[-1]
        reference=np.vstack(refs);prior=np.vstack((x[5:7],reference[:-1]))
        average=(prior+reference)/2
        wheels=np.empty_like(reference)
        for j,tau in enumerate((self.model.tau_l,self.model.tau_r)):
            decay=math.exp(-h/tau)
            wheels[:,j]=lfilter([1-decay],[1,-decay],average[:,j],zi=[decay*x[3+j]])[0]
        mean=(np.vstack((x[3:5],wheels[:-1]))+wheels)/2
        left,right=self.model.rl*mean[:,0],self.model.rr*mean[:,1]
        speed=(left+right)/2;rotation=h*(right-left)/self.model.b
        yaw=x[2]+np.cumsum(rotation);mid=yaw-rotation/2
        positions=x[:2]+np.cumsum(h*speed[:,None]*np.column_stack((np.cos(mid),np.sin(mid))),axis=0)
        states=np.column_stack((positions,yaw,wheels,reference))
        indices=np.unique(np.r_[np.arange(3,len(states),4),len(states)-1])
        return np.vstack((x,states[indices])),np.r_[0.,(indices+1)*h]

    def check(self,state,previous,command,points,age,uncertainty=0.,obstacles=()):
        if age<0 or age>.3 or not math.isfinite(age):
            return (0.,0.),dict(state='STOP',reason='raw scan stale',replan=True)
        points=np.asarray(points,float).reshape(-1,2)
        if not len(points) or not all(np.all(np.isfinite(v)) for v in (state,previous,command,points)):
            return (0.,0.),dict(state='STOP',reason='invalid safety observation',replan=True)
        uncertainty=max(0.,min(.1,uncertainty));margin=self.margin+uncertainty
        v=abs((self.model.rl*state[3]+self.model.rr*state[4])/2)
        stopping=v*(self.latency+age)+v*v/(2*self.deceleration)+margin
        if self.fast_enabled:
            # A circumscribed reachable disc proves clearance without choosing a path.
            # Every wheel/profile state remains bounded by these extrema under the ramp map.
            maxima=np.max(abs(np.array([state[3:5],state[5:7],previous,command])),axis=0)
            bound=(self.model.rl*maxima[0]+self.model.rr*maxima[1])/2
            motor_stop=(self.model.rl*(maxima[0]*self.model.tau_l+maxima[0]**2/(2*self.model.accel))+
                        self.model.rr*(maxima[1]*self.model.tau_r+maxima[1]**2/(2*self.model.accel)))/2
            stop_length=max(motor_stop,bound**2/(2*self.deceleration))
            delay=math.ceil((self.latency+age)/.02-1e-9)*.02
            nearest=float(np.min(np.linalg.norm(points-np.asarray(state[:2]),axis=1)))
            def outside(hold):
                radius=self.footprint_radius+bound*(delay+hold)+stop_length+margin
                if nearest<=radius:return False
                horizon=min(3.,delay+hold+max(bound/self.deceleration,
                    float(max(maxima))/self.model.accel+3*max(self.model.tau_l,self.model.tau_r))+.5)
                return all(math.hypot(x-state[0],y-state[1])-r-math.hypot(vx,vy)*horizon>radius
                           for x,y,r,vx,vy in obstacles)
            if outside(.10):
                future=outside(.5)
                return tuple(command),dict(state='CLEAR' if future else 'REPLAN',
                    reason='conservative reachable disc clear',d_stop=stopping,
                    replan=not future,fast_bound=True)
        def safe(u,hold):
            xs,times=self.rollout(state,previous,u,hold,age)
            d=clearance(xs[:,:3],points)
            for x,y,r,vx,vy in obstacles:
                moving=np.stack((x+vx*times,y+vy*times),axis=1)[:,None,:]
                d=np.minimum(d,clearance(xs[:,:3],moving,r))
            return bool(np.min(d)>=margin),float(np.min(d))
        braking,dist=safe((0.,0.),0.)
        if not braking:
            return (0.,0.),dict(state='STOP',reason='braking sweep collision',clearance=dist,d_stop=stopping,replan=True)
        safe_command,dist=safe(command,.10)
        if not safe_command:
            scaled=tuple(.5*np.asarray(command));ok,dist=safe(scaled,.10)
            if ok:return scaled,dict(state='SLOW',reason='limit stopping sweep',clearance=dist,d_stop=stopping,replan=True)
            return (0.,0.),dict(state='STOP',reason='command stopping sweep collision',clearance=dist,d_stop=stopping,replan=True)
        future,_=safe(command,.5)
        return tuple(command),dict(state='CLEAR' if future else 'REPLAN',reason='safe braking envelope',
                                   clearance=dist,d_stop=stopping,replan=not future)
