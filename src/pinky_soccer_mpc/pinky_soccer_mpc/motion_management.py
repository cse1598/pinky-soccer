"""Controller-independent state continuity, completion contracts and timed plan buffers.

All poses and goals remain in map; only measured motion predicts between absolute fixes.
No ground truth, controller gains, solver, ROS or strategy is used here.
"""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np
from .arrival import wrap


def pose_error(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1]),abs(wrap(a[2]-b[2]))


class ContinuousState:
    def __init__(self,tau=.12,max_correction=.20,max_yaw_correction=.8):
        if not all(math.isfinite(v) and v>0 for v in (tau,max_correction,max_yaw_correction)):
            raise ValueError('invalid state continuity parameters')
        self.tau,self.max_correction,self.max_yaw_correction=tau,max_correction,max_yaw_correction
        self.pose=None;self.time=None;self.source=None;self.last_fix=None
        self.v=self.w=0.;self.valid=False;self.recovery=[];self.correction=np.zeros(3)

    def predict(self,now,v,w):
        if self.time is not None and (now<self.time or now-self.time>.5):
            self.valid=False;self.pose=None;self.source=None
        if self.pose is not None:
            dt=max(0.,now-self.time);mid=self.pose[2]+w*dt/2
            self.pose+=np.array([v*dt*math.cos(mid),v*dt*math.sin(mid),w*dt])
            self.pose[2]=wrap(self.pose[2])
        self.time=now;self.v,self.w=v,w
        return None if self.pose is None else tuple(self.pose)

    def correct(self,pose,stamp,now):
        p=np.asarray(pose,float)
        if not np.all(np.isfinite(p)) or stamp>now+.05 or now-stamp>.25:
            self.valid=False;return False
        if self.source is not None and stamp<=self.source:return self.valid
        self.source=stamp
        # Align the absolute fix to the current prediction time, without changing its frame.
        age=max(0.,now-stamp);mid=p[2]+self.w*age/2
        p=p+np.array([self.v*age*math.cos(mid),self.v*age*math.sin(mid),self.w*age])
        if self.pose is None:
            self.pose=p;self.time=now;self.valid=True;self.last_fix=now;return True
        dp,da=pose_error(p,self.pose)
        if dp>.25 or da>.7 or not self.valid:
            if self.recovery and pose_error(p,self.recovery[-1])[0]>.05:self.recovery=[]
            if self.recovery and pose_error(p,self.recovery[-1])[1]>.15:self.recovery=[]
            self.recovery.append(p)
            self.valid=False
            if len(self.recovery)>=3:
                self.pose=p;self.time=now;self.valid=True;self.recovery=[];self.last_fix=now
            return self.valid
        dt=max(.001,min(.25,now-(self.last_fix if self.last_fix is not None else now-.03)))
        self.last_fix=now
        e=p-self.pose;e[2]=wrap(e[2]);e*=1-math.exp(-dt/self.tau)
        norm=np.linalg.norm(e[:2]);e[:2]*=min(1.,self.max_correction*dt/max(norm,1e-12))
        e[2]=float(np.clip(e[2],-self.max_yaw_correction*dt,self.max_yaw_correction*dt))
        self.pose+=e;self.pose[2]=wrap(self.pose[2]);self.correction=e
        return True


@dataclass(frozen=True)
class CompletionPolicy:
    mode: str='STRICT_STOP'
    position: float=.02
    heading: float=math.pi/60
    dwell: float=.1
    next_target: tuple=()
    next_ready: bool=False

    @classmethod
    def parse(cls,data,default='STRICT_STOP'):
        mode=data.get('mode',default)
        if mode=='DEFAULT':mode=default
        if mode not in ('LEGACY','STRICT_STOP','PRACTICAL_STOP','PASS_THROUGH','HANDOFF'):
            raise ValueError('unknown completion mode')
        pos=float(data.get('position',.02));ang=float(data.get('heading',math.pi/60))
        dwell=float(data.get('dwell',.1));nxt=tuple(data.get('next_target',()))
        if not all(math.isfinite(x) and x>0 for x in (pos,ang,dwell)) or pos>.25 or ang>math.pi:
            raise ValueError('invalid task tolerances')
        if nxt and (len(nxt)!=3 or not all(math.isfinite(float(x)) for x in nxt)):
            raise ValueError('invalid successor pose')
        if not isinstance(data.get('next_ready',False),bool):raise ValueError('next_ready must be boolean')
        return cls(mode,pos,ang,dwell,tuple(map(float,nxt)),data.get('next_ready',False))


class Completion:
    def __init__(self,legacy):
        self.legacy=legacy;self.reset()

    def reset(self):
        self.braking=False;self.since=None;self.done=False;self.previous_plane=None;self.passed=False
        self.legacy.reset()

    def update(self,now,pose,raw,goal,v,w,wheels,fresh,policy,can_brake=True):
        if policy.mode=='LEGACY':
            inside,done,d,a=self.legacy.update(now,raw,goal,v,w,wheels,fresh)
            return dict(brake=inside and self.legacy.since is not None,done=done,eligible=False,prepare=False,distance=d,angle=a)
        d,a=pose_error(pose,goal);rd,ra=pose_error(raw,goal)
        position=self.legacy.pos_tol if policy.mode=='STRICT_STOP' else policy.position
        heading=self.legacy.ang_tol if policy.mode=='STRICT_STOP' else policy.heading
        inside=d<=position and a<=heading and rd<=position and ra<=heading
        eligible=prepare=False
        if policy.mode in ('PASS_THROUGH','HANDOFF'):
            if policy.next_target and policy.next_ready and fresh:
                if policy.mode=='HANDOFF':
                    return dict(brake=False,done=False,eligible=inside,prepare=inside,distance=rd,angle=ra)
                direction=np.array(policy.next_target[:2],float)-np.array(goal[:2]);length=np.linalg.norm(direction)
                if length>1e-6:
                    direction/=length;offset=np.array(raw[:2])-np.array(goal[:2])
                    plane=float(offset@direction);lateral=abs(float(np.cross(direction,offset)))
                    movement=raw[2]+(math.pi if v<0 else 0.)
                    aligned=(ra<=heading and abs(wrap(movement-math.atan2(direction[1],direction[0])))<=heading)
                    crossed=self.previous_plane is not None and self.previous_plane<=0<=plane
                    self.passed=self.passed or (crossed and lateral<=position and aligned)
                    eligible=(self.passed and lateral<=position and aligned and abs(v)>.01 and 0<=plane<=max(position,.05))
                    prepare=(eligible if policy.mode=='HANDOFF' else -.12<=plane<=0 and lateral<=2*position and aligned)
                    self.previous_plane=plane
            return dict(brake=False,done=False,eligible=eligible,prepare=prepare,distance=rd,angle=ra)
        # Brake on entering the pose region, before requiring measured standstill.
        if inside and can_brake:self.braking=True
        if max(d,rd)>position+.005 or max(a,ra)>heading+math.radians(1):
            self.braking=False;self.since=None;self.done=False
        stopped=abs(v)<=.01 and abs(w)<=.05 and max(abs(q) for q in wheels)<=.2
        if self.braking and stopped and fresh:
            if self.since is None:self.since=now
            dwell=self.legacy.dwell if policy.mode=='STRICT_STOP' else policy.dwell
            self.done=inside and now-self.since>=dwell
        else:self.since=None;self.done=False
        return dict(brake=self.braking,done=self.done,eligible=False,prepare=False,distance=rd,angle=ra)


class TrajectoryBuffer:
    """Timed wheel controls; switching is permitted only after independent actual-state replay.

    Replaying the motor map splices physical pose/velocity continuously and respects ramp
    limits. It does not claim arbitrary curvature/C2 continuity at zero linear velocity.
    """
    def __init__(self,dt,max_age=.3):
        self.dt,self.max_age=dt,max_age;self.clear()

    def clear(self):
        self.active=self.pending=self.result=None;self.origin=self.wall=None;self.version=None

    def offer(self,result,version):
        self.pending=(result,version)

    def activate(self,now,wall,version,validate):
        pending=self.pending;self.pending=None
        if pending is None or pending[1]!=version:return None
        result=pending[0]
        us=np.asarray(result.get('us',[]),float)
        if not len(us) or not validate(us):return None
        self.active=us.copy();self.result=result;self.origin=now;self.wall=wall;self.version=version
        return tuple(us[0])

    def continuation(self,now,wall,version,validate):
        if (self.active is None or self.version!=version or now<self.origin or wall<self.wall
                or now-self.origin>self.max_age or wall-self.wall>self.max_age):return None
        k=int(math.floor((now-self.origin+1e-8)/self.dt))
        if k>=len(self.active) or not validate(self.active[k:]):return None
        return tuple(self.active[k])


def should_replan(active_safe,goal_delta,pose_jump,tracking_error,remaining_gain,
                  goal_threshold=.02,pose_threshold=.05,error_threshold=.05,time_hysteresis=.15):
    if not active_safe:return True,'collision'
    if pose_jump>pose_threshold:return True,'pose correction'
    if tracking_error>error_threshold:return True,'tracking error'
    if goal_delta>goal_threshold:return True,'goal changed'
    if remaining_gain>time_hysteresis:return True,'arrival improvement'
    return False,'retain safe trajectory'


class ConvergenceGuard:
    """Detect repeated near-goal command reversals; elapsed time never implies success."""
    def __init__(self):self.reset()

    def reset(self):
        self.best=math.inf;self.improved=None;self.signs=[0,0];self.reversals=deque()

    def update(self,now,distance,angle,v,w):
        merit=distance+.05*angle
        if merit<self.best-.003:self.best=merit;self.improved=now
        for i,(value,deadband) in enumerate(((v,.01),(w,.05))):
            sign=1 if value>deadband else -1 if value<-deadband else 0
            if sign and self.signs[i] and sign!=self.signs[i]:self.reversals.append(now)
            if sign:self.signs[i]=sign
        while self.reversals and now-self.reversals[0]>2.:self.reversals.popleft()
        return (distance<.05 and angle<.2 and len(self.reversals)>=4 and
                self.improved is not None and now-self.improved>.8)
