"""Timestamp-aligned discrete disturbance observer for the ramp/lag wheel model.

Inputs are stamped at local publication, not hardware acceptance. Measurements
must share the ROS clock. The observer estimates lumped wheel acceleration,
not transport latency or slip. No ground truth is used.
"""
from collections import deque
import math

import numpy as np

from .ramp_predictor import RampPredictor

FORECAST_TAU = .25
OBSERVER_TAU = .12
DISTURBANCE_LIMIT = 20.  # rad/s^2, initial conservative experimental bound
MAX_AGE = .20
HISTORY = 1.0


class DelayObserver:
    def __init__(self,params,enabled=True):
        self.model=RampPredictor(params,.1)
        self.tau=np.array([self.model.tau_l,self.model.tau_r])
        self.enabled=bool(enabled)
        self.reset()

    def reset(self):
        self.commands=deque()
        self.samples=deque()
        self.innovation=np.zeros(2)
        self.d=np.zeros(2)
        self.accepted=0
        self.rejected=0

    def command(self,t,u):
        t=float(t)
        u=np.asarray(u,float).copy()
        if self.commands and t<self.commands[-1][0]-1e-8:
            self.reset()
        if not self.commands or not np.array_equal(self.commands[-1][1],u):
            self.commands.append((t,u))
        while len(self.commands)>1 and self.commands[1][0]<t-HISTORY:
            self.commands.popleft()
        while len(self.samples)>1 and self.samples[1][0]<t-HISTORY:
            self.samples.popleft()

    def command_at(self,t):
        for stamp,u in reversed(self.commands):
            if stamp<=t+1e-9:
                return u
        return np.zeros(2)

    def propagate(self,x,start,end,d):
        """ZOH command history, constant disturbance over each measured interval."""
        if end<start-1e-8 or end-start>HISTORY:
            raise ValueError('invalid replay interval')
        x=np.asarray(x,float).copy()
        t=start
        u=self.command_at(t)
        for stamp,next_u in self.commands:
            if stamp<=start+1e-9:
                continue
            if stamp>=end-1e-9:
                break
            x=self.model.step(x,u,stamp-t,d)
            t,u=stamp,next_u
        return self.model.step(x,u,max(0.,end-t),d)

    def measure(self,stamp,receipt,wheel):
        stamp,receipt=float(stamp),float(receipt)
        wheel=np.asarray(wheel,float)
        if (wheel.shape!=(2,) or not np.all(np.isfinite(wheel)) or not math.isfinite(stamp)
            or not math.isfinite(receipt)
            or stamp>receipt+.005 or receipt-stamp>MAX_AGE):
            self.rejected+=1
            return False
        stamp=min(stamp,receipt)
        if self.samples and stamp<=self.samples[-1][0]+1e-8:
            self.rejected+=1
            return False
        if not self.samples or stamp-self.samples[-1][0]>MAX_AGE:
            self.d=np.zeros(2)
            self.innovation=np.zeros(2)
            self.samples.clear()
            ref=self.command_at(stamp)
        else:
            old_t,old_w,old_ref,_=self.samples[-1]
            dt=stamp-old_t
            nominal=self.propagate(np.r_[np.zeros(3),old_w,old_ref],old_t,stamp,np.zeros(2))
            ref=nominal[5:7]
            gain=-self.tau*np.expm1(-dt/self.tau)
            # Exact ZOH input-to-wheel sensitivity for additive acceleration.
            # Innovation relative to the previous disturbance estimate.
            raw=(wheel-nominal[3:5])/gain
            self.innovation=wheel-nominal[3:5]-gain*self.d
            alpha=-math.expm1(-dt/OBSERVER_TAU)
            self.d=np.clip((1-alpha)*self.d+alpha*raw,
                           -DISTURBANCE_LIMIT,DISTURBANCE_LIMIT) if self.enabled else np.zeros(2)
        self.samples.append((stamp,wheel.copy(),np.asarray(ref).copy(),self.d.copy()))
        self.accepted+=1
        return True

    def current(self,now):
        """Return wheel/profile and disturbance estimate at the current ROS time."""
        if not self.samples:
            return None
        t,w,ref,d=self.samples[-1]
        if now<t-1e-8 or now-t>MAX_AGE:
            return None
        x=self.propagate(np.r_[np.zeros(3),w,ref],t,now,d)
        return x[3:7],d.copy()

    def align_pose(self,pose,stamp,now):
        """Advance timestamped pose using command history and wheel measurements.

        Wheel updates correct the replayed speeds at their own times. Pose
        integration is differential-drive dead reckoning, not a slip observer.
        """
        if (not np.all(np.isfinite(pose)) or not math.isfinite(stamp)
                or now-stamp>MAX_AGE or stamp>now+1e-8):
            return None
        anchor=None
        for sample in self.samples:
            if sample[0]<=stamp+1e-8:
                anchor=sample
            else:
                break
        if anchor is None or stamp-anchor[0]>MAX_AGE:
            return None
        t,w,ref,d=anchor
        x=self.propagate(np.r_[np.zeros(3),w,ref],t,stamp,d)
        x[:3]=pose
        t=stamp
        for ts,ws,rs,ds in self.samples:
            if ts<=stamp+1e-8 or ts>now+1e-8:
                continue
            x=self.propagate(x,t,ts,d)
            x[3:7]=np.r_[ws,rs]
            t,d=ts,ds
        return self.propagate(x,t,now,d)
