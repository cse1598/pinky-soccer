"""Referee-first, instantaneous sensor selection and continuous Kalman state.

No dwell timer or consecutive-frame requirement. Existing observation freshness
limits reject stale packets only; they never delay a valid source transition.
"""
from dataclasses import dataclass
import math

import numpy as np

from .ball_filter import BallFilter, LOST


@dataclass(frozen=True)
class HybridParams:
    agreement_m: float = .06
    confidence_min: float = .65
    referee_sigma: float = .03
    local_sigma: float = .012
    referee_velocity_sigma: float = .12
    max_position_correction: float = .025
    max_velocity_correction: float = .18

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('invalid hybrid parameter: '+name)
        if self.confidence_min > 1:
            raise ValueError('confidence_min must be <= 1')


@dataclass
class Observation:
    stamp: float
    xy: object
    velocity: object = None
    confidence: float = 0.
    distance: float = 0.


def vector(value):
    if value is None:
        return None
    a = np.asarray(value, float)
    return a if a.shape == (2,) and np.all(np.isfinite(a)) else None


class HybridBallFilter(BallFilter):
    def __init__(self, params=None, bounds=None, ball_radius=.02135, hybrid=None):
        super().__init__(params, bounds, ball_radius)
        self.h = hybrid or HybridParams()
        self.referee = self.local = None
        self.source = 'none'
        self.last_used = {'referee_camera': -math.inf, 'robot_camera': -math.inf}
        self.last_velocity = -math.inf
        self.difference = None
        self.reason = 'no observations'
        self.updated = False

    def receive_referee(self, stamp, xy, velocity=None):
        if not math.isfinite(stamp) or (self.referee is not None and stamp <= self.referee.stamp):
            return
        vel = vector(velocity)
        if vel is not None and np.linalg.norm(vel) > self.p.max_speed:
            vel = None
        self.referee = Observation(stamp, vector(xy), vel)

    def receive_local(self, stamp, xy, confidence=0., distance=0.):
        if not math.isfinite(stamp) or (self.local is not None and stamp <= self.local.stamp):
            return
        self.local = Observation(stamp, vector(xy), confidence=confidence, distance=distance)

    def invalidate_local(self):
        self.local = None

    def fresh(self, obs, now, timeout):
        return obs is not None and obs.xy is not None and -.05 <= now-obs.stamp <= timeout

    def choose(self, now):
        # Same existing validity windows as referee and BallFilter OBSERVED;
        # not a wait-before-switching policy.
        ref = self.referee if self.fresh(self.referee, now, .4) else None
        local = self.local if self.fresh(self.local, now, self.p.observed_timeout) else None
        self.difference = None
        if ref is None:
            self.reason = 'no valid referee reference'
            return 'none', None, None
        if local is not None and math.isfinite(local.confidence) and (
                local.confidence >= self.h.confidence_min and
                math.isfinite(local.distance) and local.distance >= 0.):
            # Compare at local capture time rather than the two arrival times.
            ref_xy = ref.xy if ref.velocity is None else ref.xy+ref.velocity*(local.stamp-ref.stamp)
            self.difference = float(np.linalg.norm(local.xy-ref_xy))
            if self.difference <= self.h.agreement_m:
                self.reason = 'local agrees with referee'
                return 'robot_camera', local, ref
            self.reason = 'position disagreement'
        else:
            self.reason = 'local absent or low confidence'
        return 'referee_camera', ref, ref

    def correct(self, z, H, sigma):
        # Bound the innovation correction, not the physical velocity prediction.
        # Joseph form with the actual (possibly reduced) gain retains PSD covariance.
        R = np.eye(2)*sigma**2
        S = H@self.P@H.T+R
        K = np.linalg.solve(S, H@self.P).T
        delta = K@(z-H@self.x)
        scale = min(1., self.h.max_position_correction/max(1e-12,np.linalg.norm(delta[:2])),
                    self.h.max_velocity_correction/max(1e-12,np.linalg.norm(delta[2:])))
        K *= scale
        self.x += scale*delta
        A = np.eye(4)-K@H
        self.P = A@self.P@A.T+K@R@K.T
        self.P = .5*(self.P+self.P.T)

    def step(self, now):
        self.updated = False
        source, obs, ref = self.choose(now)
        self.source = source
        self.predict(now)
        if obs is None:
            return False
        new_position = obs.stamp > self.last_used[source]
        new_velocity = ref.velocity is not None and ref.stamp > self.last_velocity
        if not (new_position or new_velocity):
            return False
        # Compensate camera transport age with referee velocity (or prior velocity).
        vel = ref.velocity if ref.velocity is not None else (
            self.x[2:].copy() if self.initialized else np.zeros(2))
        z = obs.xy+vel*max(0.,now-obs.stamp)
        sigma = (self.h.local_sigma+self.p.meas_sigma_per_m2*obs.distance**2
                 if source == 'robot_camera' else self.h.referee_sigma)
        # Extra uncertainty for a transported observation. No hard reset on source change.
        sigma = math.hypot(sigma, self.p.accel_noise*max(0.,now-obs.stamp)**2/2)
        reacquire = not self.initialized or self.state(now) == LOST
        if reacquire:
            if not new_position:
                return False
            self.reset(z,now,sigma)
            if ref.velocity is not None:
                self.x[2:] = ref.velocity
                self.P[2:,2:] = np.eye(2)*self.h.referee_velocity_sigma**2
                self.last_velocity = ref.stamp
        else:
            if new_position:
                self.correct(z,np.eye(4)[:2],sigma)
            if new_velocity:
                self.correct(ref.velocity,np.eye(4)[2:],self.h.referee_velocity_sigma)
                self.last_velocity = ref.stamp
        if new_position:
            self.last_used[source] = obs.stamp
            # Freshness follows capture time, not repeated prediction/publishing.
            self.last_seen = obs.stamp if reacquire else max(self.last_seen,obs.stamp)
            self.updated = True
        self._constrain()
        return self.updated
