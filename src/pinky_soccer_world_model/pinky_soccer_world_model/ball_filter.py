"""2D constant-velocity Kalman filter for the ball with rolling-friction decay.

State [x, y, vx, vy] in the map frame. Between observations the velocity decays as
exp(-dt / friction_tau) and the position covariance grows, so an unobserved ball
becomes uncertain and is eventually reported LOST. Structure follows the usual
ball-model idea (see refs/BHumanCodeRelease Modeling/BallStateEstimator); no code copied.
"""

from dataclasses import dataclass
import math

import numpy as np

OBSERVED, ESTIMATED, LOST = 0, 1, 2


@dataclass
class BallFilterParams:
    """Tuning values; all are starting assumptions to tune in simulation and on carpet."""

    friction_tau: float = 1.5        # [s] velocity e-folding time (assumption, measure)
    accel_noise: float = 0.5         # [m/s^2] white acceleration noise std
    meas_sigma_base: float = 0.02    # [m] measurement std at 0 m
    meas_sigma_per_m2: float = 0.03  # [m/m^2] extra std per squared distance
    init_vel_sigma: float = 0.5      # [m/s] velocity std on (re)initialisation
    gate_chi2: float = 13.8          # Mahalanobis^2 gate (2 dof, 99.9 %)
    observed_timeout: float = 0.3    # [s] OBSERVED while the last fix is this recent
    lost_timeout: float = 5.0        # [s] LOST after this long without a fix
    lost_sigma: float = 0.5          # [m] LOST when the worst-axis std exceeds this
    max_speed: float = 3.0           # [m/s] clamp for numerical sanity


class BallFilter:
    """Kalman filter; time arguments are seconds on any monotonic clock."""

    def __init__(self, params=None, bounds=None, ball_radius=0.021335):
        self.p = params or BallFilterParams()
        self.bounds = bounds          # pinky_soccer_field Rect or None
        self.radius = ball_radius
        self.x = None                 # state vector, None until the first fix
        self.P = None
        self.t = None                 # time of the state
        self.last_seen = None         # time of the last accepted fix
        self.rejected = 0

    @property
    def initialized(self):
        """Return True once a first observation was accepted."""
        return self.x is not None

    def reset(self, z, t, sigma):
        """Start from a single position fix with large velocity uncertainty."""
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        v = self.p.init_vel_sigma ** 2
        self.P = np.diag([sigma ** 2, sigma ** 2, v, v])
        self.t = t
        self.last_seen = t

    def predict(self, t):
        """Propagate to time t (no-op for t <= current time)."""
        if not self.initialized or t <= self.t:
            return
        dt = t - self.t
        decay = math.exp(-dt / self.p.friction_tau)
        # Integral of the decaying velocity over dt.
        gain = self.p.friction_tau * (1.0 - decay)
        F = np.array([[1, 0, gain, 0], [0, 1, 0, gain], [0, 0, decay, 0], [0, 0, 0, decay]])
        q = self.p.accel_noise ** 2
        G = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q
        self.t = t
        self._constrain()

    def measurement_sigma(self, distance):
        """Return the position std of a camera fix at the given range [m]."""
        return self.p.meas_sigma_base + self.p.meas_sigma_per_m2 * distance * distance

    def update(self, z, t, distance=0.0):
        """Fuse a map-frame position fix taken at time t. Return True if accepted."""
        sigma = self.measurement_sigma(distance)
        if not self.initialized or self.state(t) == LOST:
            self.reset(z, t, sigma)
            return True
        self.predict(t)
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        R = np.eye(2) * sigma ** 2
        y = np.asarray(z, float) - H @ self.x
        S = H @ self.P @ H.T + R
        if float(y @ np.linalg.solve(S, y)) > self.p.gate_chi2:
            self.rejected += 1
            if self.rejected >= 3:      # consistent jumps: the ball was moved, re-acquire
                self.reset(z, t, sigma)
                self.rejected = 0
                return True
            return False
        self.rejected = 0
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        self.last_seen = t
        self._constrain()
        return True

    def _constrain(self):
        speed = math.hypot(self.x[2], self.x[3])
        if speed > self.p.max_speed:
            self.x[2:] *= self.p.max_speed / speed
        if self.bounds is None:
            return
        x, y = self.bounds.clamp(self.x[0], self.x[1], self.radius)
        if x != self.x[0]:
            self.x[2] = 0.0             # stopped by the wall (inelastic assumption)
        if y != self.x[1]:
            self.x[3] = 0.0
        self.x[0], self.x[1] = x, y

    def max_sigma(self):
        """Return the largest position standard deviation [m]."""
        if not self.initialized:
            return math.inf
        return math.sqrt(max(np.linalg.eigvalsh(self.P[:2, :2])))

    def state(self, t):
        """Return OBSERVED / ESTIMATED / LOST at time t (state already predicted to t)."""
        if not self.initialized:
            return LOST
        age = t - self.last_seen
        if age <= self.p.observed_timeout:
            return OBSERVED
        if age > self.p.lost_timeout or self.max_sigma() > self.p.lost_sigma:
            return LOST
        return ESTIMATED

    def covariance3(self):
        """Return the row-major 3x3 (x, y, theta) covariance for TrackedObject."""
        cov = [0.0] * 9
        if self.initialized:
            cov[0], cov[1] = self.P[0, 0], self.P[0, 1]
            cov[3], cov[4] = self.P[1, 0], self.P[1, 1]
        else:
            cov[0] = cov[4] = 1e6
        cov[8] = 1e6                    # ball heading is meaningless
        return cov
