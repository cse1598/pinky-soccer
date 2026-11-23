"""
Sensor models for the skill bench. Turn ground truth into what the robot would know.

Camera: the ball is detected when its whole image circle is inside the frame (the real
detector rejects circles cut by the frame edge), within max range and not occluded. Fixes
arrive at the camera rate with latency and range-dependent noise and are fused by the same
BallFilter as the world model. Self pose: truth plus small noise (AMCL-like, assumption).
Ultrasonic: cone rays against world/ball/opponents with the simulator's min/max range rules.
All noise values are assumptions to be replaced by measurements.
"""

from dataclasses import dataclass
import math
import random

import numpy as np

from ..context import BallEstimate, ESTIMATED, LOST, OBSERVED, SkillContext
from ..geometry import Pose


@dataclass
class NoiseModel:
    """Perception assumptions (not measured)."""

    camera_latency: float = 0.08        # [s] capture -> fix available
    camera_fps: float = 15.0
    max_range: float = 2.5              # [m] detector range
    pos_sigma: float = 0.006            # [m] fix noise at 0 m
    pos_sigma_per_m: float = 0.02       # [m/m] growth with range
    dropout: float = 0.05               # probability a visible ball is missed
    # visible share of the circle outline needed (robust_ball accepts >= 0.42; conservative)
    min_arc: float = 0.5
    partial_sigma_gain: float = 2.0     # noise multiplier for a circle cut by the frame
    pose_sigma: float = 0.008           # [m] self localisation error (slowly varying)
    heading_sigma: float = 0.01         # [rad]
    pose_tau: float = 2.0               # [s] correlation time of the localisation error
    us_sigma: float = 0.003             # [m]
    us_rate: float = 10.0


class Perception:
    """Builds SkillContext from a SoccerWorld."""

    def __init__(self, world, field, noise=None, seed=0, filter_params=None):
        from pinky_soccer_world_model.ball_filter import BallFilter, BallFilterParams
        self.w = world
        self.field = field
        self.n = noise or NoiseModel()
        self.rng = random.Random(seed)
        self.filter = BallFilter(filter_params or BallFilterParams(), field_rect(world, field),
                                 world.ball_radius)
        cam = world.camera_cfg
        self.width, self.height = cam['width'], cam['height']
        self.fx = self.width / 2.0 / math.tan(cam['horizontal_fov'] / 2.0)
        self.next_frame = 0.0
        self.pending = []                  # (available_time, capture_time, map_xy, range)
        self.next_us = 0.0
        self.us = math.nan
        self.history = []                  # (t, true pose) for latency-aware transforms
        self.visible_now = False
        self.frame_now = False
        self.last_t = None
        self.pose_err = [self.rng.gauss(0, self.n.pose_sigma),
                         self.rng.gauss(0, self.n.pose_sigma),
                         self.rng.gauss(0, self.n.heading_sigma)]

    # ------------------------------------------------------------------ camera
    def ball_visible(self):
        """Return (visible, range, arc share) for the true ball from the camera pose."""
        pos, rot = self.w.camera_pose()
        bx, by, _, _ = self.w.ball_state()
        ball = np.array([bx, by, self.w.ball_radius])
        rel = rot.T @ (ball - pos)                    # camera frame: x right, y up, -z fwd
        depth = -rel[2]
        rng = float(np.linalg.norm(ball - pos))
        if depth <= 0.02 or rng > self.n.max_range:
            return False, rng, 0.0
        u = self.fx * rel[0] / depth
        v = self.fx * rel[1] / depth
        r_px = self.fx * self.w.ball_radius / depth
        a = np.linspace(0, 2 * math.pi, 48, endpoint=False)
        inside = (np.abs(u + r_px * np.cos(a)) <= self.width / 2.0) & \
            (np.abs(v + r_px * np.sin(a)) <= self.height / 2.0)
        arc = float(inside.mean())
        if arc < self.n.min_arc:
            return False, rng, arc
        # Occlusion: any of centre/top/sides hidden by another object hides the ball.
        d, geom = self.w.ray(pos, (ball - pos) / rng, rng + 0.05)
        if geom != self.w.ball_geom:
            return False, rng, arc
        return True, rng, arc

    def tick(self):
        """Advance sensor timing; call every control cycle. Returns SkillContext."""
        t = self.w.time
        truth = self.w.robot_pose()
        self.history.append((t, truth))
        while self.history and self.history[0][0] < t - 1.0:
            self.history.pop(0)
        self.frame_now = t >= self.next_frame
        if self.frame_now:
            self.next_frame = t + 1.0 / self.n.camera_fps
            visible, rng, arc = self.ball_visible()
            self.visible_now = visible
            if visible and self.rng.random() > self.n.dropout:
                bx, by, _, _ = self.w.ball_state()
                s = self.n.pos_sigma + self.n.pos_sigma_per_m * rng
                if arc < 0.999:
                    s *= self.n.partial_sigma_gain
                z = (bx + self.rng.gauss(0, s), by + self.rng.gauss(0, s))
                self.pending.append((t + self.n.camera_latency, t, z, rng))
        seen = False
        while self.pending and self.pending[0][0] <= t:
            _, tc, z, rng = self.pending.pop(0)
            self.filter.update(z, tc, rng)
            seen = True
        self.filter.predict(t)
        if t >= self.next_us:
            self.next_us = t + 1.0 / self.n.us_rate
            self.us = self.ultrasonic()
        dt = t - self.last_t if self.last_t is not None else 0.0
        self.last_t = t
        k = math.exp(-dt / self.n.pose_tau) if dt > 0 else 1.0
        q = math.sqrt(max(0.0, 1.0 - k * k))
        sig = (self.n.pose_sigma, self.n.pose_sigma, self.n.heading_sigma)
        self.pose_err = [k * e + q * self.rng.gauss(0, sg) for e, sg in zip(self.pose_err, sig)]
        pose = Pose(truth[0] + self.pose_err[0], truth[1] + self.pose_err[1],
                    truth[2] + self.pose_err[2])
        v, w = self.w.robot_twist()
        return SkillContext(t=t, pose=pose, v=v, w=w, ball=self.ball_estimate(t, seen),
                            us_range=self.us, opponents=self.opponents(), field=self.field)

    def ball_estimate(self, t, seen):
        """Return the filter state as a BallEstimate (None before the first fix)."""
        f = self.filter
        if not f.initialized:
            return None
        state = {0: OBSERVED, 1: ESTIMATED, 2: LOST}[f.state(t)]
        x, y, vx, vy = (float(v) for v in f.x)
        return BallEstimate(x, y, vx, vy, state, t - f.last_seen, f.max_sigma(), seen)

    def opponents(self):
        """Opponent centres with small noise (LiDAR clustering stand-in)."""
        out = []
        for i in range(len(self.w.opponent_mocap)):
            x, y = self.w.opponent_xy(i)
            if abs(x) < 4:
                out.append((x + self.rng.gauss(0, 0.01), y + self.rng.gauss(0, 0.01)))
        return out

    # ------------------------------------------------------------------ ultrasonic
    def ultrasonic(self):
        """Range like the simulator adapter: min of cone rays, -inf/inf outside limits."""
        cfg = self.w.sensor_cfg['ultrasonic']
        origin, axis = self.w.ultrasonic_origin()
        half = cfg['field_of_view'] / 2.0
        best = math.inf
        for a in np.linspace(-half, half, cfg.get('samples', 5)):
            c, s = math.cos(a), math.sin(a)
            d = np.array([c * axis[0] - s * axis[1], s * axis[0] + c * axis[1], axis[2]])
            dist, _ = self.w.ray(origin, d / np.linalg.norm(d), cfg['max_range'] + 0.1)
            best = min(best, dist)
        if best < cfg['min_range']:
            return -math.inf
        if best > cfg['max_range']:
            return math.inf
        return best + self.rng.gauss(0, self.n.us_sigma)


def field_rect(world, field):
    """Rect of the inner walls for the ball filter."""
    from pinky_soccer_field.field_geometry import Rect
    return Rect(-field.wall_x, -field.wall_y, field.wall_x, field.wall_y)
