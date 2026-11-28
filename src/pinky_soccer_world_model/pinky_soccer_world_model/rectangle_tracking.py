"""Sensor-only startup aiming and bounded odometry-assisted rectangle tracking."""
import math

from .geometry import compose2d, inverse2d, transform_point2d, wrap
from .heading_reference import mirrored


def goal_aims(pose, length, camera_xy=(0., 0.), camera_yaw=0.):
    """Short-side midpoints; identical relative turns under a pi-mirrored pose."""
    cx, cy = transform_point2d(pose, camera_xy)
    return [dict(sign=sign, center=(sign*length/2, 0.),
                 error=wrap(math.atan2(-cy, sign*length/2-cx)-pose[2]-camera_yaw))
            for sign in (-1, 1)]


class GoalInitializer:
    def __init__(self, length, turn_rate=.3, tolerance=.035, settle=.5, view_timeout=2.5,
                 heading_source='imu'):
        self.heading_source = heading_source
        self.length, self.turn_rate, self.tolerance = length, turn_rate, tolerance
        self.settle, self.view_timeout = settle, view_timeout
        self.state = 'FITTING_RECTANGLE'
        self.target = None
        self.attempted = []
        self.stopped_at = None
        self.offset = None  # selected heading - map yaw, maintained by trusted walls
        self.initial_pose = None
        self.anchors = {}

    def step(self, pose, now, camera_xy=(0., 0.), camera_yaw=0., speed=0.):
        if self.offset is not None or self.state == 'GOAL_COLOR_FAILED':
            return 0.
        options = goal_aims(pose, self.length, camera_xy, camera_yaw)
        # A geometric fit may switch its provisional pi branch. Keep the same
        # physical end by storing its relative identity in the caller's pose.
        if self.target is None:
            self.target = min(options, key=lambda g: abs(g['error']))['sign']
        item = next(g for g in options if g['sign'] == self.target)
        self.error, self.centers = item['error'], options
        if abs(self.error) > self.tolerance:
            self.stopped_at = None
            self.state = 'AIMING_GOAL'
            return math.copysign(self.turn_rate, self.error)
        if abs(speed) > .05:
            self.stopped_at = None
        elif self.stopped_at is None:
            self.stopped_at = now
        elapsed = 0. if self.stopped_at is None else now-self.stopped_at
        self.state = 'READING_GOAL' if elapsed >= self.settle else 'SETTLING_GOAL'
        if elapsed >= self.settle+self.view_timeout:
            self.attempted.append(self.target)
            if len(self.attempted) == 2:
                self.state = 'GOAL_COLOR_FAILED'
            else:
                self.target = -self.target
                self.stopped_at = None
                self.state = 'AIMING_GOAL'
        return 0.

    def identify(self, pose, color, imu_yaw, stamp):
        if self.state != 'READING_GOAL' or color not in ('yellow', 'blue'):
            return None
        color_sign = 1 if color == 'blue' else -1
        chosen = pose if self.target == color_sign else mirrored(pose)
        self.offset = wrap(imu_yaw-chosen[2])
        self.initial_pose = chosen
        self.state = 'READY'
        self.anchors = {c: dict(blue_axis_imu_yaw=self.offset,
            goal_axis_imu_yaw=wrap(self.offset+(math.pi if c == 'yellow' else 0.)),
            observed=c == color, observation_stamp=stamp if c == color else None,
            raw_observation_imu_yaw=imu_yaw if c == color else None)
            for c in ('yellow', 'blue')}
        if self.heading_source == 'odom':
            self.anchors = {c: dict(heading_source='odom',
                blue_axis_heading_yaw=self.offset,
                goal_axis_heading_yaw=wrap(self.offset+(math.pi if c == 'yellow' else 0.)),
                observed=c == color, observation_stamp=stamp if c == color else None,
                raw_observation_heading_yaw=imu_yaw if c == color else None)
                for c in ('yellow', 'blue')}
        return chosen


class RectangleTracker:
    def __init__(self, coast=.8, reinitialize=3., max_position_innovation=.12,
                 max_yaw_innovation=.25, imu_limit=.5, drift_gain=.02, heading_source='imu'):
        self.heading_source = heading_source
        self.coast, self.reinitialize = coast, reinitialize
        self.max_position, self.max_yaw = max_position_innovation, max_yaw_innovation
        self.imu_limit, self.drift_gain = imu_limit, drift_gain
        self.pose = self.odom = self.imu = self.stamp = self.last_fix = self.offset = None
        self.rejected = 0
        self.requires_reinitialization = False
        self.recovery_streak = 0
        self.recovery_stamp = -math.inf

    def seed(self, pose, odom, imu_yaw, stamp):
        self.pose, self.odom, self.imu = pose, odom, imu_yaw
        self.offset, self.stamp, self.last_fix = wrap(imu_yaw-pose[2]), stamp, stamp
        self.requires_reinitialization = False
        self.recovery_streak = 0
        self.recovery_stamp = -math.inf

    def predict(self, odom, imu_yaw, stamp):
        if self.pose is None or stamp < self.stamp:
            return None
        delta = compose2d(inverse2d(self.odom), odom)
        predicted = compose2d(self.pose, delta)
        # Incremental heading avoids injecting accumulated offset drift as a jump.
        predicted = (*predicted[:2], wrap(self.pose[2]+wrap(imu_yaw-self.imu)))
        self.pose, self.odom, self.imu, self.stamp = predicted, odom, imu_yaw, stamp
        return predicted

    def correct(self, hypotheses, imu_yaw, stamp):
        if self.last_fix is not None and stamp-self.last_fix > self.reinitialize:
            self.requires_reinitialization = True
        if self.pose is None or self.requires_reinitialization:
            return False
        return self.accept_candidate(hypotheses, imu_yaw, stamp)

    def candidate(self, hypotheses, imu_yaw):
        if self.pose is None:
            return None
        expected = self.pose[2] if self.heading_source == 'odom' else wrap(imu_yaw-self.offset)
        errors = [abs(wrap(p[2]-expected)) for p in hypotheses]
        index = int(errors[1] < errors[0])
        chosen = hypotheses[index]
        if (errors[index] > self.imu_limit or errors[1-index]-errors[index] < .7 or
                math.hypot(chosen[0]-self.pose[0], chosen[1]-self.pose[1]) > self.max_position or
                abs(wrap(chosen[2]-self.pose[2])) > self.max_yaw):
            self.rejected += 1
            return None
        return chosen

    def accept_candidate(self, hypotheses, imu_yaw, stamp):
        chosen = self.candidate(hypotheses, imu_yaw)
        if chosen is None:
            return False
        self.pose, self.last_fix = chosen, stamp
        self.offset = (wrap(imu_yaw-chosen[2]) if self.heading_source == 'odom' else
                       wrap(self.offset+self.drift_gain*wrap(imu_yaw-chosen[2]-self.offset)))
        return True

    def try_recover(self, hypotheses, heading, stamp):
        """Restore the existing map branch only after three plausible new fits."""
        if stamp <= self.recovery_stamp:
            return False
        if stamp-self.recovery_stamp > self.coast:
            self.recovery_streak = 0
        self.recovery_stamp = stamp
        chosen = self.candidate(hypotheses, heading)
        if chosen is None:
            self.recovery_streak = 0
            return False
        self.recovery_streak += 1
        if self.recovery_streak < 3:
            return False
        if not self.accept_candidate(hypotheses, heading, stamp):
            return False
        self.requires_reinitialization = False
        self.recovery_streak = 0
        return True

    def validity(self, stamp):
        if self.last_fix is None:
            return False, 'UNINITIALIZED', 0.
        age = max(0., stamp-self.last_fix)
        if age > self.reinitialize or self.requires_reinitialization:
            self.requires_reinitialization = True
            return False, 'REINITIALIZATION_REQUIRED', 0.
        if age > self.coast:
            return False, 'LOCALIZATION_LOST', 0.
        return True, 'READY' if age < .25 else 'ODOM_COAST', max(.1, 1-age/self.coast)
