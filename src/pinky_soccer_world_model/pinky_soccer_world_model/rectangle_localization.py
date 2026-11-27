"""ROS-free rectangular-wall fit and two IMU heading references.

Only a copy of a scan is filtered. Internal returns remain available to the
obstacle perception/control stack on the original scan topic.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .geometry import wrap
from .heading_reference import goal_error, mirrored


@dataclass
class RectangleFit:
    pose: tuple
    mask: np.ndarray
    residual: float
    wall_counts: tuple
    measured_size: tuple

    @property
    def hypotheses(self):
        return (self.pose, mirrored(self.pose))


def _pair(values, extent, tolerance, minimum):
    """Find two supported parallel lines at approximately the known separation."""
    bins = np.floor(values/tolerance).astype(int)
    unique, counts = np.unique(bins, return_counts=True)
    peaks = (unique[np.argsort(counts)[-8:]]+.5)*tolerance
    centers = np.r_[peaks+extent/2, peaks-extent/2]
    left = np.abs(values[None, :]-centers[:, None]+extent/2) <= tolerance
    right = np.abs(values[None, :]-centers[:, None]-extent/2) <= tolerance
    nl, nr = left.sum(axis=1), right.sum(axis=1)
    scores = np.where((nl >= minimum) & (nr >= minimum), nl+nr, -1)
    if scores.max() < 0:
        return None
    i = int(np.argmax(scores))
    lo, hi = float(np.median(values[left[i]])), float(np.median(values[right[i]]))
    return (lo+hi)/2, hi-lo


def fit_rectangle(points, length, width, tolerance=.025, minimum=12, prior_yaw=None):
    """Fit four outer wall lines, ignoring internal clutter and isolated outliers.

    Geometry comes from the selected field, not a simulator's truth. All four
    sides must be observed; a partially hidden rectangle is not invented.
    Returns the two equivalent pi-separated poses via RectangleFit.hypotheses.
    """
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('points must be Nx2 in base_footprint')
    valid = np.isfinite(points).all(axis=1)
    p = points[valid]
    if len(p) < 4*minimum or length <= width or width <= 0:
        return None

    def candidate(angle):
        c, s = math.cos(angle), math.sin(angle)
        q = p @ np.array([[c, s], [-s, c]])
        a = _pair(q[:, 0], length, tolerance, minimum)
        b = _pair(q[:, 1], width, tolerance, minimum)
        if a is None or b is None:
            return None
        center = np.array([a[0], b[0]])
        if np.any(np.abs(center) >= np.array([length, width])/2-.025):
            return None  # sensor/robot must be inside the enclosure
        z = q-center
        dx, dy = np.abs(np.abs(z)-[length/2, width/2]).T
        mx = (dx < tolerance) & (np.abs(z[:, 1]) <= width/2+tolerance)
        my = (dy < tolerance) & (np.abs(z[:, 0]) <= length/2+tolerance)
        masks = (mx & (z[:, 0] < 0), mx & (z[:, 0] > 0),
                 my & (z[:, 1] < 0), my & (z[:, 1] > 0))
        counts = tuple(int(m.sum()) for m in masks)
        if min(counts) < minimum:
            return None
        for index, mask in enumerate(masks):
            tangent = z[mask, 1 if index < 2 else 0]
            if np.ptp(tangent) < .15*(width if index < 2 else length):
                return None
        keep = mx | my
        residual = float(np.sqrt(np.mean(np.minimum(dx[keep], dy[keep])**2)))
        score = int(keep.sum()) - 2*len(p)*residual/tolerance
        mask = np.zeros(len(points), dtype=bool)
        mask[np.flatnonzero(valid)] = keep
        fit = RectangleFit((-float(center[0]), -float(center[1]), wrap(angle)), mask,
                           residual, counts, (a[1], b[1]))
        return score, fit

    best = None
    if prior_yaw is not None:
        # Usually the IMU-propagated heading is already close. Refine the four
        # supported lines by 2x2 least squares before any angle sweep. Clutter
        # is excluded by the same known-size support tests as the full search.
        fast = candidate(float(prior_yaw))
        if fast:
            angle = float(prior_yaw)
            for _ in range(3):
                c, s = math.cos(angle), math.sin(angle)
                q = p @ np.array([[c, s], [-s, c]])
                center = -np.asarray(fast[1].pose[:2])
                z = q-center
                distance = np.abs(np.abs(z)-[length/2, width/2])
                wall_axis = np.argmin(distance, axis=1)
                tilts, weights = [], []
                keep = fast[1].mask[valid]
                for axis in (0, 1):
                    for sign in (-1, 1):
                        cloud = q[keep & (wall_axis == axis) & (np.sign(z[:, axis]) == sign)]
                        if len(cloud) < minimum:
                            continue
                        cov = np.cov(cloud.T, bias=True)
                        phi = .5*math.atan2(2*cov[0, 1], cov[0, 0]-cov[1, 1])
                        tilt = wrap(phi-(math.pi/2 if axis == 0 else 0.))
                        if tilt > math.pi/2:
                            tilt -= math.pi
                        elif tilt < -math.pi/2:
                            tilt += math.pi
                        tilts.append(tilt)
                        weights.append(len(cloud)*float(np.trace(cov)))
                if not weights:
                    break
                angle = wrap(angle-float(np.average(tilts, weights=weights)))
                refined = candidate(angle)
                if refined is None:
                    break
                fast = refined
            if fast[1].residual < tolerance*.5:
                return fast[1]
    angles = (np.deg2rad(np.arange(-90., 90., 1.)) if prior_yaw is None else
              prior_yaw+np.deg2rad(np.arange(-8., 8.01, 1.)))
    for angle in angles:
        item = candidate(float(angle))
        if item and (best is None or item[0] > best[0]):
            best = item
    if best is None:
        return None
    around = best[1].pose[2]
    for delta in np.deg2rad(np.arange(-1., 1.001, .1)):
        item = candidate(wrap(around+float(delta)))
        if item and item[0] > best[0]:
            best = item
    return best[1]


class GoalImuReferences:
    """Record both colours, then compare two wrapped angular distances only.

    The raw IMU angle of an off-axis goal observation is saved, but its corrected
    axis reference uses the rectangular pose and calibrated camera bearing.
    Seeing a blue patch is not by itself evidence that the robot faces +X.
    """
    def __init__(self, field, confirmations=4):
        self.field = field
        self.confirmations = confirmations
        self.samples = {c: deque(maxlen=confirmations) for c in ('yellow', 'blue')}
        self.references = {}
        self.last_image_stamp = None

    def observe(self, hypotheses, color, bearing, imu_yaw, image_stamp, camera_xy=(0., 0.)):
        if color not in self.samples or image_stamp == self.last_image_stamp:
            return False
        if not all(math.isfinite(x) for x in (bearing, imu_yaw, image_stamp)):
            return False
        self.last_image_stamp = image_stamp
        errors = [goal_error(self.field, p, color, bearing, camera_xy) for p in hypotheses]
        chosen = int(errors[1] < errors[0])
        if errors[chosen] > .25 or errors[1-chosen]-errors[chosen] < .7:
            return False
        pose = hypotheses[chosen]
        offset = wrap(imu_yaw-pose[2])
        samples = self.samples[color]
        samples.append(dict(imu_yaw=imu_yaw, bearing=bearing, pose=list(pose),
                            image_stamp=image_stamp, offset=offset))
        if len(samples) < self.confirmations:
            return False
        mean = math.atan2(sum(math.sin(s['offset']) for s in samples),
                          sum(math.cos(s['offset']) for s in samples))
        if max(abs(wrap(s['offset']-mean)) for s in samples) > .1:
            return False
        other = self.references.get('blue' if color == 'yellow' else 'yellow')
        if other and abs(wrap(other['blue_axis_imu_yaw']-mean)) > .2:
            return False
        self.references[color] = dict(blue_axis_imu_yaw=mean,
            goal_axis_imu_yaw=wrap(mean+(math.pi if color == 'yellow' else 0.)),
            raw_observations=list(samples))
        return True

    @property
    def complete(self):
        return len(self.references) == 2

    def select(self, hypotheses, imu_yaw):
        """Compare current IMU minus provisional yaw with blue vs yellow anchors."""
        if not self.complete or not math.isfinite(imu_yaw):
            return None
        offsets = [r['blue_axis_imu_yaw'] for r in self.references.values()]
        blue = math.atan2(sum(math.sin(v) for v in offsets), sum(math.cos(v) for v in offsets))
        yellow = wrap(blue+math.pi)
        delta = wrap(imu_yaw-hypotheses[0][2])
        errors = [abs(wrap(delta-blue)), abs(wrap(delta-yellow))]
        i = int(errors[1] < errors[0])
        if errors[i] > .35 or errors[1-i]-errors[i] < .7:
            return None
        return dict(pose=hypotheses[i], branch=i, errors=errors,
                    blue_axis_imu_yaw=blue, yellow_axis_imu_yaw=yellow)


class StartupGate:
    """ROS-free single-owner command policy for startup and missing inputs."""
    def __init__(self):
        self.aborted = False
        self.ever_ready = False

    def command(self, ready, sensors_fresh, camera_fresh, have_rectangle,
                within_budget, turn_rate=.3):
        if self.aborted:
            return 0., 0., 'ABORTED'
        if ready:
            self.ever_ready = True
            return 0., 0., 'READY'
        if self.ever_ready:
            return 0., 0., 'LOCALIZATION_LOST'
        if not within_budget:
            return 0., 0., 'CALIBRATION_TIMEOUT'
        if not (sensors_fresh and camera_fresh and have_rectangle):
            return 0., 0., 'WAITING_FOR_ROBOT'
        return 0., turn_rate, 'SEEKING_GOALS'
