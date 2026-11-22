"""ROS-free goal bearing and magnetic symmetry checks for LiDAR localization.

This only chooses between (x,y,yaw) and (-x,-y,yaw+pi). It does not infer a
metric pose from colour or use a simulator's true pose as an observation.
"""

from collections import deque
import math

import cv2
import numpy as np

from .geometry import pixel_ray, transform_point2d, wrap


def mirrored(pose):
    """Return the exactly 180-degree symmetric field hypothesis."""
    return (-pose[0], -pose[1], wrap(pose[2]+math.pi))


def magnetic_heading(field, gravity):
    """Right-handed yaw relative to magnetic north, tilt compensated in base axes.

    `gravity` is the accelerometer's upwards specific force during near-static
    motion, not a signed gravity model vector. Both inputs are in base axes.
    """
    b, up = np.asarray(field, float), np.asarray(gravity, float)
    if not np.isfinite(b).all() or not np.isfinite(up).all():
        raise ValueError('Nonfinite magnetic/acceleration sample')
    strength, acceleration = np.linalg.norm(b), np.linalg.norm(up)
    if not 10e-6 <= strength <= 100e-6 or abs(acceleration-9.81) > 1.5:
        raise ValueError('Magnetic magnitude or acceleration outside quality limits')
    up = up/acceleration
    front = np.array([1., 0., 0.])-up*up[0]
    front_norm = np.linalg.norm(front)
    if front_norm < .1:
        raise ValueError('Robot forward axis is nearly vertical')
    front /= front_norm
    left = np.cross(up, front)
    bx, by = float(b@front), float(b@left)
    if math.hypot(bx, by) < 5e-6:
        raise ValueError('Horizontal magnetic field too weak')
    return math.atan2(-by, bx), float(strength)


def goal_blobs(frame, ranges, min_area=80.):
    """Detect complete coloured goal panels; reject circles, edge crops and clutter."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width = frame.shape[:2]
    results = []
    for color, (lo, hi) in ranges.items():
        mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if (area < min_area or not 1.5 <= w/h <= 20 or area/(w*h) < .55
                    or min(x, y, width-x-w, height-y-h) < 3):
                continue
            candidates.append((area, (x+w/2., y+h/2.), (x, y, w, h)))
        candidates.sort(reverse=True)
        # Two similarly sized panels of one colour are ambiguous.
        if candidates and (len(candidates) == 1 or candidates[0][0] > 2*candidates[1][0]):
            area, uv, box = candidates[0]
            results.append({'color': color, 'uv': uv, 'box': box, 'area': area})
    return results


def observed_bearing(uv, k, d, base_from_optical):
    """Account for intrinsics, distortion and the actual camera mounting axes."""
    ray = base_from_optical[:3, :3] @ pixel_ray(*uv, k, d)
    if not np.isfinite(ray).all() or np.linalg.norm(ray[:2]) < .1:
        raise ValueError('Goal ray is too close to vertical')
    return math.atan2(ray[1], ray[0])


def goal_error(field, pose, color, bearing, camera_xy=(0., 0.)):
    """Angular residual to the centre of the visible coloured goal back plate."""
    a, b = field.goal_back(color)
    gx, gy = (a[0]+b[0])/2, (a[1]+b[1])/2
    cx, cy = transform_point2d(pose, camera_xy)
    return abs(wrap(math.atan2(gy-cy, gx-cx)-pose[2]-bearing))


class HeadingReference:
    """Learn field/magnetic offset from repeated visual identity observations."""

    def __init__(self, field, confirmations=5):
        self.field = field
        self.confirmations = confirmations
        self.samples = deque(maxlen=confirmations)
        self.offset = None
        self.strength = None
        self.vote = 0

    def visual(self, pose, color, bearing, heading, strength, camera_xy=(0., 0.)):
        """Return chosen pose/flip, after a consistent series of independent images.

        Observation is only admissible when the geometric bearing identifies one
        candidate strongly. Seeing blue alone does NOT mean the robot faces +X.
        """
        if not all(math.isfinite(v) for v in (*pose, bearing, heading, strength, *camera_xy)):
            self.samples.clear()
            return None
        flip = mirrored(pose)
        errors = [goal_error(self.field, p, color, bearing, camera_xy) for p in (pose, flip)]
        index = int(errors[1] < errors[0])
        if errors[index] > .25 or errors[1-index]-errors[index] < .7:
            self.samples.clear()
            return None
        selected = (pose, flip)[index]
        offset = wrap(selected[2]-heading)
        self.samples.append((offset, strength, index))
        if len(self.samples) < self.confirmations:
            return None
        mean = math.atan2(sum(math.sin(v[0]) for v in self.samples),
                          sum(math.cos(v[0]) for v in self.samples))
        if (max(abs(wrap(v[0]-mean)) for v in self.samples) > .12
                or len({v[2] for v in self.samples}) != 1):
            return None
        # A contradictory new visual anchor requires a new consensus; update it
        # only from these qualified colour observations, never from AMCL alone.
        self.offset = mean
        self.strength = float(np.mean([v[1] for v in self.samples]))
        return {'pose': selected, 'flip': bool(index), 'offset': mean, 'errors': errors,
                'color': color, 'bearing': bearing, 'magnetic_heading': heading,
                'strength_tesla': strength}

    def magnetic(self, pose, heading, strength):
        """Only correct a clear 180° alias; noisy moderate yaw errors do not reseed."""
        if self.offset is None or abs(strength/self.strength-1) > .2:
            self.vote = 0
            return None
        expected = wrap(heading+self.offset)
        error = abs(wrap(pose[2]-expected))
        self.vote = self.vote+1 if error > math.radians(145) else 0
        if self.vote < self.confirmations:
            return None
        self.vote = 0
        return {'pose': mirrored(pose), 'flip': True, 'magnetic_heading': heading,
                'offset': self.offset, 'error_rad': error, 'strength_tesla': strength}
