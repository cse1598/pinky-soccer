"""Shared display rules for both UIs (RViz markers and the OpenCV top view).

| object   | OBSERVED                 | ESTIMATED                         | LOST              |
|----------|--------------------------|-----------------------------------|-------------------|
| self     | solid robot + arrow      | + covariance ellipse              | -                 |
| ball     | solid circle             | faint circle + covariance ellipse | X at last pose    |
| opponent | solid box                | faint box + seconds since seen    | hidden            |
"""

import math

import numpy as np

OBSERVED, ESTIMATED, LOST = 0, 1, 2
STATE_NAMES = {OBSERVED: 'OBSERVED', ESTIMATED: 'ESTIMATED', LOST: 'LOST'}

# RGB in 0..1 (RViz); the top view converts to BGR 0..255.
COLORS = {
    'carpet': (0.13, 0.45, 0.20),
    'wall': (0.05, 0.05, 0.05),
    'line': (0.95, 0.95, 0.95),
    'yellow': (0.95, 0.80, 0.10),
    'blue': (0.15, 0.35, 0.90),
    'self': (0.10, 0.80, 0.95),
    'ball': (1.00, 0.45, 0.00),
    'opponent': (0.85, 0.15, 0.15),
    'fov': (1.00, 1.00, 0.60),
    'target': (0.90, 0.10, 0.90),
    'path': (0.90, 0.40, 0.90),
    'truth': (0.60, 0.60, 0.60),
    'center': (0.05, 0.05, 0.05),
}
SOLID_ALPHA, FAINT_ALPHA = 1.0, 0.35
ELLIPSE_SIGMA = 2.0          # covariance ellipses are drawn at 2 sigma
PREDICTION_SEC = 1.5         # ball prediction horizon (constant velocity, dotted)
ROBOT_RADIUS = 0.06          # [m] drawing only
OPPONENT_SIZE = 0.12         # [m] drawing only


def bgr(name, scale=1.0):
    """Return an OpenCV BGR tuple for a colour name."""
    r, g, b = COLORS[name]
    return (int(b * 255 * scale), int(g * 255 * scale), int(r * 255 * scale))


def covariance_ellipse(cov9, sigma=ELLIPSE_SIGMA):
    """Return (semi_major, semi_minor, angle_rad) of the xy block of a 3x3 covariance."""
    c = np.array([[cov9[0], cov9[1]], [cov9[3], cov9[4]]], float)
    c = 0.5 * (c + c.T)
    vals, vecs = np.linalg.eigh(c)
    vals = np.clip(vals, 0.0, None)
    major = sigma * math.sqrt(vals[1])
    minor = sigma * math.sqrt(vals[0])
    angle = math.atan2(vecs[1, 1], vecs[0, 1])
    return major, minor, angle


def ball_prediction(pose, velocity, horizon=PREDICTION_SEC, steps=10):
    """Return points of the constant-velocity prediction (dotted line)."""
    return [(pose[0] + velocity[0] * horizon * k / steps,
             pose[1] + velocity[1] * horizon * k / steps) for k in range(steps + 1)]


def fov_polygon(pose, hfov, max_range, segments=12):
    """Return a polygon (list of (x, y)) of the camera field of view wedge."""
    x, y, th = pose
    pts = [(x, y)]
    for k in range(segments + 1):
        a = th - hfov / 2 + hfov * k / segments
        pts.append((x + max_range * math.cos(a), y + max_range * math.sin(a)))
    return pts
