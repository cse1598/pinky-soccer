"""Small, ROS-free geometry helpers: transforms, colour detection, ray casting."""

import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    """Rotation matrix of a unit quaternion."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def yaw_from_quat(x, y, z, w):
    """Yaw (rotation about z) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    """Return (x, y, z, w) for a pure yaw rotation."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def compose2d(a, b):
    """Return pose a * b for 2D poses (x, y, theta)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse2d(a):
    """Return the inverse of 2D pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def transform_point2d(pose, p):
    """Map point p from the frame of `pose` into its parent frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * p[0] - s * p[1], pose[1] + s * p[0] + c * p[1])


BLOB_SHAPE_DEFAULTS = {'min_circle_fill': .72, 'min_axis_ratio': .75,
                       'min_color_fraction': .55, 'max_radial_error': .15}


def color_mask(bgr, hsv_ranges):
    """Return the actual HSV selection before morphological shape processing."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    for low, high in hsv_ranges:
        mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    return mask


def _compact_blob(bgr, hsv_ranges, min_area=12, min_circularity=.72, *,
                  min_circle_fill=.72, min_axis_ratio=.75,
                  min_color_fraction=.55, max_radial_error=.15, mask=None):
    """Select a filled, near-circular colour region; return (u,v,radius,area) or None.

    Centre/radius come from the enclosing outline circle, not the colour centroid:
    dark ball logos and highlights may move the centroid without moving the ball.
    A square can have high circularity; enclosing-circle fill and radial residual
    explicitly reject it. Border-cropped regions are not complete ball observations.
    """
    import cv2
    raw = color_mask(bgr, hsv_ranges) if mask is None else mask
    kernel = np.ones((3, 3), np.uint8)
    clean = cv2.morphologyEx(raw, cv2.MORPH_OPEN, kernel)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if (min(x, y) <= 0 or x+w >= bgr.shape[1] or y+h >= bgr.shape[0]
                or min(w, h)/max(w, h) < min_axis_ratio):
            continue
        perimeter = cv2.arcLength(c, True)
        circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        if circularity < min_circularity:
            continue
        (u, v), r = cv2.minEnclosingCircle(c)
        if r <= 0 or area/(math.pi*r*r) < min_circle_fill:
            continue
        distances = np.linalg.norm(c.reshape(-1, 2)-[u, v], axis=1)
        radial_error = float(np.sqrt(np.mean((distances-r)**2))/r)
        if radial_error > max_radial_error:
            continue
        yy, xx = np.ogrid[y:y+h, x:x+w]
        inside = (xx-u)**2+(yy-v)**2 <= r*r
        fraction = np.count_nonzero(raw[y:y+h, x:x+w][inside])/np.count_nonzero(inside)
        if fraction < min_color_fraction:
            continue
        if best is None or area > best[3]:
            best = (float(u), float(v), float(r), area)
    return best


def detect_color_blob(bgr, hsv_ranges, min_area=12, min_circularity=.72, **shape):
    """Compatibility API using the common lighting/arc detector, with one output."""
    from .robust_ball import observe_ball
    shape.pop('mask', None)
    return observe_ball(bgr, hsv_ranges, min_area, min_circularity, shape).blob


def pixel_ray(u, v, K, D=None):
    """Return the unit ray in the optical frame (x right, y down, z forward) of pixel (u, v)."""
    K = np.asarray(K, float).reshape(3, 3)
    if D is not None and np.any(np.asarray(D, float) != 0):
        import cv2
        pts = cv2.undistortPoints(np.array([[[u, v]]], np.float64), K, np.asarray(D, float))
        x, y = pts[0, 0]
    else:
        x, y = (u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]
    d = np.array([x, y, 1.0])
    return d / np.linalg.norm(d)


def intrinsics_from_hfov(width, height, hfov_deg):
    """Approximate pinhole intrinsics, explicitly not a camera calibration."""
    if width <= 0 or height <= 0 or not math.isfinite(hfov_deg) or not 1 < hfov_deg < 179:
        raise ValueError('Positive image dimensions and HFOV between 1 and 179 required')
    focal = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    return np.array([[focal, 0, (width - 1) / 2],
                     [0, focal, (height - 1) / 2], [0, 0, 1]], float)


def intersect_plane_z(origin, direction, z):
    """Intersect a ray with the horizontal plane at height z. None if it never hits."""
    if direction[2] > -1e-6:
        return None
    s = (z - origin[2]) / direction[2]
    if s <= 0:
        return None
    return origin + s * direction
