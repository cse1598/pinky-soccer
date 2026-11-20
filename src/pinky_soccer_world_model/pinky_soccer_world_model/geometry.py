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


def detect_color_blob(bgr, hsv_ranges, min_area=12, min_circularity=0.4):
    """Find the largest blob within HSV ranges. Return (u, v, radius_px, area) or None.

    hsv_ranges: list of ((h, s, v) low, (h, s, v) high) in OpenCV units (H 0..180).
    """
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for low, high in hsv_ranges:
        part = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(c, True)
        circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        if circularity < min_circularity:
            continue
        if best is None or area > best[3]:
            m = cv2.moments(c)
            (_, _), r = cv2.minEnclosingCircle(c)
            best = (m['m10'] / m['m00'], m['m01'] / m['m00'], r, area)
    return best


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


def intersect_plane_z(origin, direction, z):
    """Intersect a ray with the horizontal plane at height z. None if it never hits."""
    if direction[2] > -1e-6:
        return None
    s = (z - origin[2]) / direction[2]
    if s <= 0:
        return None
    return origin + s * direction
