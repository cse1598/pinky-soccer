"""Opponent candidates from a 2D scan: points left after removing the known walls."""

import numpy as np


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Return (N, 2) points in the scan frame for finite ranges inside the valid band."""
    r = np.asarray(ranges, float)
    a = angle_min + angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def cluster_opponents(points, walls, wall_margin=0.06, gap=0.05, min_points=3,
                      max_size=0.30):
    """Group map-frame points that are not walls into opponent-sized clusters.

    points: (N, 2) in scan order (neighbouring beams adjacent). walls: Rect of the inner
    wall faces. Returns a list of (x, y, size) cluster centres. Points outside the walls
    shrunk by wall_margin are treated as wall returns and dropped.
    """
    if len(points) == 0:
        return []
    keep = np.array([walls.contains(x, y, wall_margin) for x, y in points])
    pts = points[keep]
    if len(pts) == 0:
        return []
    clusters, current = [], [pts[0]]
    for p in pts[1:]:
        if np.hypot(*(p - current[-1])) <= gap:
            current.append(p)
        else:
            clusters.append(current)
            current = [p]
    clusters.append(current)
    # The scan wraps around: join the last and first clusters when they touch.
    if len(clusters) > 1 and np.hypot(*(clusters[0][0] - clusters[-1][-1])) <= gap:
        clusters[0] = clusters.pop() + clusters[0]
    out = []
    for c in clusters:
        c = np.asarray(c)
        size = float(np.max(np.hypot(*(c - c.mean(axis=0)).T)) * 2) if len(c) > 1 else 0.0
        if len(c) >= min_points and size <= max_size:
            out.append((float(c[:, 0].mean()), float(c[:, 1].mean()), size))
    return out
