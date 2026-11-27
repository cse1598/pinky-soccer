"""Opponent candidates from a 2D scan: points left after removing the known walls."""

from collections import deque
from dataclasses import dataclass

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


@dataclass(frozen=True)
class Candidate:
    """Observed surface mean, not an inferred robot centre [map metres]."""

    x: float
    y: float
    size: float
    count: int

    @property
    def xy(self):
        return np.array((self.x, self.y))


def indexed_candidates(ranges, angle_min, angle_increment, range_min, range_max,
                       rotation, translation, walls, structures=(), wall_margin=.06,
                       gap=.05, min_points=3, min_size=.015, max_size=.30,
                       self_radius=.08, max_beam_gap=3, max_adaptive_gap=.09):
    """O(N) scan-order clustering. Retain beam holes; wrap only a full revolution.

    Adaptive distance allows sparse far returns but cannot exceed max_adaptive_gap.
    A few missing beams can join fragments only within that same spatial bound.
    No circle, shape completion, or ground-truth input is used.
    """
    r = np.asarray(ranges, float)
    indices = np.arange(len(r))
    angles = angle_min + indices * angle_increment
    valid = np.isfinite(r) & (r >= max(range_min, self_radius)) & (r <= range_max)
    local = np.column_stack((r[valid]*np.cos(angles[valid]),
                             r[valid]*np.sin(angles[valid])))
    pts = local @ np.asarray(rotation)[:2, :2].T + np.asarray(translation)[:2]
    ids, distances = indices[valid], r[valid]
    if not len(pts):
        return []
    keep = ((pts[:, 0] >= walls.xmin+wall_margin) &
            (pts[:, 0] <= walls.xmax-wall_margin) &
            (pts[:, 1] >= walls.ymin+wall_margin) &
            (pts[:, 1] <= walls.ymax-wall_margin))
    for rect in structures:
        # Distance to an existing solid footprint, not its bounding disk.
        dx = np.maximum(np.maximum(rect.xmin-pts[:, 0], 0), pts[:, 0]-rect.xmax)
        dy = np.maximum(np.maximum(rect.ymin-pts[:, 1], 0), pts[:, 1]-rect.ymax)
        keep &= np.hypot(dx, dy) > wall_margin
    pts, ids, distances = pts[keep], ids[keep], distances[keep]
    if not len(pts):
        return []

    def adjacent(a, b, beam_gap):
        threshold = min(max_adaptive_gap, max(gap, 1.8*min(distances[a], distances[b])
                                             * abs(angle_increment)*beam_gap))
        return (beam_gap <= max_beam_gap and
                np.linalg.norm(pts[a]-pts[b]) <= threshold)

    groups, start = [], 0
    for i in range(1, len(pts)):
        if not adjacent(i-1, i, ids[i]-ids[i-1]):
            groups.append(np.arange(start, i))
            start = i
    groups.append(np.arange(start, len(pts)))
    full_scan = abs(angle_increment)*len(r) >= 2*np.pi-2*abs(angle_increment)
    if (full_scan and len(groups) > 1 and
            adjacent(len(pts)-1, 0, len(r)-ids[-1]+ids[0])):
        groups[0] = np.concatenate((groups.pop(), groups[0]))
    out = []
    for g in groups:
        c = pts[g]
        centre = c.mean(axis=0)
        size = float(2*np.max(np.linalg.norm(c-centre, axis=1)))
        if len(g) >= min_points and min_size <= size <= max_size:
            out.append(Candidate(float(centre[0]), float(centre[1]), size, len(g)))
    return out


class SingleOpponentTracker:
    """One map-frame alpha-beta track, confirmed by hits in distinct scan stamps."""

    def __init__(self, alpha=.6, beta=.06, hold_sec=1., observed_sec=.3,
                 confirm_hits=3, confirm_window=5, max_speed=.6,
                 association_slack=.08, velocity_deadband=.015):
        if not (0 < alpha <= 1 and 0 <= beta <= 1 and hold_sec > 0 and
                observed_sec > 0 and 1 <= confirm_hits <= confirm_window and
                max_speed > 0 and association_slack > 0 and velocity_deadband >= 0):
            raise ValueError('invalid opponent tracking parameters')
        self.alpha, self.beta = alpha, beta
        self.hold, self.observed_sec = hold_sec, observed_sec
        self.hits, self.window = confirm_hits, confirm_window
        self.max_speed, self.slack = max_speed, association_slack
        self.deadband = velocity_deadband
        self.reset()

    def reset(self):
        self.position = self.velocity = self.measured = None
        self.last_seen = self.last_scan = None
        self.pending = []
        self.velocity_valid = False

    def update(self, candidates, stamp):
        """At most one correction per increasing measurement timestamp (seconds)."""
        if not np.isfinite(stamp) or stamp <= 0:
            return False
        if self.last_scan is not None:
            if stamp < self.last_scan:
                return False  # late packets are not a new measurement epoch
            elif stamp == self.last_scan:
                return False
        self.last_scan = stamp
        if self.position is not None and stamp-self.last_seen > self.hold:
            self.position = self.velocity = self.measured = None
            self.velocity_valid = False
            self.pending = []
        if self.position is not None:
            dt = stamp-self.last_seen
            predicted = self.position+self.velocity*dt
            nearby = [c for c in candidates
                      if np.linalg.norm(c.xy-self.measured) <= self.slack+self.max_speed*dt
                      and np.linalg.norm(c.xy-predicted) <= self.slack+self.max_speed*dt]
            if not nearby:
                return False
            chosen = min(nearby, key=lambda c: np.linalg.norm(c.xy-predicted))
            residual = chosen.xy-predicted
            self.position = predicted+self.alpha*residual
            self.velocity += self.beta*residual/max(dt, .001)
            speed = np.linalg.norm(self.velocity)
            if speed > self.max_speed:
                self.velocity *= self.max_speed/speed
            if np.linalg.norm(chosen.xy-self.measured) < self.deadband*dt:
                self.velocity *= .5
            self.measured, self.last_seen = chosen.xy, stamp
            self.velocity_valid = True
            return True

        # Bounded tentative hypotheses, each containing one bool per scan, not per tick.
        used = set()
        surviving = []
        for track in self.pending:
            point, seen, history = track
            if stamp-seen > self.hold:
                continue
            options = [(i, c) for i, c in enumerate(candidates) if i not in used and
                       np.linalg.norm(c.xy-point) <= self.slack+self.max_speed*(stamp-seen)]
            history.append(False)
            if options:
                i, chosen = min(options, key=lambda ic: np.linalg.norm(ic[1].xy-point))
                used.add(i)
                point, seen, history[-1] = chosen.xy, stamp, True
                if sum(history) >= self.hits:
                    self.position, self.measured = point.copy(), point.copy()
                    self.velocity = np.zeros(2)
                    self.last_seen = stamp
                    self.pending = []
                    return True
            if any(history):
                surviving.append((point, seen, history))
        for i, c in enumerate(candidates):
            if i not in used and len(surviving) < 8:
                surviving.append((c.xy, stamp, deque([True], maxlen=self.window)))
        self.pending = surviving
        if self.hits == 1 and self.pending:
            self.position = self.measured = self.pending[0][0].copy()
            self.velocity, self.last_seen = np.zeros(2), stamp
            self.pending = []
            return True
        return False

    def sample(self, now):
        """Read-only prediction; tick frequency never changes the filter state."""
        if self.position is None:
            return None
        age = now-self.last_seen
        if age < 0 or age > self.hold:
            return None
        state = 0 if age <= self.observed_sec and self.last_scan == self.last_seen else 1
        velocity = self.velocity.copy() if self.velocity_valid else np.zeros(2)
        if np.linalg.norm(velocity) < self.deadband:
            velocity.fill(0)
        position = self.position+velocity*age
        return dict(position=position, velocity=velocity, state=state,
                    last_seen=self.last_seen, confidence=max(0., 1-age/self.hold),
                    variance=.01+(.05*age)**2)
