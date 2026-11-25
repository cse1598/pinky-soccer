"""Frozen obstacle set for one GoToPose solve, from the enabled avoidance options.

A snapshot is taken when a solve is submitted and is re-taken before a scheduled command
is activated (the activation re-check). Obstacles are static within one snapshot.
"""
from dataclasses import dataclass, field as dc_field
import math

import numpy as np

from . import collision_geometry as cg

LOST = 2


@dataclass(frozen=True)
class AvoidanceOptions:
    """GoToPose options (BehaviorGoal avoid_*), fixed per goal_id."""

    walls: bool = False
    opponents: bool = False
    ball: bool = False

    @property
    def any(self):
        return self.walls or self.opponents or self.ball

    def as_dict(self):
        return dict(walls=self.walls, opponents=self.opponents, ball=self.ball)


@dataclass(frozen=True)
class AvoidanceConfig:
    """Clearances [m]. `margin` is required by the exact replay check; the NMPC discs use
    margin + solver_margin (sub-sample sweep and linearization allowance)."""

    margin: float = 0.02
    solver_margin: float = 0.005
    opponent_radius: float = 0.10    # Pinky circumscribed radius 0.097 m, cluster centre
    ball_radius: float = 0.02135     # measured golf ball, 42.7 mm
    sigma_scale: float = 2.0         # inflate by sigma_scale x position sigma ...
    sigma_cap: float = 0.10          # ... at most this much
    max_opponents: int = 2

    def __post_init__(self):
        for k in ('margin', 'solver_margin', 'opponent_radius', 'ball_radius', 'sigma_scale',
                  'sigma_cap'):
            v = getattr(self, k)
            if not (math.isfinite(v) and v >= 0):
                raise ValueError(f'{k} must be finite and nonnegative')
        if self.max_opponents < 0:
            raise ValueError('max_opponents must be nonnegative')


@dataclass
class Circle:
    """Round obstacle: centre, effective radius (object + uncertainty, no margin)."""

    x: float
    y: float
    radius: float
    kind: str


@dataclass
class ObstacleSnapshot:
    options: AvoidanceOptions
    config: AvoidanceConfig
    walls: object = None              # cg.FieldObstacles when walls are enabled
    circles: list = dc_field(default_factory=list)
    stamp: float = 0.0

    def clearances(self, poses):
        """Signed clearance [m] per obstacle (list over 'walls' and circles) for each pose.

        Returns (names, matrix len(poses) x len(names)). Vectorized for the replay check.
        """
        poses = np.asarray(poses, float).reshape(-1, 3)
        names, cols = [], []
        if self.walls is not None:
            names.append('walls')
            cols.append(wall_clearance(poses, self.walls))
        for i, c in enumerate(self.circles):
            names.append(f'{c.kind}{i}')
            cols.append(circle_clearance(poses, (c.x, c.y)) - c.radius)
        if not cols:
            return names, np.zeros((len(poses), 0))
        return names, np.column_stack(cols)

    def describe(self):
        """Diagnostics and verifier map drawing."""
        return dict(options=self.options.as_dict(), margin=self.config.margin,
                    walls=self.walls.as_dict() if self.walls is not None else None,
                    circles=[dict(x=c.x, y=c.y, r=c.radius, kind=c.kind) for c in self.circles])


def wall_clearance(poses, walls):
    """Vectorized exact clearance of the footprint to the walls and goal blocks."""
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    out = np.full(len(poses), np.inf)
    for rect in cg.FOOTPRINT_RECTS:
        corners = cg.rect_corners(rect)
        gx = poses[:, :1] + c * corners[:, 0] - s * corners[:, 1]
        gy = poses[:, 1:2] + s * corners[:, 0] + c * corners[:, 1]
        out = np.minimum(out, np.minimum(np.min(walls.x_limit - np.abs(gx), axis=1),
                                         np.min(walls.y_limit - np.abs(gy), axis=1)))
        poly = np.stack([gx, gy], axis=2)                   # m x 4 x 2
        for block in walls.blocks:
            out = np.minimum(out, block_distance(poly, block))
    return out


def block_distance(poly, block, near=0.15):
    """Exact distance to a goal block within `near`; beyond it a lower bound (>= near)."""
    lo, hi = block.min(axis=0), block.max(axis=0)
    gap = np.maximum(np.maximum(lo - poly, poly - hi), 0.0)    # per vertex, m x 4 x 2
    bound = np.min(np.linalg.norm(gap, axis=-1), axis=1)      # vertex distance >= distance
    lower = np.maximum(bound - 0.12, 0.0)                      # quad diagonal < 0.12 m
    out = lower.copy()
    idx = np.flatnonzero(lower < near)
    if len(idx):
        out[idx] = convex_distance(poly[idx], block)
    return out


def segment_distances(points, a, b):
    """Distances from points (... x 2) to segments a->b (broadcast, ... x 2)."""
    ab = b - a
    t = np.clip(np.sum((points - a) * ab, axis=-1) / np.maximum(np.sum(ab * ab, axis=-1),
                                                                  1e-18), 0.0, 1.0)
    return np.linalg.norm(points - (a + t[..., None] * ab), axis=-1)


def convex_distance(polys, block):
    """Signed distance between m convex quads (m x 4 x 2, CCW) and one convex quad (4 x 2).

    Separated: min vertex-to-edge distance (exact for convex polygons).
    Overlapping: minus the smallest separating-axis overlap.
    """
    m = polys.shape[0]
    blk = np.broadcast_to(block, (m, 4, 2))
    gaps = []
    for poly in (polys, blk):
        e = np.roll(poly, -1, axis=1) - poly
        n = np.stack([-e[..., 1], e[..., 0]], axis=-1)
        n = n / np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-18)
        pa = np.einsum('mjk,mik->mij', polys, n)            # m x axis x vertex
        pb = np.einsum('mjk,mik->mij', blk, n)
        gaps.append(np.maximum(pb.min(-1) - pa.max(-1), pa.min(-1) - pb.max(-1)))
    gap = np.concatenate(gaps, axis=1)                      # m x 8
    separated = np.max(gap, axis=1) >= 0
    best = np.full(m, np.inf)
    for p_set, q_set in ((polys, blk), (blk, polys)):
        q0, q1 = q_set, np.roll(q_set, -1, axis=1)
        for v in range(4):
            d = segment_distances(p_set[:, v:v + 1, :], q0, q1)   # m x 4
            best = np.minimum(best, d.min(axis=1))
    # Overlap: penetration along the best separating-axis candidate.
    return np.where(separated, best, np.max(gap, axis=1))


def circle_clearance(poses, centre):
    """Vectorized signed distance from a point to the exact footprint (negative inside)."""
    dx, dy = centre[0] - poses[:, 0], centre[1] - poses[:, 1]
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    best = np.full(len(poses), np.inf)
    for x0, y0, x1, y1 in cg.FOOTPRINT_RECTS:
        ox = np.maximum.reduce([x0 - lx, np.zeros_like(lx), lx - x1])
        oy = np.maximum.reduce([y0 - ly, np.zeros_like(ly), ly - y1])
        outside = np.hypot(ox, oy)
        inside = -np.minimum.reduce([lx - x0, x1 - lx, ly - y0, y1 - ly])
        best = np.minimum(best, np.where((ox > 0) | (oy > 0), outside, inside))
    return best


def sigma_of(obj):
    """Largest position standard deviation of a TrackedObject [m]."""
    cov = obj.covariance
    return math.sqrt(max(cov[0], cov[4], 0.0))


def take(options, config, field_obstacles, world=None, stamp=0.0):
    """Build a snapshot from the enabled options and the latest WorldModel (may be None)."""
    snap = ObstacleSnapshot(options, config, field_obstacles if options.walls else None,
                            stamp=stamp)
    if world is None:
        return snap

    def usable(obj):
        p = obj.pose
        return obj.state != LOST and all(math.isfinite(v) for v in (p.x, p.y))

    def inflate(obj):
        return min(config.sigma_cap, config.sigma_scale * sigma_of(obj))

    if options.opponents:
        tracked = [o for o in world.opponents if usable(o)]
        me = world.self.pose
        tracked.sort(key=lambda o: math.hypot(o.pose.x - me.x, o.pose.y - me.y))
        for o in tracked[:config.max_opponents]:
            snap.circles.append(Circle(o.pose.x, o.pose.y,
                                       config.opponent_radius + inflate(o), 'opponent'))
    if options.ball and usable(world.ball):
        b = world.ball
        snap.circles.append(Circle(b.pose.x, b.pose.y, config.ball_radius + inflate(b), 'ball'))
    return snap
