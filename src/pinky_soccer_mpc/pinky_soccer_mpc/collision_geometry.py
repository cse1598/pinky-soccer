"""Footprint and obstacle geometry for the optional GoToPose collision constraints.

Two representations of the same robot:
  * exact footprint: union of robot-frame rectangles, used by the independent replay check
  * disc cover: discs that contain the exact footprint, used inside the NMPC (smooth)
The disc cover is conservative; acceptance is decided by the exact footprint at sub-samples
between NMPC nodes, like the existing octagon terminal set vs the exact circle replay.

Units: m, rad, map frame. The pose is base_footprint (wheel axle centre, MPC pose).
"""
import math

import numpy as np

# Robot-frame collision boxes [xmin, ymin, xmax, ymax] of the Pinky Pro + bumper v1, from the
# MuJoCo collision geoms (2026-11-25, scene cache of pinky_soccer_sim): chassis mesh
# x -0.0745..0.0387 (y made symmetric to the wider +0.0473 side), caster x -0.076, bumper
# front face x 0.0415, screen mount x 0.0421, wheels x +-0.028, y +-(0.0206..0.0766).
FOOTPRINT_RECTS = ((-0.076, -0.0473, 0.0421, 0.0473),
                   (-0.028, 0.0206, 0.028, 0.0766),
                   (-0.028, -0.0766, 0.028, -0.0206))
# Two discs covering FOOTPRINT_RECTS (covering radius 0.0681 m, checked by tests).
FOOTPRINT_DISCS = ((-0.017, 0.034, 0.069), (-0.017, -0.034, 0.069))


def rect_corners(rect):
    """Corners of [xmin, ymin, xmax, ymax], counter-clockwise."""
    x0, y0, x1, y1 = rect
    return np.array(((x0, y0), (x1, y0), (x1, y1), (x0, y1)), float)


def transform(points, pose):
    """Robot-frame points to the map frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    p = np.asarray(points, float)
    return np.column_stack((pose[0] + c * p[:, 0] - s * p[:, 1],
                            pose[1] + s * p[:, 0] + c * p[:, 1]))


def footprint_polygons(pose, rects=FOOTPRINT_RECTS):
    """Map-frame convex polygons (4x2 arrays) of the exact footprint."""
    return [transform(rect_corners(r), pose) for r in rects]


def disc_centres(pose, discs=FOOTPRINT_DISCS):
    """Map-frame disc centres (n x 2) and radii (n)."""
    d = np.asarray(discs, float)
    return transform(d[:, :2], pose), d[:, 2]


def segment_distance(p, a, b):
    """Distance from point p to segment ab."""
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-18), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def point_polygon_distance(p, poly):
    """Signed distance from p to a convex counter-clockwise polygon (negative inside)."""
    edges = [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]
    d = min(segment_distance(p, a, b) for a, b in edges)
    inside = all((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0
                 for a, b in edges)
    return -d if inside else d


def polygon_distance(a, b):
    """Distance between two convex polygons; negative (minus the SAT overlap) if they overlap."""
    overlap = math.inf
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array((-e[1], e[0])) / max(np.linalg.norm(e), 1e-18)
            pa, pb = a @ n, b @ n
            gap = max(pb.min() - pa.max(), pa.min() - pb.max())
            if gap >= 0:
                break
            overlap = min(overlap, -gap)
        else:
            continue
        # Separated: exact distance is the closest vertex-to-edge pair.
        return min(min(segment_distance(p, q[i], q[(i + 1) % len(q)])
                       for i in range(len(q)) for p in r)
                   for r, q in ((a, b), (b, a)))
    return -overlap


class FieldObstacles:
    """Walls and goal structures of a pinky_soccer_field Field, for the robot body.

    The goal mouth roof (goal_height 70 mm) is below the chassis top, so each goal is one
    solid block from the goal line to the end wall (conservative, as for the NMPC).
    """

    def __init__(self, field):
        walls = field.inner_walls
        self.x_limit = float(walls.xmax)
        self.y_limit = float(walls.ymax)
        self.goal_front = float(field.half_length)
        self.goal_half_width = float(field.spec['goal_width'] / 2 +
                                     field.spec.get('goal_thickness',
                                                    field.spec['wall_thickness']))
        self.blocks = []
        for s in (-1, 1):
            x0, x1 = sorted((s * self.goal_front, s * self.x_limit))
            self.blocks.append(np.array(((x0, -self.goal_half_width), (x1, -self.goal_half_width),
                                         (x1, self.goal_half_width), (x0, self.goal_half_width))))

    def clearance(self, polygons):
        """Smallest distance of the footprint polygons to the walls and goal blocks."""
        out = math.inf
        for poly in polygons:
            out = min(out, float(np.min(self.x_limit - np.abs(poly[:, 0]))),
                      float(np.min(self.y_limit - np.abs(poly[:, 1]))))
            for block in self.blocks:
                out = min(out, polygon_distance(poly, block))
        return out

    def as_dict(self):
        """Serializable description (diagnostics, verifier map)."""
        return dict(x_limit=self.x_limit, y_limit=self.y_limit, goal_front=self.goal_front,
                    goal_half_width=self.goal_half_width)


def circle_clearance(polygons, centre, radius):
    """Smallest distance of the footprint polygons to a circle (negative if overlapping)."""
    c = np.asarray(centre, float)
    return min(point_polygon_distance(c, poly) for poly in polygons) - float(radius)


def interpolate_poses(xs, samples=5):
    """Poses at `samples` sub-steps of every NMPC interval (straight-line, shortest yaw).

    Between 0.1 s nodes the robot moves at most about 2.5 cm; the sub-samples bound the
    footprint sweep used by the replay check (the exact 5 ms motor substeps are not stored).
    """
    xs = np.asarray(xs, float)[:, :3]
    out = [xs[0]]
    for a, b in zip(xs, xs[1:]):
        dth = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        for k in range(1, samples + 1):
            t = k / samples
            out.append(np.array((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                                 a[2] + t * dth)))
    return np.asarray(out)
