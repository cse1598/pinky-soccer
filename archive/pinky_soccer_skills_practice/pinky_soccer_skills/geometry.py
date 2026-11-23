"""Planar geometry helpers shared by skills, planners and the test bench (no ROS)."""

from dataclasses import dataclass
import math


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    """Clamp v to [lo, hi]."""
    return max(lo, min(hi, v))


@dataclass
class Pose:
    """Planar pose: x, y [m] and heading theta [rad]."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    @property
    def xy(self):
        """Return (x, y)."""
        return (self.x, self.y)

    def to_local(self, px, py):
        """Express a world point in this pose's frame (x forward, y left)."""
        dx, dy = px - self.x, py - self.y
        c, s = math.cos(self.theta), math.sin(self.theta)
        return (c * dx + s * dy, -s * dx + c * dy)

    def to_world(self, lx, ly):
        """Express a point given in this pose's frame in the world frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return (self.x + c * lx - s * ly, self.y + s * lx + c * ly)


def dist(a, b):
    """Euclidean distance between two (x, y) points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bearing(a, b):
    """Direction from point a to point b [rad]."""
    return math.atan2(b[1] - a[1], b[0] - a[0])


def unit(angle):
    """Return the unit vector of an angle."""
    return (math.cos(angle), math.sin(angle))


def offset(p, angle, d):
    """Point p moved d along direction angle."""
    return (p[0] + d * math.cos(angle), p[1] + d * math.sin(angle))


def segment_point_distance(a, b, p):
    """Return (distance, t) from p to segment a-b, t in [0, 1] along the segment."""
    ax, ay = b[0] - a[0], b[1] - a[1]
    n2 = ax * ax + ay * ay
    t = 0.0 if n2 < 1e-12 else clamp(((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / n2, 0.0, 1.0)
    q = (a[0] + t * ax, a[1] + t * ay)
    return dist(q, p), t


def rect_point_distance(xmin, xmax, ymin, ymax, px, py):
    """Distance from a point to an axis-aligned rectangle (0 inside)."""
    dx = max(xmin - px, 0.0, px - xmax)
    dy = max(ymin - py, 0.0, py - ymax)
    return math.hypot(dx, dy)


class Polyline:
    """Piecewise-linear path with arc-length queries."""

    def __init__(self, points):
        pts = [tuple(map(float, p)) for p in points]
        dedup = [pts[0]]
        for p in pts[1:]:
            if dist(p, dedup[-1]) > 1e-6:
                dedup.append(p)
        if len(dedup) < 2:
            raise ValueError('a path needs two distinct points')
        self.points = dedup
        self.cum = [0.0]
        for a, b in zip(dedup, dedup[1:]):
            self.cum.append(self.cum[-1] + dist(a, b))

    @property
    def length(self):
        """Total arc length [m]."""
        return self.cum[-1]

    @property
    def end(self):
        """Last point."""
        return self.points[-1]

    def project(self, p):
        """Return (s, lateral, heading): arc length, signed offset (left +), tangent."""
        best = None
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            d, t = segment_point_distance(a, b, p)
            if best is None or d < best[0] - 1e-12:
                best = (d, i, t)
        d, i, t = best
        a, b = self.points[i], self.points[i + 1]
        heading = bearing(a, b)
        # Signed: positive when p is left of the travel direction.
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        s = self.cum[i] + t * (self.cum[i + 1] - self.cum[i])
        return s, math.copysign(d, cross) if d > 1e-12 else 0.0, heading

    def point_at(self, s):
        """Point at arc length s (clamped)."""
        s = clamp(s, 0.0, self.length)
        for i in range(len(self.points) - 1):
            if s <= self.cum[i + 1] or i == len(self.points) - 2:
                seg = self.cum[i + 1] - self.cum[i]
                t = 0.0 if seg < 1e-12 else (s - self.cum[i]) / seg
                a, b = self.points[i], self.points[i + 1]
                return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        return self.end

    def heading_at(self, s):
        """Tangent direction at arc length s."""
        s = clamp(s, 0.0, self.length)
        for i in range(len(self.points) - 1):
            if s <= self.cum[i + 1] or i == len(self.points) - 2:
                return bearing(self.points[i], self.points[i + 1])
        return bearing(self.points[-2], self.points[-1])


def round_corners(points, radius, step=0.03):
    """Replace polyline corners by circular arcs of at most `radius` (fillets)."""
    pts = [tuple(p) for p in points]
    if len(pts) < 3 or radius <= 0:
        return pts
    out = [pts[0]]
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        h1, h2 = bearing(a, b), bearing(b, c)
        turn = wrap(h2 - h1)
        if abs(turn) < 1e-3:
            out.append(b)
            continue
        # Tangent length limited by half of each adjacent segment.
        t = min(radius * math.tan(abs(turn) / 2.0), 0.5 * dist(a, b), 0.5 * dist(b, c))
        r = t / math.tan(abs(turn) / 2.0)
        p1 = (b[0] - t * math.cos(h1), b[1] - t * math.sin(h1))
        side = 1.0 if turn > 0 else -1.0
        centre = (p1[0] - side * r * math.sin(h1), p1[1] + side * r * math.cos(h1))
        n = max(2, int(abs(turn) * r / step) + 1)
        start = bearing(centre, p1)
        for k in range(n + 1):
            ang = start + side * abs(turn) * k / n
            out.append((centre[0] + r * math.cos(ang), centre[1] + r * math.sin(ang)))
    out.append(pts[-1])
    return out
