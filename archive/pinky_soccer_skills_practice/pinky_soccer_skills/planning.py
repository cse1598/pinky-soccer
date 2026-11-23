"""Path planning for composite skills: dribble paths that go around opponents (CurveDribble)."""

import math

from .geometry import bearing, clamp, dist, segment_point_distance


def avoid_path(start, goal, obstacles, radius, field=None, margin=0.12):
    """
    Polyline from start to goal keeping `radius` from every obstacle centre.

    One detour waypoint per blocking obstacle, on the side with the shorter path that stays
    inside the field. Returns a list of points (start and goal included).
    """
    path = [tuple(start), tuple(goal)]
    for _ in range(3):                                   # at most a few detours
        blocked = None
        for i, (a, b) in enumerate(zip(path, path[1:])):
            for o in obstacles:
                d, t = segment_point_distance(a, b, o)
                if d < radius and 0.0 < t < 1.0:
                    blocked = (i, o)
                    break
            if blocked:
                break
        if blocked is None:
            return path
        i, o = blocked
        a, b = path[i], path[i + 1]
        h = bearing(a, b)
        best = None
        for side in (1.0, -1.0):
            w = (o[0] - side * math.sin(h) * radius * 1.25,
                 o[1] + side * math.cos(h) * radius * 1.25)
            if field is not None:
                w = (clamp(w[0], -field.wall_x + margin, field.wall_x - margin),
                     clamp(w[1], -field.wall_y + margin, field.wall_y - margin))
                if dist(w, o) < radius:                  # squeezed against a wall
                    continue
            length = dist(a, w) + dist(w, b)
            if best is None or length < best[0]:
                best = (length, w)
        if best is None:
            return path
        path.insert(i + 1, best[1])
    return path


def shot_aim(ball, field, inset):
    """Aim point inside the opponent goal mouth, biased towards the ball's side."""
    gx, _ = field.opp_goal
    half = field.goal_width / 2.0 - inset
    return (gx + math.copysign(0.03, gx), clamp(0.5 * ball[1], -half, half))
