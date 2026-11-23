"""
Ball prediction queries (L0). Pure functions, no motion.

Rolling model: constant rolling deceleration (as in pinky_soccer_sim materials.yaml
ball_floor.rolling_deceleration_m_s2, an EVA-mat assumption until measured). Note that
pinky_soccer_world_model.ball_filter predicts with an exponential decay instead; skills
re-predict from the filter's position/velocity with this model. Walls stop the ball.
"""

import math

from .geometry import clamp

DEFAULT_DECEL = 0.35   # [m/s^2]


def travel(speed, t, decel=DEFAULT_DECEL):
    """Distance rolled after t seconds from `speed`."""
    if decel <= 0:
        return speed * t
    t_stop = speed / decel
    t = min(t, t_stop)
    return speed * t - 0.5 * decel * t * t


def predict(ball, t, decel=DEFAULT_DECEL, field=None, radius=0.02135):
    """Return predicted (x, y) of the ball after t seconds."""
    if t <= 0.0:
        return (ball.x, ball.y)
    speed = math.hypot(ball.vx, ball.vy)
    if speed < 1e-6:
        return (ball.x, ball.y)
    d = travel(speed, t, decel)
    x, y = ball.x + ball.vx / speed * d, ball.y + ball.vy / speed * d
    if field is not None:
        x = clamp(x, -field.wall_x + radius, field.wall_x - radius)
        y = clamp(y, -field.wall_y + radius, field.wall_y - radius)
    return (x, y)


def rest_point(ball, decel=DEFAULT_DECEL, field=None, radius=0.02135):
    """Where the ball stops if nothing touches it."""
    return predict(ball, 1e3, decel, field, radius)


def reach(speed, decel=DEFAULT_DECEL):
    """Total rolling distance from `speed`."""
    return speed * speed / (2.0 * decel) if decel > 0 else math.inf


def intercept(ball, eta, horizon=3.0, step=0.05, decel=DEFAULT_DECEL, field=None):
    """
    Earliest (t, point) where the robot can be at the predicted ball.

    eta(point) returns the robot's time to reach point. Falls back to the point at the
    horizon when the ball is never reachable in time.
    """
    t = 0.0
    while t <= horizon:
        p = predict(ball, t, decel, field)
        if eta(p) <= t:
            return t, p
        t += step
    return horizon, predict(ball, horizon, decel, field)


def crossing_x(ball, x_line, horizon=3.0, decel=DEFAULT_DECEL):
    """Return (t, y) when the predicted ball crosses x = x_line, or None."""
    speed = math.hypot(ball.vx, ball.vy)
    if abs(ball.vx) < 1e-3 or speed < 1e-6:
        return None
    need = x_line - ball.x
    if need / ball.vx < 0:
        return None
    d = need * speed / ball.vx                   # distance along the path
    if d > reach(speed, decel):
        return None
    # speed*t - decel*t^2/2 = d
    disc = speed * speed - 2.0 * decel * d
    t = (speed - math.sqrt(max(disc, 0.0))) / decel if decel > 0 else d / speed
    if t > horizon:
        return None
    return t, ball.y + ball.vy / speed * d
