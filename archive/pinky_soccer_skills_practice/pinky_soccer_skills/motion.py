"""
Motion primitives (L1).

Command limiting, pose control with reverse, line following, kinematic rollouts and a
candidate-path planner that keeps clear of the ball.

Everything here is pure Python and deterministic, so the same code is used by the skills,
by their planners (as a fast unicycle model) and by unit tests.
"""

import math

from .geometry import bearing, clamp, dist, offset, Pose, rect_point_distance, unit, wrap


class Limiter:
    """Acceleration ramps and wheel-speed saturation that keep the turning curvature."""

    def __init__(self, robot, dt):
        self.r = robot
        self.dt = dt
        self.v = 0.0
        self.w = 0.0

    def reset(self, v=0.0, w=0.0):
        """Set the current command state."""
        self.v, self.w = v, w

    def saturate(self, v, w):
        """Scale (v, w) together into the speed and wheel limits."""
        r = self.r
        s = 1.0
        if abs(v) > r.max_lin:
            s = min(s, r.max_lin / abs(v))
        if abs(w) > r.max_ang:
            s = min(s, r.max_ang / abs(w))
        wheel = (abs(v) + abs(w) * r.track / 2.0) / r.wheel_radius
        if wheel > r.wheel_max_speed:
            s = min(s, r.wheel_max_speed / wheel)
        return v * s, w * s

    def __call__(self, v, w):
        """Return the limited command and remember it."""
        v, w = self.saturate(v, w)
        dv, dw = self.r.accel_lin * self.dt, self.r.accel_ang * self.dt
        self.v = clamp(v, self.v - dv, self.v + dv)
        self.w = clamp(w, self.w - dw, self.w + dw)
        return self.v, self.w


def pose_law(pose, goal, gains, direction=1):
    """
    Polar pose controller (rho, alpha, beta); direction -1 drives backwards.

    Returns (v, w, rho). The law converges to the goal position with the goal heading;
    the caller scales (v, w) together, which keeps the geometric path.
    """
    th = pose.theta if direction > 0 else wrap(pose.theta + math.pi)
    gth = goal.theta if direction > 0 else wrap(goal.theta + math.pi)
    dx, dy = goal.x - pose.x, goal.y - pose.y
    rho = math.hypot(dx, dy)
    path = math.atan2(dy, dx) if rho > 1e-9 else th
    alpha = wrap(path - th)
    beta = wrap(gth - path)
    v = gains.k_rho * rho
    w = gains.k_alpha * alpha + gains.k_beta * beta
    if abs(alpha) > math.pi / 2:      # goal behind in this direction: turn first
        v = 0.0
    return direction * v, w, rho


def turn_rate(err, robot, gains):
    """In-place turn rate with a stopping profile for the heading error err."""
    w = min(robot.max_ang, math.sqrt(2.0 * gains.turn_decel * abs(err)) + 0.2)
    return math.copysign(w, err)


def time_estimate(pose, goal, robot, direction=1, final_heading=True):
    """Rough rotate-translate-rotate time [s] in the given direction."""
    th = pose.theta if direction > 0 else wrap(pose.theta + math.pi)
    gth = goal.theta if direction > 0 else wrap(goal.theta + math.pi)
    rho = dist(pose.xy, goal.xy)
    if rho < 1e-6:
        return abs(wrap(goal.theta - pose.theta)) / robot.max_ang
    path = bearing(pose.xy, goal.xy)
    turn = abs(wrap(path - th)) + (abs(wrap(gth - path)) if final_heading else 0.0)
    v, a = robot.max_lin, robot.accel_lin
    ramp = v / a
    lin = rho / v + ramp if rho > v * ramp else 2.0 * math.sqrt(rho / a)
    return turn / robot.max_ang + lin


class DriveTo:
    """Drive to a pose (or through waypoints) choosing forward or reverse per segment."""

    def __init__(self, robot, gains, waypoints, pass_tol=0.045):
        self.r = robot
        self.g = gains
        self.set_waypoints(waypoints)
        self.pass_tol = pass_tol

    def set_waypoints(self, waypoints):
        """Waypoints: list of Pose; intermediate headings are ignored (pass-through)."""
        self.wps = list(waypoints)
        self.index = 0
        self.direction = None
        self.turning = False

    @property
    def goal(self):
        """Return the final pose."""
        return self.wps[-1]

    def _target(self, pose):
        while self.index < len(self.wps) - 1:
            if dist(pose.xy, self.wps[self.index].xy) < self.pass_tol:
                self.index += 1
                self.direction = None
            else:
                break
        target = self.wps[self.index]
        last = self.index == len(self.wps) - 1
        if not last:
            # Pass-through: aim the heading at the next waypoint.
            nxt = self.wps[self.index + 1]
            target = Pose(target.x, target.y, bearing(target.xy, nxt.xy))
        return target, last

    def _choose(self, pose, target, last):
        if not self.g.reverse:
            self.direction = 1
            return
        fwd = time_estimate(pose, target, self.r, 1, last)
        rev = time_estimate(pose, target, self.r, -1, last)
        if self.direction is None:
            self.direction = 1 if fwd <= rev else -1
        elif self.direction > 0 and rev + self.g.reverse_hysteresis < fwd:
            self.direction = -1
        elif self.direction < 0 and fwd + self.g.reverse_hysteresis < rev:
            self.direction = 1

    def command(self, pose):
        """Return (v, w, done)."""
        target, last = self._target(pose)
        rho = dist(pose.xy, target.xy)
        if last and (rho < self.g.pos_tol or (self.turning and rho < 2.0 * self.g.pos_tol)):
            self.turning = True
            err = wrap(target.theta - pose.theta)
            if abs(err) < self.g.heading_tol:
                return 0.0, 0.0, True
            return 0.0, turn_rate(err, self.r, self.g), False
        self.turning = False
        self._choose(pose, target, last)
        v, w, _ = pose_law(pose, target, self.g, self.direction)
        if abs(v) < 1e-9:
            return 0.0, w, False
        # Path shape (curvature) from the law, speed from a stopping-distance profile.
        kappa = w / v
        speed = self.r.max_lin
        if last:
            speed = min(speed, math.sqrt(2.0 * self.g.decel * max(rho - 0.5 * self.g.pos_tol,
                                                                  0.0)) + 0.03)
        v = math.copysign(speed, v)
        return v, kappa * v, False


def line_follow(pose, point, heading, speed, k_stanley, k_heading, max_ang):
    """Stanley steering onto the line through point with direction heading. Return (v, w)."""
    c, s = math.cos(heading), math.sin(heading)
    lateral = -s * (pose.x - point[0]) + c * (pose.y - point[1])   # left of the line: +
    err = wrap(heading - pose.theta) + math.atan2(-k_stanley * lateral,
                                                  max(abs(speed), 0.05))
    return speed, clamp(k_heading * err, -max_ang, max_ang)


def ball_clearance(pose, ball_xy, robot):
    """Gap [m] between the ball surface and the robot footprint (negative = overlap)."""
    lx, ly = pose.to_local(*ball_xy)
    x0, x1, y0, y1 = robot.footprint
    return rect_point_distance(x0, x1, y0, y1, lx, ly) - robot.ball_radius


def side_clearance(pose, ball_xy, robot):
    """
    Clearance that only counts sides, corners and back.

    A ball straight in front of the bumper face (inside its width) counts as clear unless
    it would be pushed; touching it with the face while arriving from behind is fine.
    """
    lx, ly = pose.to_local(*ball_xy)
    x0, x1, y0, y1 = robot.footprint
    if lx > x1 and abs(ly) < robot.bumper_half_width - 0.004:
        return 1.0 if lx - x1 - robot.ball_radius > -0.003 else -1.0
    return rect_point_distance(x0, x1, y0, y1, lx, ly) - robot.ball_radius


def rollout(pose, waypoints, robot, gains, ball_xy=None, dt=0.05, t_max=12.0, v0=0.0,
            w0=0.0, obstacles=()):
    """Simulate DriveTo on a unicycle. Return dict(time, ok, clearance, path)."""
    ctl = DriveTo(robot, gains, waypoints)
    lim = Limiter(robot, dt)
    lim.reset(v0, w0)
    p = Pose(pose.x, pose.y, pose.theta)
    t, clearance, path = 0.0, math.inf, [(p.x, p.y, p.theta)]
    while t < t_max:
        v, w, done = ctl.command(p)
        if done:
            return {'time': t, 'ok': True, 'clearance': clearance, 'path': path}
        v, w = lim(v, w)
        p = Pose(p.x + v * math.cos(p.theta + 0.5 * w * dt) * dt,
                 p.y + v * math.sin(p.theta + 0.5 * w * dt) * dt, wrap(p.theta + w * dt))
        t += dt
        if ball_xy is not None:
            clearance = min(clearance, side_clearance(p, ball_xy, robot))
        for ox, oy, rad in obstacles:
            clearance = min(clearance, math.hypot(p.x - ox, p.y - oy) - rad)
        path.append((p.x, p.y, p.theta))
    return {'time': t, 'ok': False, 'clearance': clearance, 'path': path}


def candidate_paths(pose, target, ball_xy, robot, keep_out, obstacles=()):
    """
    Waypoint chains to reach target (a pose facing the ball) around the ball.

    Candidates: direct; around either side (beside, behind-side); reverse out first.
    """
    h = target.theta
    n = (-math.sin(h), math.cos(h))
    u = unit(h)
    out = {'direct': [target]}
    for side, name in ((1, 'left'), (-1, 'right')):
        beside = (ball_xy[0] + side * keep_out * n[0], ball_xy[1] + side * keep_out * n[1])
        behind = (beside[0] - keep_out * u[0], beside[1] - keep_out * u[1])
        front = (beside[0] + keep_out * u[0], beside[1] + keep_out * u[1])
        out[f'{name}_behind'] = [Pose(*behind), target]
        out[f'{name}_beside'] = [Pose(*beside), Pose(*behind), target]
        out[f'{name}_around'] = [Pose(*front), Pose(*beside), Pose(*behind), target]
    for back in (0.08, 0.15):
        p = offset(pose.xy, pose.theta, -back)
        out[f'back_{int(back * 100)}'] = [Pose(*p), target]
        # Pre-approach point on the target line: reach it (often in reverse, turning on
        # the way) and then drive straight into the target.
        q = offset(target.xy, h, -back)
        out[f'line_{int(back * 100)}'] = [Pose(q[0], q[1], h), target]
    # Detours around opponents (robot-centre clearance circles).
    if obstacles:
        from .planning import avoid_path
        circles = [(ox, oy) for ox, oy, _ in obstacles]
        rad = max(r for _, _, r in obstacles) + 0.03
        for name, wps in list(out.items()):
            first = wps[0]
            via = avoid_path(pose.xy, first.xy, circles, rad)[1:-1]
            if via:
                out[f'{name}_avoid'] = [Pose(*v) for v in via] + wps
    return out


def plan_around_ball(pose, target, ball_xy, robot, gains, min_clearance, keep_out,
                     v0=0.0, w0=0.0, t_max=12.0, obstacles=()):
    """Choose the fastest collision-free candidate. Return (name, waypoints, result)."""
    best = None
    fallback = None
    for name, wps in candidate_paths(pose, target, ball_xy, robot, keep_out,
                                     obstacles).items():
        res = rollout(pose, wps, robot, gains, ball_xy, t_max=t_max, v0=v0, w0=w0,
                      obstacles=obstacles)
        cost = res['time'] + (0.0 if res['ok'] else 100.0)
        if res['clearance'] >= min_clearance:
            if best is None or cost < best[2]['cost']:
                res['cost'] = cost
                best = (name, wps, res)
        res['cost'] = cost - 10.0 * min(res['clearance'], 0.0) * 100.0
        if fallback is None or res['cost'] < fallback[2]['cost']:
            fallback = (name, wps, res)
    return best or fallback
