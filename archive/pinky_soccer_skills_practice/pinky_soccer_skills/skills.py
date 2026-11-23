"""
Ball skills (L2).

Each skill is a small state machine: construct it with its inputs, call step(ctx) every
control cycle, and read SkillOutput (v, w, status). Skills never read sensors directly;
they only see SkillContext. Outputs are raw requests; the caller applies motion.Limiter.
See SKILLS.md for the behaviour of every skill, its inputs and its success/failure rules.
"""

import math

from . import prediction
from .context import FAILURE, LOST, OBSERVED, RUNNING, SkillOutput, SUCCESS
from .geometry import bearing, clamp, dist, offset, Polyline, Pose, round_corners, unit, wrap
from .motion import (ball_clearance, DriveTo, line_follow, plan_around_ball, time_estimate,
                     turn_rate)


# ----------------------------------------------------------------------------- helpers
class LocalBall:
    """
    Ball estimate that survives the camera blind zone in front of the bumper.

    The camera cannot see a ball closer than about 4-5 cm to the bumper. Between camera
    fixes the ball is predicted with a small contact model: a free ball keeps rolling with
    constant rolling deceleration; when the bumper catches up with it the ball is pushed
    at the robot's speed. An ultrasonic range (minus its reading at contact) is blended in
    as a weak gap measurement when it is plausible.
    """

    def __init__(self, P):
        self.r = P.robot
        self.c = P.ball_model
        self.pos = None             # map (x, y)
        self.vel = (0.0, 0.0)       # map velocity
        self.xy = None              # local (x, y)
        self.held = False
        self.unseen = 0.0
        self.missing = 0.0

    def gap(self, xy=None):
        """Distance between ball surface and bumper face along x (negative = overlap)."""
        xy = xy or self.xy
        return None if xy is None else xy[0] - self.r.contact_dist

    def us_near(self, ctx):
        """Ultrasonic says something is within a few centimetres."""
        return ctx.us_range == -math.inf or (math.isfinite(ctx.us_range) and
                                             ctx.us_range < self.c.us_contact_range + 0.03)

    def update(self, ctx, dt):
        """Refresh from the context; return the local (x, y) or None."""
        b = ctx.ball
        if b is None:
            self.pos = self.xy = None
            return None
        if b.seen_now or self.pos is None:
            self.pos, self.vel, self.unseen = b.xy, (b.vx, b.vy), 0.0
            if not b.seen_now:
                self.unseen = b.age
        else:
            self.unseen += dt
            self._predict(dt)
        self.xy = ctx.pose.to_local(*self.pos)
        gap = self.gap()
        # Driving forward into the predicted ball means we are pushing it.
        if gap < 0.0 and ctx.v > 0.02 and not b.seen_now and \
                abs(self.xy[1]) < self.r.bumper_half_width + self.r.ball_radius:
            self.xy = (self.r.contact_dist, self.xy[1])
            self.pos = ctx.pose.to_world(*self.xy)
            c, s = math.cos(ctx.pose.theta), math.sin(ctx.pose.theta)
            along = self.vel[0] * c + self.vel[1] * s
            if ctx.v > along:
                self.vel = (ctx.v * c, ctx.v * s)
        if not b.seen_now and self.c.us_weight > 0.0 and math.isfinite(ctx.us_range) \
                and abs(self.xy[1]) < self.r.bumper_half_width:
            us_gap = ctx.us_range - self.c.us_contact_range
            if -0.01 < us_gap < 0.08:
                g = (1 - self.c.us_weight) * self.gap() + self.c.us_weight * max(0.0, us_gap)
                self.xy = (self.r.contact_dist + g, self.xy[1])
                self.pos = ctx.pose.to_world(*self.xy)
        self.held = self.gap() < 0.004
        # Time the ball was predicted well inside the camera view but not detected.
        should_see = self.gap() > self.c.visible_gap and abs(self.xy[1]) < 0.4 * self.xy[0]
        self.missing = self.missing + dt if (should_see and not b.seen_now) else 0.0
        return self.xy

    def lost(self, ctx, blind_time):
        """Ball missing where it should be visible for a while, or unseen for long."""
        if self.xy is None:
            return True
        return self.missing > self.c.missing_time or \
            (self.unseen > blind_time and not self.us_near(ctx))

    def _predict(self, dt):
        vx, vy = self.vel
        speed = math.hypot(vx, vy)
        if speed > 1e-6:
            new = max(0.0, speed - self.c.roll_decel * dt)
            travel = 0.5 * (speed + new) * dt
            self.pos = (self.pos[0] + vx / speed * travel, self.pos[1] + vy / speed * travel)
            self.vel = (vx / speed * new, vy / speed * new)

    def map_xy(self, pose):
        """Ball in the map frame."""
        return self.pos


class FixCounter:
    """Counts camera fixes; resets when no fix arrived for `window` seconds."""

    def __init__(self, window=0.25):
        self.window = window
        self.count = 0
        self.last = None

    def update(self, ctx):
        """Return the number of recent fixes."""
        b = ctx.ball
        if b is not None and b.seen_now:
            self.count = self.count + 1 if (self.last is not None and
                                            ctx.t - self.last <= self.window) else 1
            self.last = ctx.t
        elif self.last is not None and ctx.t - self.last > self.window:
            self.count = 0
        return self.count


def keep_inside(p, field, margin):
    """Clamp a point inside the inner walls."""
    if field is None:
        return p
    return (clamp(p[0], -field.wall_x + margin, field.wall_x - margin),
            clamp(p[1], -field.wall_y + margin, field.wall_y - margin))


class Skill:
    """Base class with a clock for timeouts."""

    name = 'Skill'

    def __init__(self, P):
        self.P = P
        self.t0 = None
        self.prev_t = None
        self.phase = ''

    def elapsed(self, ctx):
        """Seconds since the first step."""
        if self.t0 is None:
            self.t0 = ctx.t
        return ctx.t - self.t0

    def dt(self, ctx):
        """Seconds since the previous step."""
        d = 0.0 if self.prev_t is None else ctx.t - self.prev_t
        self.prev_t = ctx.t
        return d

    def out(self, v, w, **info):
        """Return a RUNNING output tagged with the current phase."""
        return SkillOutput(v, w, RUNNING, phase=self.phase, info=info)


# ----------------------------------------------------------------------------- L1 wrappers
class GoToPose(Skill):
    """Drive to (x, y, theta) in the map frame; reverse is used when faster."""

    name = 'GoToPose'

    def __init__(self, P, x, y, theta, max_time=20.0):
        super().__init__(P)
        self.drive = DriveTo(P.robot, P.drive, [Pose(x, y, theta)])
        self.max_time = max_time

    def step(self, ctx):
        """One control cycle."""
        if self.elapsed(ctx) > self.max_time:
            return SkillOutput.fail('timeout')
        v, w, done = self.drive.command(ctx.pose)
        if done:
            g = self.drive.goal
            return SkillOutput.done('arrived', pos_err=dist(ctx.pose.xy, g.xy),
                                    heading_err=abs(wrap(g.theta - ctx.pose.theta)))
        self.phase = 'reverse' if v < 0 else 'forward' if v > 0 else 'turn'
        return self.out(v, w)


class TurnTo(Skill):
    """Turn in place to a heading."""

    name = 'TurnTo'

    def __init__(self, P, heading, tol=None):
        super().__init__(P)
        self.heading = heading
        self.tol = tol or P.drive.heading_tol

    def step(self, ctx):
        """One control cycle."""
        err = wrap(self.heading - ctx.pose.theta)
        if abs(err) < self.tol:
            return SkillOutput.done('aligned', heading_err=abs(err))
        return self.out(0.0, turn_rate(err, self.P.robot, self.P.drive))


# ----------------------------------------------------------------------------- SearchBall
class SearchBall(Skill):
    """
    Find the ball: look at its last estimate, spin in place, then patrol vantage points.

    SUCCESS once the camera confirmed the ball in confirm_frames consecutive cycles.
    """

    name = 'SearchBall'

    def __init__(self, P, vantage=None, spin_dir=None):
        super().__init__(P)
        self.c = P.search
        self.vantage = [tuple(v) for v in (vantage or self.c.vantage)]
        self.fixes = FixCounter()
        self.spin_dir = spin_dir
        self.spun = 0.0
        self.last_theta = None
        self.vantage_i = 0
        self.drive = None

    def step(self, ctx):
        """One control cycle."""
        self.elapsed(ctx)
        b = ctx.ball
        if self.fixes.update(ctx) >= self.c.confirm_frames:
            return SkillOutput.done('found', phase=self.phase)
        if self.last_theta is not None:
            self.spun += abs(wrap(ctx.pose.theta - self.last_theta))
        self.last_theta = ctx.pose.theta
        if not self.phase:
            recent = b is not None and b.age < self.c.turn_to_last_max_age
            self.phase = 'turn_last' if recent else 'spin'
            if recent:
                err = wrap(bearing(ctx.pose.xy, b.xy) - ctx.pose.theta)
                self.spin_dir = self.spin_dir or (1 if err >= 0 else -1)
            self.spin_dir = self.spin_dir or 1
        if self.phase == 'turn_last':
            err = wrap(bearing(ctx.pose.xy, b.xy) - ctx.pose.theta)
            if abs(err) < 0.15:
                self.phase, self.spun = 'spin', 0.0
            else:
                return self.out(0.0, turn_rate(err, self.P.robot, self.P.drive))
        if self.phase == 'spin':
            if self.spun < self.c.spin_turns * 2.0 * math.pi:
                return self.out(0.0, self.spin_dir * self.c.spin_speed)
            self.phase = 'patrol'
            vx, vy = self.vantage[self.vantage_i % len(self.vantage)]
            heading = bearing((vx, vy), ctx.field.opp_goal) if ctx.field else 0.0
            self.drive = DriveTo(self.P.robot, self.P.drive, [Pose(vx, vy, heading)])
        v, w, done = self.drive.command(ctx.pose)
        if done:
            self.vantage_i += 1
            self.phase, self.spun = 'spin', 0.0
        return self.out(v, w)


# ----------------------------------------------------------------------------- ApproachBall
class ApproachBall(Skill):
    """
    Stop in front of the ball at a given gap, facing a heading (or an aim point).

    Plans the fastest collision-free maneuver among candidates (direct, around either side,
    reverse out first) with kinematic rollouts, re-planned while the ball moves. A moving
    ball is intercepted at its predicted position.
    """

    name = 'ApproachBall'

    def __init__(self, P, heading=None, aim=None, gap=0.03, pos_tol=None, heading_tol=None,
                 predict=None, max_time=None):
        super().__init__(P)
        self.c = P.approach
        self.heading, self.aim, self.gap = heading, aim, gap
        self.pos_tol = pos_tol or self.c.pos_tol
        self.heading_tol = heading_tol or self.c.heading_tol
        self.predict = self.c.predict if predict is None else predict
        self.max_time = max_time or self.c.max_time
        self.plan = None
        self.plan_t = -1e9
        self.plan_ball = None
        self.drive = None
        self.target = None
        self.final = None
        self.local = LocalBall(P)

    def target_pose(self, ctx, ball_xy):
        """Pose at the gap behind ball_xy facing the approach heading."""
        if self.aim is not None:
            h = bearing(ball_xy, self.aim)
        elif self.heading is not None:
            h = self.heading
        else:
            h = bearing(ctx.pose.xy, ball_xy)
        standoff = self.P.robot.contact_dist + self.gap
        x, y = offset(ball_xy, h, -standoff)
        x, y = keep_inside((x, y), ctx.field, 0.08)
        return Pose(x, y, h)

    def ball_point(self, ctx):
        """Ball position to aim for (intercept of the predicted path when it moves)."""
        b = ctx.ball
        if not self.predict or b.speed < 0.05 or not b.seen_now and self.local.unseen > 0.3:
            return self.local.pos

        def eta(p):
            return time_estimate(ctx.pose, self.target_pose(ctx, p), self.P.robot)
        _, p = prediction.intercept(b, eta, decel=self.P.ball_model.roll_decel,
                                    field=ctx.field)
        return p

    def step(self, ctx):
        """One control cycle."""
        if self.elapsed(ctx) > self.max_time:
            return SkillOutput.fail('timeout')
        if ctx.ball is None:
            return SkillOutput.fail('ball_lost')
        self.local.update(ctx, self.dt(ctx))
        if self.local.lost(ctx, self.c.blind_time):
            return SkillOutput.fail('ball_lost')
        ball = self.ball_point(ctx)
        target = self.target_pose(ctx, ball)
        if self.final is not None and dist(ball, self.final[0]) < self.c.final_hold:
            target = self.final[1]               # hold the target while finishing
        rho = dist(ctx.pose.xy, target.xy)
        # Never turn in place with the ball at the bumper: back out and re-enter instead.
        touching = self.local.gap() is not None and self.local.gap() < self.c.final_min_gap \
            and abs(wrap(target.theta - ctx.pose.theta)) > 0.15
        lat0 = abs(-(ctx.pose.x - target.x) * math.sin(target.theta)
                   + (ctx.pose.y - target.y) * math.cos(target.theta))
        if self.final is not None or (rho < self.c.final_dist and not touching
                                      and lat0 < self.c.final_lateral):
            if self.final is None:
                self.final = (ball, target)
            return self.finish(ctx, target, ball)
        moved = self.plan_ball is None or dist(ball, self.plan_ball) > 0.03 \
            or abs(wrap(target.theta - self.target.theta)) > 0.15
        if moved or ctx.t - self.plan_t > self.c.replan_period:
            self.replan(ctx, ball, target)
        else:
            self.drive.wps[-1] = target          # follow small target motion
        self.target = target
        v, w, done = self.drive.command(ctx.pose)
        if done:
            self.drive.set_waypoints([target])
        if rho < self.c.final_slow_dist:
            s = min(1.0, self.c.final_speed / max(abs(v), 1e-6))
            v, w = v * s, w * s if abs(v) > 1e-6 else w
        self.phase = self.plan
        clear = ball_clearance(ctx.pose, ball, self.P.robot)
        return self.out(v, w, pos_err=rho, clearance=clear)

    def finish(self, ctx, target, ball):
        """Last few centimetres in the target frame: heading first, then creep along it."""
        c = self.c
        h = target.theta
        dx, dy = ctx.pose.x - target.x, ctx.pose.y - target.y
        along = dx * math.cos(h) + dy * math.sin(h)          # + = past the target
        lat = -dx * math.sin(h) + dy * math.cos(h)
        e_h = wrap(h - ctx.pose.theta)
        info = {'pos_err': math.hypot(along, lat), 'heading_err': abs(e_h), 'lateral': lat,
                'along': along, 'plan': self.plan}
        self.phase = 'finish'
        if abs(lat) > c.final_lateral_abort or abs(along) > c.final_dist * 1.6:
            self.final = None                    # drifted off: plan again
            self.plan = None
            self.drive = None
            self.plan_ball = None
            return self.out(0.0, 0.0, **info)
        if abs(e_h) > self.heading_tol:
            return self.out(0.0, turn_rate(e_h, self.P.robot, self.P.drive), **info)
        if abs(along) > self.pos_tol:
            v = -math.copysign(min(c.final_speed, 0.03 + 2.0 * abs(along)), along)
            w = clamp(4.0 * e_h - math.copysign(1.0, v) * 6.0 * lat, -1.0, 1.0)
            return self.out(v, w, **info)
        if abs(lat) <= c.final_lateral:
            return SkillOutput.done('in_position', **info)
        self.final = None
        self.plan = None
        self.drive = None
        self.plan_ball = None
        return self.out(0.0, 0.0, **info)

    def replan(self, ctx, ball, target):
        """Choose the fastest safe candidate, keeping the current one unless clearly worse."""
        r = self.P.robot
        min_clear = self.c.clearance
        corner = math.hypot(r.footprint[0], r.footprint[3])   # centre -> rear corner
        keep_out = r.ball_radius + corner + self.c.clearance
        obstacles = [(ox, oy, self.c.opponent_radius) for ox, oy in ctx.opponents
                     if dist((ox, oy), target.xy) > self.c.opponent_radius]
        name, wps, res = plan_around_ball(ctx.pose, target, ball, r, self.P.drive, min_clear,
                                          keep_out, v0=ctx.last_cmd[0], w0=ctx.last_cmd[1],
                                          obstacles=obstacles)
        if self.plan is not None and name != self.plan and self.drive is not None:
            from .motion import candidate_paths, rollout
            current = candidate_paths(ctx.pose, target, ball, r, keep_out,
                                      obstacles).get(self.plan)
            if current is not None:
                cur = rollout(ctx.pose, current, r, self.P.drive, ball,
                              v0=ctx.last_cmd[0], w0=ctx.last_cmd[1], obstacles=obstacles)
                if cur['ok'] and cur['clearance'] >= min_clear \
                        and cur['time'] < res['time'] + self.c.switch_gain:
                    name, wps = self.plan, current
        if name != self.plan or self.drive is None:
            self.drive = DriveTo(r, self.P.drive, wps)
        else:
            # Same maneuver: keep progress along it but refresh the geometry.
            idx = self.drive.index
            self.drive.wps = wps
            self.drive.index = min(idx, len(wps) - 1)
        self.plan, self.plan_t, self.plan_ball = name, ctx.t, ball


class OrbitBall(ApproachBall):
    """
    Re-orient around a ball we are already at: end facing a new heading, close to it.

    Differential drive cannot circle a ball while facing it, so this is ApproachBall with a
    small gap: the planner picks reverse-out-and-re-enter or a tight path around the ball,
    whichever its rollouts find faster and collision-free.
    """

    name = 'OrbitBall'

    def __init__(self, P, heading=None, aim=None, gap=0.015, **kw):
        super().__init__(P, heading=heading, aim=aim, gap=gap, **kw)


# ----------------------------------------------------------------------------- Dribble
class Dribble(Skill):
    """
    Move the ball along a path by tapping it with the bumper.

    Tap cycle: push at tap_speed until the ball has been at the bumper for push_time, then
    coast slower so the ball rolls ahead and re-enters the camera view, steer onto the line
    that pushes the ball back towards the path, and push again. mode 'push' keeps contact
    instead (no taps). Steering keeps the robot on the line through the ball centre whose
    direction corrects the ball's lateral path error (Stanley law).
    """

    name = 'Dribble'

    def __init__(self, P, path=None, target=None, mode=None, end_speed=None):
        super().__init__(P)
        self.c = P.dribble
        self.path_pts = path
        self.target = target
        self.path = None
        self.mode = mode or self.c.mode
        self.local = LocalBall(P)
        self.phase = 'push'
        self.phase_t = 0.0
        self.contact_t = 0.0
        self.sub = None
        self.taps = 0
        self.reorients = 0
        self.max_lateral = 0.0
        self.end_speed = end_speed

    def step(self, ctx):
        """One control cycle."""
        t = self.elapsed(ctx)
        dt = self.dt(ctx)
        if t > self.c.max_time:
            return SkillOutput.fail('timeout')
        if ctx.ball is None:
            return SkillOutput.fail('ball_lost')
        bl = self.local.update(ctx, dt)
        ball = self.local.map_xy(ctx.pose)
        if self.path is None:
            pts = [ball] + list(self.path_pts[1:]) if self.path_pts else [ball, self.target]
            self.path = Polyline(round_corners(pts, self.c.corner_radius))
        if self.local.lost(ctx, self.c.lost_time):
            return SkillOutput.fail('ball_lost', unseen=self.local.unseen)
        s, lat, _ = self.path.project(ball)
        self.max_lateral = max(self.max_lateral, abs(lat))
        if dist(ball, self.path.end) < self.c.goal_tol or s >= self.path.length - 0.01:
            return SkillOutput.done('arrived', taps=self.taps, reorients=self.reorients,
                                    max_lateral=self.max_lateral)
        psi_path = self.path.heading_at(s + self.c.lookahead)
        psi = psi_path - clamp(math.atan(self.c.k_path * lat), -self.c.max_correction,
                               self.c.max_correction)
        gap = self.local.gap()
        # Ball behind the bumper face (beside a wheel or under the robot): control lost.
        if bl[0] < self.P.robot.bumper_front_x - 0.005:
            return SkillOutput.fail('lost_control', gap=gap, lateral=bl[1])
        slipped = gap < 0.03 and abs(bl[1]) > self.c.recenter_lateral
        # Re-approach when the ball is far ahead; re-orient around it (OrbitBall) when the
        # push direction changes too much to turn with the ball on a flat bumper.
        turn_needed = abs(wrap(psi - ctx.pose.theta)) > self.c.reorient_angle
        if self.sub is not None or gap > self.c.lost_gap or turn_needed or slipped:
            if self.sub is None:
                self.sub = OrbitBall(self.P, heading=psi, gap=0.02, pos_tol=0.03,
                                     heading_tol=0.15)
                self.reorients += 1
            self.sub.heading = psi
            o = self.sub.step(ctx)
            if o.status == RUNNING:
                self.phase = 'reorient'
                return self.out(o.v, o.w, gap=gap, lateral=lat)
            self.sub = None
            if o.status == FAILURE:
                return SkillOutput.fail(o.reason)
            self.phase, self.phase_t = 'push', 0.0
        heading_err = wrap(psi - ctx.pose.theta)
        fresh = self.local.unseen < self.c.fresh_time
        aligned = abs(heading_err) < self.c.align_heading and \
            (not fresh or abs(bl[1]) < self.c.align_lateral)
        speed = self.speed(gap, aligned, dt)
        if fresh:
            # Fresh camera fix: put the robot on the line through the ball centre.
            v, w = line_follow(ctx.pose, ball, psi, speed, self.c.k_stanley,
                               self.c.k_heading, self.P.robot.max_ang)
        else:
            # Ball hidden at the bumper: only hold the push direction.
            v, w = speed, clamp(self.c.k_heading * heading_err, -self.P.robot.max_ang,
                                self.P.robot.max_ang)
        if gap < self.c.near_gap:
            # Turning with the ball on a flat bumper: keep the curvature low or it slides off;
            # never spin in place next to it. (Lateral-acceleration limits were tried and
            # lost the ball more often; large turns use OrbitBall instead.)
            lim = self.c.max_curvature * max(v, 0.0) + self.c.min_turn
            w = clamp(w, -lim, lim)
        if self.c.wiggle_amp > 0.0:
            w += self.c.wiggle_amp * math.sin(2.0 * math.pi * self.c.wiggle_freq * t)
        return self.out(v, w, gap=gap, lateral=lat, ball_y=bl[1], taps=self.taps)

    def speed(self, gap, aligned, dt):
        """Forward speed from the tap state machine (push <-> coast)."""
        c = self.c
        if self.mode == 'push':
            self.phase = 'push'
            return c.tap_speed if aligned else c.coast_speed
        self.phase_t += dt
        if self.phase == 'push':
            self.contact_t = self.contact_t + dt if gap <= c.gap_close else 0.0
            # hybrid: keep pushing while the last sighting is recent enough
            hold = self.mode == 'hybrid' and self.local.unseen < c.recheck_period and aligned
            if not hold and (self.contact_t >= c.push_time or
                             (not aligned and self.phase_t > 0.1)):
                self.phase, self.phase_t, self.contact_t = 'coast', 0.0, 0.0
            return c.tap_speed
        # coast: let the ball roll ahead into the camera view, then tap again once it was
        # seen (fresh fix) and we are aligned. Waiting is capped by coast_max.
        seen = self.local.unseen == 0.0
        if aligned and self.phase_t > 0.1 and (seen or self.phase_t >= c.coast_max):
            self.phase, self.phase_t, self.contact_t = 'push', 0.0, 0.0
            self.taps += 1
            return c.tap_speed
        return c.coast_speed if gap > c.gap_open else 0.0


# ----------------------------------------------------------------------------- ChargeShot
class ChargeShot(Skill):
    """
    Back off from the ball, then accelerate through it towards an aim point.

    Strength is set by the run-up distance `backoff`: the strike accelerates at
    strike_accel, so the predicted impact speed is min(max_lin, sqrt(2 * a * backoff)).
    """

    name = 'ChargeShot'

    def __init__(self, P, aim, backoff=None):
        super().__init__(P)
        self.c = P.shot
        self.aim = aim
        self.backoff = self.c.backoff if backoff is None else backoff
        self.phase = 'align'
        self.sub = None
        self.local = LocalBall(P)
        self.strike_t = None
        self.impact_v = None
        self.travel = 0.0
        self.line = None

    def quick_ready(self, ctx):
        """Return True when the ball is at the bumper and we already face the aim."""
        xy = self.local.xy
        if xy is None or self.local.gap() > 0.03 or abs(xy[1]) > self.c.quick_lateral:
            return False
        ball = self.local.map_xy(ctx.pose)
        return abs(wrap(bearing(ball, self.aim) - ctx.pose.theta)) < self.c.quick_heading

    @property
    def predicted_impact(self):
        """Impact speed expected from the run-up [m/s]."""
        return min(self.P.robot.max_lin, math.sqrt(2.0 * self.c.strike_accel * self.backoff))

    def step(self, ctx):
        """One control cycle."""
        if self.elapsed(ctx) > self.c.max_time:
            return SkillOutput.fail('timeout')
        dt = self.dt(ctx)
        if ctx.ball is None:
            return SkillOutput.fail('ball_lost')
        self.local.update(ctx, dt)
        ball = self.local.map_xy(ctx.pose)
        if self.phase == 'align' and self.sub is None and self.quick_ready(ctx):
            # Ball already at the bumper and pointing at the aim: just back off straight.
            self.phase, self.back_from = 'backoff', ctx.pose.xy
            self.line = (ball, bearing(ball, self.aim))
        if self.phase == 'backoff':
            if dist(ctx.pose.xy, self.back_from) >= self.backoff:
                self.phase, self.strike_t = 'strike', ctx.t
                self.line = (self.local.map_xy(ctx.pose), self.line[1])
            else:
                err = wrap(self.line[1] - ctx.pose.theta)
                return self.out(-self.P.robot.max_lin, clamp(4.0 * err, -1.0, 1.0))
        if self.phase == 'align':
            if self.sub is None:
                self.sub = ApproachBall(self.P, aim=self.aim, gap=self.backoff,
                                        pos_tol=self.c.align_lateral,
                                        heading_tol=self.c.align_heading)
            o = self.sub.step(ctx)
            if o.status == FAILURE:
                return SkillOutput.fail(o.reason)
            if o.status == RUNNING:
                return self.out(o.v, o.w, plan=o.phase)
            self.phase, self.strike_t = 'strike', ctx.t
            self.line = (ball, bearing(ball, self.aim))
        if self.phase == 'strike':
            ts = ctx.t - self.strike_t
            speed = min(self.P.robot.max_lin, self.c.strike_accel * ts + 0.02)
            self.travel += max(ctx.v, 0.0) * dt
            v, w = line_follow(ctx.pose, self.line[0], self.line[1], speed, self.c.k_stanley,
                               self.c.k_heading, self.P.robot.max_ang)
            gap = self.local.gap()
            if gap is not None and gap <= 0.003 or self.travel >= self.backoff + 0.01:
                self.phase, self.impact_v, self.travel = 'follow', ctx.v, 0.0
            elif ctx.ball.known and dist(ctx.ball.xy, self.line[0]) > 0.06:
                return SkillOutput.fail('ball_moved')
            return self.out(v, w, gap=gap)
        # follow-through: keep pushing along the shot line, stop before the goal line/wall
        self.travel += max(ctx.v, 0.0) * dt
        front = ctx.pose.to_world(self.P.robot.bumper_front_x, 0.0)
        at_line = ctx.field is not None and abs(front[0]) >= ctx.field.half_length - 0.005
        if self.travel >= self.c.follow_dist or at_line:
            return SkillOutput.done('shot', impact_speed=self.impact_v,
                                    predicted_impact=self.predicted_impact,
                                    backoff=self.backoff)
        v, w = line_follow(ctx.pose, self.line[0], self.line[1], self.impact_v or 0.1,
                           self.c.k_stanley, self.c.k_heading, self.P.robot.max_ang)
        return self.out(v, w)


# ----------------------------------------------------------------------------- RecoverBall
class RecoverBall(Skill):
    """Regain sight of a lost ball: back out of the camera blind zone, else search."""

    name = 'RecoverBall'

    def __init__(self, P, last_local=None):
        super().__init__(P)
        self.c = P.recover
        self.last_local = last_local
        self.phase = ''
        self.travel = 0.0
        self.fixes = FixCounter()
        self.search = None

    def step(self, ctx):
        """One control cycle."""
        dt = self.dt(ctx)
        b = ctx.ball
        if self.fixes.update(ctx) >= self.c.confirm_frames:
            return SkillOutput.done('found', phase=self.phase)
        if not self.phase:
            near = self.last_local
            if near is None and b is not None:
                near = ctx.ball_local()
            close = near is not None and math.hypot(*near) < self.c.near_dist + \
                self.P.robot.contact_dist
            self.phase = 'back' if close else 'search'
            self.spin_dir = 1 if (near is None or near[1] >= 0) else -1
        if self.phase == 'back':
            self.travel += abs(ctx.v) * dt
            if self.travel < self.c.back_dist:
                return self.out(-self.c.back_speed, 0.0)
            self.phase = 'search'
        if self.search is None:
            self.search = SearchBall(self.P, spin_dir=self.spin_dir)
        o = self.search.step(ctx)
        if o.status != RUNNING:
            return o
        return self.out(o.v, o.w, search=o.phase)


# ----------------------------------------------------------------------------- BlockShot
class BlockShot(Skill):
    """
    Stand on our goal's defence line between the ball and the goal, facing the ball.

    Uses the predicted crossing point of a ball moving towards our goal, otherwise the
    line from the ball to the goal centre. Drives backwards when that is faster so the
    camera keeps facing the ball. Runs until the caller switches skill.
    """

    name = 'BlockShot'

    def __init__(self, P):
        super().__init__(P)
        self.c = P.block
        self.drive = None
        self.local = LocalBall(P)
        self.hold_heading = None

    def ball(self, ctx):
        """Ball state re-predicted with the rolling model from the last fix."""
        from .context import BallEstimate
        lb = self.local
        return BallEstimate(lb.pos[0], lb.pos[1], lb.vel[0], lb.vel[1], ctx.ball.state,
                            lb.unseen)

    def intercept_point(self, ctx, x_line, side):
        """Earliest point on the predicted ball path (before our defence line) we reach first."""
        b, f = self.ball(ctx), ctx.field
        best = None
        t = 0.0
        while t <= self.c.horizon:
            p = prediction.predict(b, t, self.P.ball_model.roll_decel, f)
            if (p[0] - x_line) * side > 0.0:              # ball passed our line
                break
            face = bearing(p, b.xy) if dist(p, b.xy) > 0.05 else ctx.pose.theta
            eta = time_estimate(ctx.pose, Pose(p[0], p[1], face), self.P.robot)
            margin = t - eta
            if margin >= self.c.intercept_margin:
                return p
            if best is None or margin > best[0]:
                best = (margin, p)
            t += 0.05
        return None if best is None else best[1]

    def guard_point(self, ctx):
        """Point to stand on: intercept a ball heading to our goal, else the defence line."""
        f, b = ctx.field, self.ball(ctx)
        gx, gy = f.own_goal
        side = -1.0 if gx < 0 else 1.0                       # sign of our goal x
        x_line = gx - side * (self.c.line_offset + self.P.robot.contact_dist)
        half = f.goal_width / 2.0 + self.c.post_margin
        y = None
        if b.speed > 0.05 and b.vx * side > 0:
            hit = prediction.crossing_x(b, gx, self.c.horizon, self.P.ball_model.roll_decel)
            if hit is not None and abs(hit[1]) < half + 0.05:   # on target
                p = self.intercept_point(ctx, x_line, side)
                if p is not None:
                    return p
                y = prediction.crossing_x(b, x_line, self.c.horizon,
                                          self.P.ball_model.roll_decel)
                y = None if y is None else y[1]
        if y is None:
            dx = b.x - gx
            ratio = 0.0 if abs(dx) < 1e-6 else (x_line - gx) / dx
            y = gy + clamp(ratio, 0.0, 1.0) * (b.y - gy)
        return (x_line, clamp(y, -half, half))

    def step(self, ctx):
        """One control cycle."""
        self.elapsed(ctx)
        if ctx.ball is None:
            return SkillOutput.fail('ball_lost')
        self.local.update(ctx, self.dt(ctx))
        p = self.guard_point(ctx)
        ball = self.local.pos
        h = bearing(p, ball)
        if dist(p, ball) < 0.05:
            h = bearing(p, ctx.field.opp_goal)
        if self.hold_heading is not None and dist(ctx.pose.xy, p) < 2 * self.c.pos_tol \
                and abs(wrap(h - self.hold_heading)) < 0.6:
            h = self.hold_heading                 # in position: stand still, face stays
        goal = Pose(p[0], p[1], h)
        if self.drive is None:
            self.drive = DriveTo(self.P.robot, self.P.drive, [goal])
        self.drive.wps[-1] = goal
        if self.drive.turning and dist(ctx.pose.xy, goal.xy) > 2.5 * self.P.drive.pos_tol:
            self.drive.set_waypoints([goal])
        v, w, done = self.drive.command(ctx.pose)
        in_pos = dist(ctx.pose.xy, p) < self.c.pos_tol
        self.hold_heading = h if in_pos and self.hold_heading is None else \
            (self.hold_heading if in_pos else None)
        self.phase = 'hold' if in_pos else ('reverse' if v < 0 else 'move')
        return self.out(v, w, guard=p, in_position=in_pos)


# ----------------------------------------------------------------------------- ShadowBall
class ShadowBall(Skill):
    """
    Stand between the ball and our goal at `distance` from the ball, facing it.

    Keeps the ball in view (an opponent dribbling it is behind the ball, not between) and
    blocks the way to our goal. Runs until the caller switches; info.in_position says ready.
    """

    name = 'ShadowBall'

    def __init__(self, P, distance=0.25):
        super().__init__(P)
        self.distance = distance
        self.local = LocalBall(P)
        self.drive = None

    def step(self, ctx):
        """One control cycle."""
        if ctx.ball is None:
            return SkillOutput.fail('ball_lost')
        self.local.update(ctx, self.dt(ctx))
        ball = self.local.pos
        g = ctx.field.own_goal
        d = min(self.distance, 0.8 * dist(ball, g))
        p = offset(ball, bearing(ball, g), d)
        p = keep_inside(p, ctx.field, 0.09)
        goal = Pose(p[0], p[1], bearing(p, ball))
        if self.drive is None:
            self.drive = DriveTo(self.P.robot, self.P.drive, [goal])
        self.drive.wps[-1] = goal
        if self.drive.turning and dist(ctx.pose.xy, goal.xy) > 2.5 * self.P.drive.pos_tol:
            self.drive.set_waypoints([goal])
        v, w, _ = self.drive.command(ctx.pose)
        in_pos = dist(ctx.pose.xy, p) < 0.06
        self.phase = 'hold' if in_pos else 'move'
        return self.out(v, w, in_position=in_pos)


SKILLS = {cls.name: cls for cls in (GoToPose, TurnTo, SearchBall, ApproachBall, OrbitBall,
                                    Dribble, ChargeShot, RecoverBall, BlockShot, ShadowBall)}
__all__ = ['SKILLS', 'LocalBall', 'SUCCESS', 'FAILURE', 'RUNNING', 'OBSERVED', 'LOST',
           'unit']
