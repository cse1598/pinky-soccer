"""
Composite behaviours (L3): choose and chain skills. Replaces the old BT leaf logic.

Striker (1v1 player) priorities, re-evaluated every cycle:
  1. no ball            -> RecoverBall (just lost near us) / SearchBall
  2. danger             -> BlockShot   (ball rolling at our goal, or opponent first on a
                                        ball in our half)
  3. ball in shot reach -> ChargeShot  (within reach of the goal mouth)
  4. ball at our bumper -> Dribble     (towards the shot spot; around opponents = CurveDribble)
  5. opponent has it    -> steal: ShadowBall (between ball and our goal, ball in view), then
                          poke: ApproachBall from the side (perpendicular to its dribble)
                          and carry: Dribble the ball away sideways
  6. otherwise          -> ApproachBall(aim at the goal; intercepts a rolling ball)
A running skill is kept until it ends or a higher priority interrupts it, so behaviour
does not flicker between skills.
"""

import math

from . import prediction
from .context import FAILURE, RUNNING, SkillOutput, SUCCESS
from .geometry import clamp, dist
from .planning import avoid_path, shot_aim
from .skills import (ApproachBall, BlockShot, ChargeShot, Dribble, LocalBall, RecoverBall,
                     SearchBall, ShadowBall)


class Striker:
    """Single-robot attacker/defender playbook."""

    name = 'Striker'

    def __init__(self, P, defend=True):
        self.P = P
        self.c = P.playbook
        self.defend = defend
        self.skill = None
        self.mode = None
        self.local = LocalBall(P)
        self.steal_dir = 0.0
        self.prev_t = None
        self.history = []

    # ------------------------------------------------------------------ situation
    def ball_threat(self, ctx):
        """Ball rolling at our goal fast enough to arrive."""
        b, f = ctx.ball, ctx.field
        side = math.copysign(1.0, f.own_goal[0])
        if b is None or b.speed < 0.12 or b.vx * side <= 0:
            return False
        hit = prediction.crossing_x(b, f.own_goal[0], 4.0, self.P.ball_model.roll_decel)
        return hit is not None and abs(hit[1]) < f.goal_width / 2.0 + 0.05

    def opponent_first(self, ctx, ball):
        """Return True when an opponent is closer to the ball, with the ball in our half."""
        f = ctx.field
        side = math.copysign(1.0, f.own_goal[0])
        in_our_half = ball[0] * side > -self.c.defend_margin
        mine = dist(ctx.pose.xy, ball)
        return in_our_half and any(dist(o, ball) + 0.05 < mine for o in ctx.opponents)

    def opponent_possessing(self, ctx, ball):
        """Closest opponent within steal_possess of the ball, or None."""
        best = None
        for o in ctx.opponents:
            d = dist(o, ball)
            if d < self.c.steal_possess and (best is None or d < best[0]):
                best = (d, o)
        return None if best is None else best[1]

    def steal_heading(self, ctx, ball, opp):
        """Push direction across the opponent's dribble, from the side we are on."""
        d = math.atan2(ball[1] - opp[1], ball[0] - opp[0])
        rx, ry = ctx.pose.x - ball[0], ctx.pose.y - ball[1]
        cross = math.cos(d) * ry - math.sin(d) * rx        # + : we are left of its path
        return d - math.pi / 2 if cross > 0 else d + math.pi / 2

    def possessing(self, ball_local):
        """Ball at our bumper, roughly centred."""
        if ball_local is None:
            return False
        gap = ball_local[0] - self.P.robot.contact_dist
        return -0.01 < gap < self.c.possess_gap and abs(ball_local[1]) < self.c.possess_lateral

    def in_shot_reach(self, ball, ctx):
        """Ball close enough to the goal mouth for a ChargeShot to score."""
        f = ctx.field
        gx = f.opp_goal[0]
        to_line = abs(gx - ball[0])
        return to_line < self.c.shot_range and abs(ball[1]) < f.goal_width / 2.0 + 0.02

    # ------------------------------------------------------------------ decision
    def decide(self, ctx, ball_local):
        """Mode wanted now (None = keep the current skill)."""
        if ctx.ball is None or self.local.lost(ctx, self.P.dribble.lost_time):
            return 'search'
        ball = self.local.pos
        if self.defend and (self.ball_threat(ctx) or self.opponent_first(ctx, ball)):
            return 'defend'
        if self.in_shot_reach(ball, ctx):
            return 'shoot'
        if self.possessing(ball_local):
            return 'dribble'
        if self.opponent_possessing(ctx, ball) is not None:
            return 'steal'
        return 'approach'

    def make(self, mode, ctx):
        """Instantiate the skill for a mode."""
        ball = self.local.pos
        f = ctx.field
        aim = shot_aim(ball if ball else (0.0, 0.0), f, self.c.aim_inset)
        if mode == 'search':
            if self.local.xy is not None and math.hypot(*self.local.xy) < 0.25:
                return RecoverBall(self.P, last_local=self.local.xy)
            gx = f.own_goal[0]
            return SearchBall(self.P, vantage=[(0.55 * gx, 0.0), (0.0, 0.0)])
        if mode == 'defend':
            return BlockShot(self.P)
        if mode == 'steal':
            return ShadowBall(self.P, self.c.shadow_distance)
        if mode == 'poke':
            opp = self.opponent_possessing(ctx, ball) or ctx.pose.xy
            self.steal_dir = self.steal_heading(ctx, ball, opp)
            return ApproachBall(self.P, heading=self.steal_dir, gap=0.0, heading_tol=0.3,
                                pos_tol=0.04, predict=True)
        if mode == 'carry':
            end = (ball[0] + self.c.steal_carry * math.cos(self.steal_dir),
                   ball[1] + self.c.steal_carry * math.sin(self.steal_dir))
            end = (clamp(end[0], -f.wall_x + 0.15, f.wall_x - 0.15),
                   clamp(end[1], -f.wall_y + 0.15, f.wall_y - 0.15))
            return Dribble(self.P, path=[ball, end])
        if mode == 'shoot':
            return ChargeShot(self.P, aim=aim, backoff=self.c.shot_backoff)
        if mode == 'dribble':
            spot = self.shot_spot(ball, aim, f)
            path = avoid_path(ball, spot, ctx.opponents, self.c.opponent_avoid_radius, f)
            return Dribble(self.P, path=path)
        # Face the first leg of the dribble path we will take, so the dribble starts
        # straight away instead of re-orienting at the ball.
        spot = self.shot_spot(ball, aim, f)
        path = avoid_path(ball, spot, ctx.opponents, self.c.opponent_avoid_radius, f)
        return ApproachBall(self.P, aim=path[1], gap=self.c.approach_gap)

    def shot_spot(self, ball, aim, f):
        """Where to dribble to: inside shot reach, in front of the goal."""
        gx = f.opp_goal[0]
        x = gx - math.copysign(0.6 * self.c.shot_range, gx)
        return (x, clamp(aim[1], -f.goal_width / 2.0 + 0.05, f.goal_width / 2.0 - 0.05))

    # ------------------------------------------------------------------ step
    def step(self, ctx):
        """One control cycle."""
        dt = 0.0 if self.prev_t is None else ctx.t - self.prev_t
        self.prev_t = ctx.t
        ball_local = self.local.update(ctx, dt) if ctx.ball is not None else None
        want = self.decide(ctx, ball_local)
        if self.mode in ('poke', 'carry') and want in ('steal', 'approach'):
            want = self.mode
        interrupt = (self.mode != want and (
            want in ('search', 'defend') or self.mode in ('search', 'defend')
            or self.skill is None
            or (self.mode in ('approach', 'steal') and want in ('dribble', 'shoot'))
            or (self.mode == 'approach' and want == 'steal')
            or (self.mode in ('poke', 'carry') and want in ('dribble', 'shoot', 'search',
                                                            'defend'))
            or (self.mode == 'dribble' and want == 'shoot')))
        if self.skill is None or interrupt:
            self.switch(want, ctx)
        out = self.skill.step(ctx)
        if self.mode == 'steal' and out.info.get('in_position') and \
                self.local.unseen < 0.3 and dist(ctx.pose.xy, self.local.pos) < self.c.poke_dist:
            self.switch('poke', ctx)
            out = self.skill.step(ctx)
        if out.status != RUNNING:
            nxt = self.after(self.mode, out, ctx)
            self.switch(nxt, ctx)
            out = self.skill.step(ctx)
        out.info['skill'] = self.skill.name
        out.info['mode'] = self.mode
        out.phase = f'{self.mode}:{out.phase}'
        return SkillOutput(out.v, out.w, RUNNING, out.reason, out.phase, out.info)

    def after(self, mode, out, ctx):
        """Next mode when a skill finished."""
        if mode == 'approach' and out.status == SUCCESS:
            return 'dribble'
        if mode == 'poke' and out.status == SUCCESS:
            return 'carry'
        if mode == 'carry':
            return 'approach'
        if mode == 'dribble' and out.status == SUCCESS:
            return 'shoot'
        if mode in ('dribble', 'shoot') and out.status == FAILURE and out.reason == 'ball_lost':
            return 'search'
        return self.decide(ctx, self.local.xy) if mode != 'approach' else 'approach'

    def switch(self, mode, ctx):
        """Start a new skill for mode."""
        self.mode = mode
        self.skill = self.make(mode, ctx)
        self.history.append((round(ctx.t, 2), mode))


BEHAVIORS = {'striker': Striker}
