"""GoToPose time-optimal NMPC with optional collision constraints (same controller).

This is AcadosTimeOptimalMpc unchanged — dynamics, cost, terminal set, damping, observer
forecast, horizon search, replay acceptance — plus, only for GoToPose requests with an
avoid_* option set:
  * path and terminal constraints on the footprint disc cover (walls/goal blocks, circles);
    node 0 is the fixed initial state and keeps the original constraints
  * the exact footprint replay check at sub-samples between nodes
  * detour initial guesses around round obstacles (initialization only)
It needs a separately generated acados solver because the constraint and parameter
dimensions differ; the base OCP is built by the base class and only extended here.
All options off never reaches this class (mpc_controller uses the base solver).

Obstacle slack: the disc constraints carry an exact L1 penalty only so the SQP can
recover from an infeasible linearization or a start inside the margin. A candidate whose
exact replay comes closer than the margin is rejected, so slack never admits a collision.
The terminal elastic slack of the base OCP is not used for obstacles.
"""
import hashlib
import json
import math
import time

import numpy as np

from . import collision_geometry as cg
from .acados_setup import cache_dir
from .acados_time_optimal import AcadosTimeOptimalMpc

BASE_PARAMETERS = 9
CIRCLE_SLOTS = 3          # max_opponents (2) + ball
SLOT = 4                  # per circle slot: x, y, radius + margin, on flag
SUPER_P = 8               # goal block superellipse exponent
BLOCK_LOWER = -100.       # finite, well-scaled one-sided bounds for the interior-point QP
CIRCLE_UPPER = 1e4
OBSTACLE_L1, OBSTACLE_L2 = 1e3, 1e3


class AvoidanceTimeOptimalMpc(AcadosTimeOptimalMpc):
    """Base GoToPose NMPC + optional wall/goal and circle obstacle constraints."""

    soft_obstacles = True     # exact L1 slack (replay stays strict)
    # Numerical settings of this generated solver only (base solver unchanged): the extra
    # path constraints make the dense blocks of 2-block partial condensing expensive
    # (measured ~3x faster QP iterations with 10 blocks at N 34-50), and each SQP
    # iteration costs 2-3x the base, so one candidate gets 35 ms of the 75 ms budget.
    qp_cond_n = 10
    avoid_candidate_timeout = .035

    def __init__(self, *args, **kwargs):
        self.snapshot = None
        self.required = None
        self.extra = self.inactive_extra()
        super().__init__(*args, **kwargs)

    # ------------------------------------------------------------------ OCP extension
    def build_ocp(self):
        import casadi as ca
        ocp = super().build_ocp()
        model = ocp.model
        x = model.x
        extra = ca.SX.sym('avoid', 6 + SLOT * CIRCLE_SLOTS)
        w_on, xl, yl, gx, gyh, mg = (extra[i] for i in range(6))
        c, s = ca.cos(x[2]), ca.sin(x[2])
        k = 2 ** (1 / SUPER_P)
        walls, blocks, circles = [], [], []
        for ox, oy, r in cg.FOOTPRINT_DISCS:
            cx = x[0] + c * ox - s * oy
            cy = x[1] + s * ox + c * oy
            walls += [w_on * cx / (xl - r - mg), w_on * cy / (yl - r - mg)]
            for side in (-1, 1):
                # Superellipse containing the goal block inflated by disc radius + margin.
                a = (xl - gx + r + mg) * k
                b = (gyh + r + mg) * k
                norm = ((cx - side * xl) / a) ** SUPER_P + (cy / b) ** SUPER_P
                blocks.append(w_on * (1 - norm ** (1 / SUPER_P)))
            for i in range(CIRCLE_SLOTS):
                qx, qy, rad, on = (extra[6 + SLOT * i + j] for j in range(SLOT))
                # Inactive slot: on = 0 -> h = 0, always within [0, CIRCLE_UPPER].
                circles.append(on * (((cx - qx) ** 2 + (cy - qy) ** 2) / (rad + r) ** 2 - 1))
        obstacle = ca.vertcat(*walls, *blocks, *circles)
        self.n_obstacle = obstacle.shape[0]
        self.obstacle_lower = np.r_[-np.ones(len(walls)), BLOCK_LOWER * np.ones(len(blocks)),
                                    np.zeros(len(circles))]
        self.obstacle_upper = np.r_[np.ones(len(walls)), np.zeros(len(blocks)),
                                    CIRCLE_UPPER * np.ones(len(circles))]
        model.p = ca.vertcat(model.p, extra)
        base_h = model.con_h_expr
        model.con_h_expr_0 = base_h          # fixed initial state: original constraints
        model.con_h_expr = ca.vertcat(base_h, obstacle)
        model.con_h_expr_e = ca.vertcat(model.con_h_expr_e, obstacle)
        cons = ocp.constraints
        nb = base_h.shape[0]
        cons.lh = np.r_[cons.lh, self.obstacle_lower]
        cons.uh = np.r_[cons.uh, self.obstacle_upper]
        ne = len(cons.lh_e)
        cons.lh_e = np.r_[cons.lh_e, self.obstacle_lower]
        cons.uh_e = np.r_[cons.uh_e, self.obstacle_upper]
        no = self.n_obstacle
        self.n_obstacle_soft = no if self.soft_obstacles else 0
        if self.soft_obstacles:
            cons.idxsh = np.arange(nb, nb + no)
            cons.lsh = cons.ush = np.zeros(no)
            ocp.cost.zl = ocp.cost.zu = OBSTACLE_L1 * np.ones(no)
            ocp.cost.Zl = ocp.cost.Zu = OBSTACLE_L2 * np.ones(no)
            cons.idxsh_e = np.r_[cons.idxsh_e, np.arange(ne, ne + no)]
            cons.lsh_e = np.r_[cons.lsh_e, np.zeros(no)]
            cons.ush_e = np.r_[cons.ush_e, np.zeros(no)]
            ocp.cost.zl_e = np.r_[ocp.cost.zl_e, OBSTACLE_L1 * np.ones(no)]
            ocp.cost.zu_e = np.r_[ocp.cost.zu_e, OBSTACLE_L1 * np.ones(no)]
            ocp.cost.Zl_e = np.r_[ocp.cost.Zl_e, OBSTACLE_L2 * np.ones(no)]
            ocp.cost.Zu_e = np.r_[ocp.cost.Zu_e, OBSTACLE_L2 * np.ones(no)]
        if self.qp_cond_n is not None:
            ocp.solver_options.qp_solver_cond_N = int(self.qp_cond_n)
        self.candidate_timeout = min(self.avoid_candidate_timeout, self.budget / 2)
        ocp.solver_options.timeout_max_time = self.candidate_timeout
        ocp.parameter_values = np.r_[ocp.parameter_values, self.inactive_extra()]
        key = json.dumps(dict(base=model.name, version='avoid-v2', discs=cg.FOOTPRINT_DISCS,
                              slots=CIRCLE_SLOTS, p=SUPER_P, penalty=[OBSTACLE_L1, OBSTACLE_L2],
                              soft=self.soft_obstacles, cond=self.qp_cond_n,
                              timeout=self.candidate_timeout),
                         sort_keys=True)
        model.name = 'pinky_avoid_' + hashlib.sha256(key.encode()).hexdigest()[:12]
        ocp.code_gen_options.code_export_directory = str(
            cache_dir(f'c_generated_code_{model.name}'))
        ocp.code_gen_options.json_file = f'{model.name}.json'
        return ocp

    @staticmethod
    def inactive_extra():
        """Walls off, circle slots far away."""
        out = np.zeros(6 + SLOT * CIRCLE_SLOTS)
        out[1:6] = (1., 1., .5, .1, 0.)
        for i in range(CIRCLE_SLOTS):
            out[6 + SLOT * i:6 + SLOT * (i + 1)] = (0., 0., .1, 0.)
        return out

    def extra_parameters(self):
        snap = self.snapshot
        out = self.inactive_extra()
        if snap is None:
            return out
        margin = snap.config.margin + snap.config.solver_margin
        if snap.walls is not None:
            f = snap.walls
            out[:6] = (1., f.x_limit, f.y_limit, f.goal_front, f.goal_half_width, margin)
        for i, circle in enumerate(snap.circles[:CIRCLE_SLOTS]):
            out[6 + SLOT * i:6 + SLOT * (i + 1)] = (circle.x, circle.y, circle.radius + margin, 1.)
        return out

    # ------------------------------------------------------------------ hooks
    def stage_parameters(self, k, p):
        return np.r_[p[:BASE_PARAMETERS], self.extra]

    def stage_bounds(self, k, lo, hi):
        if k == 0:
            return lo, hi
        return np.r_[lo, self.obstacle_lower], np.r_[hi, self.obstacle_upper]

    def terminal_slack(self, sl, su):
        z = np.zeros(self.n_obstacle_soft)
        return np.r_[sl, z], np.r_[su, z]

    # ------------------------------------------------------------------ replay
    def collision_violation(self, xs):
        """Largest shortfall [m] of the exact footprint clearance at replay sub-samples."""
        if self.snapshot is None or self.required is None or not len(self.required):
            return 0.0
        poses = cg.interpolate_poses(xs)
        _, clear = self.snapshot.clearances(poses)
        if not clear.size:
            return 0.0
        return float(max(0.0, np.max(self.required[None, :] - clear)))

    def validate_candidate(self, x, goal, prev, us, vb, wb, forward, terminal):
        xs, violation = super().validate_candidate(x, goal, prev, us, vb, wb, forward, terminal)
        if xs is None:
            return xs, violation
        return xs, max(violation, self.collision_violation(xs))

    def attempt(self, n, x, goal, prev, vb, wb, forward, seed, deadline, terminal=True):
        # Once a terminal-feasible horizon is accepted, do not start a shorter-horizon
        # candidate that cannot finish inside the budget (each costs up to
        # candidate_timeout); the activation slot needs the result before its boundary.
        if terminal and any(a['accepted'] and a['terminal'] for a in self.attempt_statuses):
            if time.perf_counter() + self.candidate_timeout > deadline:
                return None
        return super().attempt(n, x, goal, prev, vb, wb, forward, seed, deadline, terminal)

    # ------------------------------------------------------------------ seeds
    def seed(self, n, x, goal, prev, vb, wb, forward, direction):
        """Base rollout seed, or a detour via the side of a round obstacle it would hit."""
        base = super().seed(n, x, goal, prev, vb, wb, forward, direction)
        if self.snapshot is None or not self.snapshot.circles or n < 4:
            return base
        best, best_score = base, self.seed_score(x, base, goal)
        if best_score[0] == 0:
            return base
        for circle in self.snapshot.circles:
            for side in (-1, 1):
                way = self.waypoint(x, goal, circle, side)
                if way is None:
                    continue
                n1 = max(2, n // 2)
                u1 = super().seed(n1, x, way, prev, vb, wb, forward, direction)
                mid = self.replay(x, u1)[-1]
                u2 = super().seed(n - n1, mid, goal, u1[-1], vb, wb, forward, direction)
                cand = np.vstack([u1, u2])
                score = self.seed_score(x, cand, goal)
                if score < best_score:
                    best, best_score = cand, score
        return best

    def waypoint(self, x, goal, circle, side):
        """Pose beside `circle`, perpendicular to the start-goal line, heading along it."""
        a, b = np.asarray(x[:2], float), np.asarray(goal[:2], float)
        d = b - a
        length = float(np.linalg.norm(d))
        if length < 1e-6:
            return None
        t = d / length
        normal = np.array((-t[1], t[0])) * side
        centre = np.array((circle.x, circle.y))
        along = float(np.clip(np.dot(centre - a, t), 0.0, length))
        offset = circle.radius + self.snapshot.config.margin + 0.10
        p = a + along * t + normal * (float(np.dot(centre - a, normal)) + offset)
        return (float(p[0]), float(p[1]), math.atan2(t[1], t[0]))

    def seed_score(self, x, us, goal):
        """(collision shortfall rounded to 1 mm, terminal merit): lexicographic."""
        xs = self.replay(x, us)
        return (round(self.collision_violation(xs), 3), self.terminal_merit(xs, us, goal))

    # ------------------------------------------------------------------ solve
    def solve(self, x0, goal, v_cap=None, w_cap=None, forward_only=False,
              disturbance=(0., 0.), obstacles=None):
        """Solve with the given ObstacleSnapshot (None = no obstacles, inactive slots)."""
        self.snapshot = obstacles
        self.extra = self.extra_parameters()
        self.required = None
        start_clear = None
        if obstacles is not None:
            names, clear = obstacles.clearances(np.asarray(x0, float)[:3])
            start_clear = clear[0] if clear.size else np.zeros(0)
            # A start already inside the margin may not get closer than it is.
            self.required = np.minimum(obstacles.config.margin, start_clear - 1e-3)
        result = super().solve(x0, goal, v_cap, w_cap, forward_only=forward_only,
                               disturbance=disturbance)
        info = obstacles.describe() if obstacles is not None else {}
        if result.get('ok') and obstacles is not None and len(result['xs']):
            _, clear = obstacles.clearances(cg.interpolate_poses(result['xs']))
            info['planned_min_clearance_m'] = float(np.min(clear)) if clear.size else None
        info['start_clearance_m'] = (float(np.min(start_clear))
                                     if start_clear is not None and len(start_clear) else None)
        result['avoidance'] = info
        return result
