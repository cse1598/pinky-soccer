"""
Point-to-pose NMPC for Pinky Pro (stage 3): reach (x, y, theta) as fast as the wheels allow.

Model: pinky_soccer_dynamics wheel model, state [x, y, th, Om_L, Om_R], input = wheel speed
targets [Om_L,cmd, Om_R,cmd] (rad/s, + = drives the robot forward).

Cost (Gauss-Newton least squares, per stage, integrated over the horizon):
  w_time * T(x)                               estimated time-to-go (below), at every stage:
        -> the integral of the remaining time; with T only at the end, turning now or at the
           end of the horizon costs the same and the solver postponed it forever
  w_pos * sqrt(d^2 + eps^2)                  d = distance to the goal position
        -> L1-like: a constant pull independent of distance, so the optimum is "full speed
           until the constraints force braking" (a smooth minimum-time surrogate), instead of
           the quadratic cost's slow exponential approach
  w_head * near(d) * (1 - cos(th - th_goal)) near(d) = r_h^2 / (d^2 + r_h^2)
        -> heading matters only close to the goal, so the robot does not turn early but
           arrives already facing the goal heading when the horizon allows a curve
  w_acc * (wheel acceleration)^2               small, removes chattering
Terminal: estimated time-to-go T from the horizon's end state (cost-to-go), plus position,
heading and wheel speeds (stop at the goal):
  T = s(d) (|e1| + |e2|) / w_b + d / v_b + (1 - s(d)) |th_goal - th| / w_b
  e1 = turn to face the goal (or its back when reversing is allowed), e2 = turn from there to
  th_goal, s(d) = d^2 / (d^2 + r_s^2), v_b/w_b = this solve's speed bounds.
  Without it the horizon is myopic: a goal behind the robot needs a turn longer than the
  horizon, inside which turning only costs, so "stay still" is optimal (seen in MuJoCo).

Constraints (hard bounds on the input, soft on the path):
  |Om_cmd|  <= usage_ratio x manual rated speed   (wheel_cmd_max, 9.71 rad/s by default)
  |Om_dot|  <= usage_ratio x motor acceleration   (wheel_accel_cmd_max, 67.4 rad/s^2)
  |v| <= v_cap, |w| <= w_cap                      (online: launch caps, BehaviorGoal.max_speed)
Reverse driving is allowed by default (diff drive: backing up is often the fastest way to a
pose). solve(forward_only=True) bounds v >= 0 (soft) for goals where the camera must keep
looking ahead, e.g. around the ball.
"""

import hashlib
import json
import time

import numpy as np

from .acados_setup import acados, cache_dir

DEFAULT_WEIGHTS = {
    'pos': 10.0,        # sqrt-distance residual (L1-like position cost)
    'time': 40.0,       # time-to-go at every stage: progress now beats progress later
    'eps': 0.02,        # [m] smoothing of the L1 distance near the goal
    'head': 6.0,        # heading residual near the goal
    'head_radius': 0.15,  # [m] r_h: heading weight is half at this distance
    'acc': 2e-4,        # wheel acceleration regularisation
    'pos_e': 400.0,     # terminal position
    'head_e': 40.0,     # terminal heading
    'speed_e': 0.5,     # terminal wheel speed (stop at the goal)
    'time_e': 60.0,     # terminal time-to-go [per second]
    'time_radius': 0.05,  # [m] r_s: below this the goal heading replaces the turn-drive-turn
    'slack': 1e3,       # soft path constraint penalty (L1 and L2)
}


class PoseMpc:
    """acados point-to-pose NMPC (generated once per parameter set, cached)."""

    def __init__(self, params, horizon=1.5, steps=30, weights=None, name='pinky_pose'):
        self.params = params
        self.horizon, self.steps = float(horizon), int(steps)
        self.dt = self.horizon / self.steps
        self.w = dict(DEFAULT_WEIGHTS, **(weights or {}))
        self.wmax = params.wheel_cmd_max()
        self.amax = params.wheel_accel_cmd_max()
        self.rl, self.rr, self.b = (params.m('left_radius'), params.m('right_radius'),
                                    params.m('track'))
        self.v_limit = 0.5 * (self.rl + self.rr) * self.wmax        # straight line
        self.w_limit = (self.rl + self.rr) * self.wmax / self.b      # spin in place
        self.ocp = self._build(name)
        at = acados()
        json_file = f'{self.ocp.model.name}.json'          # written into the export dir
        lib = (cache_dir(f'c_generated_code_{self.ocp.model.name}')
               / f'libacados_ocp_solver_{self.ocp.model.name}.so')
        fresh = not lib.exists()
        self.solver = at.AcadosOcpSolver(self.ocp, json_file=json_file, build=fresh,
                                         generate=fresh, verbose=False)
        self.xs = None
        self.us = None
        self.forward_only = False
        self.count = 0
        self.reseed_every = 5            # re-check the primitive seeds at 4 Hz (20 Hz loop)
        self.final_approach = 0.12       # [m] no reseeding closer than this
        self.switch_margin = 0.85        # a reseeded plan must be 15 % cheaper to replace

    # ------------------------------------------------------------------ OCP
    def _time_to_go(self, ca, x, p):
        """Smooth turn-drive-turn minimum-time estimate (see module docstring)."""
        def sabs(z):
            return ca.sqrt(z * z + 1e-4)

        def wrap(z):
            return ca.atan2(ca.sin(z), ca.cos(z))
        dx, dy = p[0] - x[0], p[1] - x[1]
        d2 = dx * dx + dy * dy
        dist = ca.sqrt(d2 + 1e-6)
        e1 = wrap(ca.atan2(dy, dx) - x[2])
        axis = ca.atan2(ca.sin(e1), ca.sqrt(ca.cos(e1) ** 2 + 1e-4))   # nearest of front/back
        e1 = p[3] * axis + (1 - p[3]) * e1
        e2 = wrap(p[2] - x[2] - e1)
        s = d2 / (d2 + self.w['time_radius'] ** 2)
        return (s * (sabs(e1) + sabs(e2)) / p[5] + dist / p[4]
                + (1 - s) * sabs(wrap(p[2] - x[2])) / p[5])

    def _build(self, name):
        at = acados()
        import casadi as ca
        from pinky_soccer_dynamics.model import casadi_model
        x, u, xdot, f_expl, acc = casadi_model(self.params)
        p = ca.SX.sym('p', 6)          # goal x, y, theta, reverse allowed, v bound, w bound
        w = self.w
        key = json.dumps({'model': self.params.as_dict()['model'],
                          'limits': [self.wmax, self.amax], 'w': w,
                          'h': [self.horizon, self.steps]}, sort_keys=True, default=str)
        model = at.AcadosModel()
        model.name = f'{name}_{hashlib.sha1(key.encode()).hexdigest()[:10]}'
        model.x, model.u, model.xdot, model.p = x, u, xdot, p
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl
        dx, dy = x[0] - p[0], x[1] - p[1]
        d2 = dx * dx + dy * dy
        near = w['head_radius'] ** 2 / (d2 + w['head_radius'] ** 2)
        half = ca.sin(0.5 * (x[2] - p[2]))       # sin^2(e/2) = (1 - cos e) / 2
        v = 0.5 * (self.rr * x[4] + self.rl * x[3])
        om = (self.rr * x[4] - self.rl * x[3]) / self.b
        t_go = self._time_to_go(ca, x, p)
        model.cost_y_expr = ca.vertcat((d2 + w['eps'] ** 2) ** 0.25,
                                       ca.sqrt(2.0 * near) * half, acc, ca.sqrt(t_go + 1e-6))
        model.cost_y_expr_e = ca.vertcat(dx, dy, ca.sqrt(2.0) * half, x[3], x[4],
                                         ca.sqrt(t_go + 1e-6))
        path = ca.vertcat(acc, v, om)
        model.con_h_expr_0 = path
        model.con_h_expr = path

        ocp = at.AcadosOcp()
        ocp.model = model
        ocp.parameter_values = np.array([0, 0, 0, 1, self.v_limit, self.w_limit], float)
        o = ocp.solver_options
        o.N_horizon = self.steps
        o.tf = self.horizon
        ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W = np.diag([w['pos'], w['head'], w['acc'], w['acc'], w['time']])
        ocp.cost.W_e = np.diag([w['pos_e'], w['pos_e'], w['head_e'], w['speed_e'],
                                w['speed_e'], w['time_e']])
        ocp.cost.yref = np.zeros(5)
        ocp.cost.yref_e = np.zeros(6)
        c = ocp.constraints
        c.lbu, c.ubu, c.idxbu = -self.wmax * np.ones(2), self.wmax * np.ones(2), np.array([0, 1])
        bounds = np.array([self.amax, self.amax, self.v_limit, self.w_limit])
        c.lh, c.uh = -bounds, bounds
        c.lh_0, c.uh_0 = -bounds, bounds
        c.idxsh = c.idxsh_0 = np.arange(4)
        s = w['slack'] * np.ones(4)
        ocp.cost.zl = ocp.cost.zu = ocp.cost.Zl = ocp.cost.Zu = s
        ocp.cost.zl_0 = ocp.cost.zu_0 = ocp.cost.Zl_0 = ocp.cost.Zu_0 = s
        c.x0 = np.zeros(5)
        o.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        o.hessian_approx = 'GAUSS_NEWTON'
        o.integrator_type = 'ERK'
        o.sim_method_num_stages = 4
        o.sim_method_num_steps = 1
        o.nlp_solver_type = 'SQP'
        o.nlp_solver_max_iter = 15
        o.tol = 1e-3
        o.globalization = 'MERIT_BACKTRACKING'
        ocp.code_export_directory = str(cache_dir(f'c_generated_code_{model.name}'))
        return ocp

    # ------------------------------------------------------------------ solve
    def solve(self, x0, goal, v_cap=None, w_cap=None, forward_only=False):
        """
        Solve from state x0 towards goal (x, y, theta).

        Returns dict(u=(Om_L, Om_R), status, xs, us, ms). Caps below the wheel limits lower
        the linear/angular speed bounds for this solve.
        """
        s = self.solver
        n = self.steps
        x0 = np.asarray(x0, float)
        goal = np.asarray(goal, float).copy()
        goal[2] = x0[2] + np.arctan2(np.sin(goal[2] - x0[2]), np.cos(goal[2] - x0[2]))
        vb = min(self.v_limit, v_cap) if v_cap and v_cap > 0 else self.v_limit
        wb = min(self.w_limit, w_cap) if w_cap and w_cap > 0 else self.w_limit
        upper = np.array([self.amax, self.amax, vb, wb])
        lower = -upper
        if forward_only:
            lower[2] = 0.0
        params = np.array([*goal, 0.0 if forward_only else 1.0, vb, wb])
        for k in range(n + 1):
            s.set(k, 'p', params)
        for k in range(n):
            s.constraints_set(k, 'lh', lower)
            s.constraints_set(k, 'uh', upper)
        # Warm start: shift the previous solution; cold starts are seeded with primitives.
        if forward_only != self.forward_only:
            self.reset()                           # direction rule changed: old guess invalid
        self.forward_only = forward_only
        t0 = time.perf_counter()
        # Candidates: the shifted previous solution (warm start) and, on a cold start and every
        # `reseed_every` solves, the primitive seeds (forward, and reversing when allowed).
        # The OCP is non-convex: a warm start can sit in a "stay still" local minimum (turning
        # away first raises the heading cost before driving lowers the distance), so the
        # cheapest refined candidate wins.
        seeds = []
        if self.xs is not None:
            xs = np.vstack([self.xs[1:], self.xs[-1:]])
            us = np.vstack([self.us[1:], self.us[-1:]])
            xs[:, :3] += x0[:3] - xs[0, :3]
            xs[:, 2] = x0[2] + np.arctan2(np.sin(xs[:, 2] - x0[2]), np.cos(xs[:, 2] - x0[2]))
            seeds.append((xs, us))
        self.count += 1
        distance = np.hypot(goal[0] - x0[0], goal[1] - x0[1])
        # Reseed on a cold start, or periodically while still far from the goal. In the final
        # approach the plan is kept: switching between near-equal forward and reversing plans
        # there made the robot circle the goal in MuJoCo (plant != model).
        if self.xs is None or (self.count % self.reseed_every == 0
                               and distance > self.final_approach):
            seeds.append(self.primitive_guess(x0, goal, True))
            if not forward_only:
                seeds.append(self.primitive_guess(x0, goal, False))
        if len(seeds) > 1:
            best = None
            s.options_set('max_iter', 4)           # screening: a few SQP steps per candidate
            for index, (xs, us) in enumerate(seeds):
                self._load(xs, us)
                s.solve_for_x0(x0, fail_on_nonzero_status=False, print_stats_on_failure=False)
                cost = s.get_cost() if s.status in (0, 2) else np.inf
                if index > 0 and self.xs is not None:
                    cost /= self.switch_margin     # hysteresis: keep the current plan
                if best is None or cost < best[0]:
                    best = (cost, self._read())
            s.options_set('max_iter', self.ocp.solver_options.nlp_solver_max_iter)
            self._load(*best[1])                # final solve below starts from the winner
        else:
            self._load(*seeds[0])
        u0 = s.solve_for_x0(x0, fail_on_nonzero_status=False, print_stats_on_failure=False)
        ms = (time.perf_counter() - t0) * 1e3
        status = s.status
        xs = np.array([s.get(k, 'x') for k in range(n + 1)])
        us = np.array([s.get(k, 'u') for k in range(n)])
        ok = status in (0, 2) and np.all(np.isfinite(us))   # 2 = max iterations (usable)
        if ok:
            self.xs, self.us = xs, us
        else:
            self.xs = self.us = None
        return {'u': tuple(np.clip(u0, -self.wmax, self.wmax)) if ok else (0.0, 0.0),
                'status': int(status), 'ok': bool(ok), 'xs': xs, 'us': us, 'ms': ms,
                'v_bound': vb, 'w_bound': wb}

    def _load(self, xs, us):
        for k in range(self.steps + 1):
            self.solver.set(k, 'x', xs[k])
        for k in range(self.steps):
            self.solver.set(k, 'u', us[k])

    def _read(self):
        s = self.solver
        return (np.array([s.get(k, 'x') for k in range(self.steps + 1)]),
                np.array([s.get(k, 'u') for k in range(self.steps)]))

    def primitive_guess(self, x0, goal, forward_only=False):
        """
        Build a guess: turn towards the goal, drive at full wheel speed, turn to its heading.

        Simulated through the wheel model over the horizon. A local NMPC cannot discover a
        turn-around manoeuvre by itself (from rest a goal straight behind, or at heading pi,
        is a stationary point of the cost), so cold starts are seeded with this
        minimum-time-like primitive and the solver only refines it. Driving backwards is used
        when allowed and the goal is behind.
        """
        from pinky_soccer_dynamics.model import rk4
        n, dt, wm = self.steps, self.dt, self.wmax
        spin_rate = (self.rl + self.rr) * wm / self.b
        x = np.asarray(x0, float).copy()
        xs, us = [x.copy()], []
        for _ in range(n):
            dx, dy = goal[0] - x[0], goal[1] - x[1]
            dist = np.hypot(dx, dy)
            sign = 1.0
            if dist > 0.03:
                aim = np.arctan2(dy, dx)
                err = np.arctan2(np.sin(aim - x[2]), np.cos(aim - x[2]))
                if not forward_only and abs(err) > np.pi / 2:
                    sign, err = -1.0, np.arctan2(np.sin(err + np.pi), np.cos(err + np.pi))
            else:
                err = np.arctan2(np.sin(goal[2] - x[2]), np.cos(goal[2] - x[2]))
            turn = np.clip(err / (spin_rate * dt), -1.0, 1.0)      # fraction of max spin
            if dist > 0.03 and abs(err) < 0.3:
                fwd = sign * min(1.0, dist / 0.15)
                u = wm * np.array([fwd - 0.5 * turn, fwd + 0.5 * turn])
            else:
                u = wm * np.array([-turn, turn])
            u = np.clip(u, -wm, wm)
            x = rk4(self.params, x, u, dt)
            xs.append(x.copy())
            us.append(u)
        return np.array(xs), np.array(us)

    def reset(self):
        """Forget the warm start (new goal far away or after a stop)."""
        self.xs = self.us = None


def step_wheels(params, omega, command, dt, amax):
    """Advance the wheel speed estimate (first-order lag + acceleration limit) by dt."""
    out = []
    for i, tau in enumerate((params.m('tau_left'), params.m('tau_right'))):
        o = omega[i]
        for _ in range(4):
            acc = np.clip((command[i] - o) / tau, -amax, amax)
            o += acc * dt / 4
        out.append(float(o))
    return tuple(out)
