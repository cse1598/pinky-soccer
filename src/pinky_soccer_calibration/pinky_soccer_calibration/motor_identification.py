"""Motor-response identification from saved motor_response experiments (offline, read-only).

Model (same map as pinky_soccer_dynamics.profile_model.motor_step / ramp_predictor):
    profile  r_{k+1} = r_k + clip(u(t - d) - r_k, -a h, a h)                 h = 5 ms
    wheel    w_{k+1} = e^{-h/tau} w_k + (1 - e^{-h/tau}) (r_k + r_{k+1}) / 2
a = profile ramp rate [rad/s^2], tau = first-order lag per wheel [s], d = effective input
delay [s]. Comparison models drop the ramp and/or the delay.

Timelines (never mixed):
  robot_clock  motor/diagnostics present: tagged driver receipts (Goal Velocity reconstructed
               from the received command exactly like the official driver), Velocity
               Trajectory and Present Velocity are all robot unix times -> network excluded;
               the stages goal->trajectory and trajectory->present are fitted separately.
  pc_receive   no diagnostics: PC command publish time -> joint_states PC receive time; the
               delay then contains Wi-Fi both ways, the watchdog hold (<= 50 ms) and the
               driver. Reported as such.
Raw files are only read. Parameters that the data cannot separate are flagged, not forced.
"""
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np

RPM_UNIT = .229                       # Dynamixel velocity unit [rpm]
RPM2RAD = 2 * math.pi / 60
MAX_WHEEL = 25.                       # |rad/s| above this is an invalid sample
STEP = .005                           # same substep as the MPC model
MODEL_VERSION = 'ramp_first_order_profile-v1 (pinky_soccer_dynamics.profile_model)'
DRIVER_DEFAULT = {'radius_m': .027, 'separation_m': .0961, 'max_rpm': 100.,
                  'source': 'official pinky_bringup defaults (launch overrides not recorded)'}
WHEEL_NAMES = (('l_wheel_joint', 'left_wheel_joint'), ('r_wheel_joint', 'right_wheel_joint'))


def driver_goal(v, w, driver=DRIVER_DEFAULT):
    """Goal wheel speeds [rad/s, both + forward] the official driver writes for (v, w)."""
    r, b = driver['radius_m'], driver['separation_m']
    rpm = np.array([(v - w * b / 2) / r, (v + w * b / 2) / r]) / RPM2RAD
    peak = np.max(np.abs(rpm))
    if peak > driver['max_rpm']:
        rpm = rpm * driver['max_rpm'] / peak
    units = np.trunc(rpm / RPM_UNIT)                    # int() in the driver: toward zero
    return units * RPM_UNIT * RPM2RAD


@dataclass
class Series:
    t: np.ndarray
    left: np.ndarray
    right: np.ndarray

    def wheel(self, i):
        return self.left if i == 0 else self.right


@dataclass
class Run:
    experiment_id: str
    purpose: str
    kind: str
    timeline: str
    goal: Series = None               # command reference for the model input
    trajectory: Series = None         # Velocity Trajectory (measured) if available
    present: Series = None            # Present Velocity (diag) or joint_states velocity
    extra: dict = field(default_factory=dict)
    quality: dict = field(default_factory=dict)


def _clean(t, left, right, quality, name):
    t, left, right = (np.asarray(v, float) for v in (t, left, right))
    ok = np.isfinite(t) & np.isfinite(left) & np.isfinite(right)
    ok &= (np.abs(left) <= MAX_WHEEL) & (np.abs(right) <= MAX_WHEEL)
    quality[name + '_invalid_samples'] = int((~ok).sum())
    t, left, right = t[ok], left[ok], right[ok]
    order = np.argsort(t, kind='stable')
    if np.any(np.diff(t) < 0):
        quality[name + '_out_of_order'] = True
    t, left, right = t[order], left[order], right[order]
    keep = np.r_[True, np.diff(t) > 0]
    quality[name + '_duplicate_times'] = int((~keep).sum())
    t, left, right = t[keep], left[keep], right[keep]
    if len(t) > 2:
        gaps = np.diff(t)
        med = float(np.median(gaps))
        quality[name + '_rate_hz'] = 1 / med if med > 0 else None
        quality[name + '_dropouts'] = int((gaps > 3 * med).sum())
    quality[name + '_samples'] = int(len(t))
    return Series(t, left, right)


def _sign(goal, meas):
    """Per-wheel sign that maps a measured series to the + forward convention."""
    if goal is None or meas is None or len(meas.t) < 3 or len(goal.t) < 2:
        return (1., 1.), 'default'
    out = []
    for i in (0, 1):
        g = np.interp(meas.t, goal.t, goal.wheel(i))
        dot = float(np.dot(g, meas.wheel(i)))
        out.append(-1. if dot < 0 else 1.)
    return tuple(out), 'inferred_from_command_correlation'


def load_run(folder, meta, driver=DRIVER_DEFAULT):
    """Read one motor_response experiment into model series (robot clock if possible)."""
    folder = Path(folder)
    quality = {}
    cmds = list(csv.DictReader((folder / 'commands.csv').open()))
    obs = list(csv.DictReader((folder / 'observations.csv').open()))
    pc_t = np.array([float(c['publish_monotonic']) for c in cmds])
    pc_goal = np.array([driver_goal(float(c['v_mps']), float(c['w_radps']), driver) for c in cmds])
    joints, diag, odom, imu, battery = [], [], [], [], []
    for row in obs:
        data = json.loads(row['observation_json'])
        t = float(row['receive_monotonic'])
        if row['topic'] == '/joint_states':
            names, vel = data.get('name') or [], data.get('velocity') or []
            idx = [next((names.index(a) for a in al if a in names), None) for al in WHEEL_NAMES]
            if None not in idx and max(idx) < len(vel):
                joints.append((t, vel[idx[0]], vel[idx[1]]))
        elif row['topic'] == '/motor/diagnostics':
            diag.append(data.get('payload') or {})
        elif row['topic'] == '/odom' and 'v' in data:
            odom.append((t, data['v'], data['w']))
        elif row['topic'] == '/imu_raw':
            imu.append((t, data.get('angular_velocity_z')))
        elif row['topic'] in ('/batt_state', '/battery/voltage'):
            battery.append(data.get('voltage', data.get('value')))
    receipts = [c for d in diag for c in d.get('commands', []) if float(c.get('tag') or 0) > 0]
    feedback = [d['feedback'] for d in diag if d.get('feedback')]
    quality['diag_read_failures'] = max([d.get('diag_read_failures', 0) for d in diag], default=0)
    run = Run(meta.get('id', folder.name), meta.get('purpose', 'identification'),
              meta.get('plan', {}).get('kind', '?'), 'pc_receive', quality=quality)
    run.extra = {'battery_voltage': battery[-1] if battery else None, 'driver': driver,
                 'odom': np.array(odom) if odom else None, 'imu': np.array(imu) if imu else None}
    if receipts and len(feedback) > 3:
        run.timeline = 'robot_clock'
        gt = [c['written_unix_ns'] * 1e-9 for c in receipts]
        gv = np.array([driver_goal(float(c['v']), float(c['w']), driver) for c in receipts])
        run.goal = _clean(gt, gv[:, 0], gv[:, 1], quality, 'goal')
        ft = [(f['read_start_unix_ns'] + f['read_done_unix_ns']) * 5e-10 for f in feedback]
        unit = RPM_UNIT * RPM2RAD
        for name, key in (('trajectory', 'velocity_trajectory'), ('present', 'present_velocity')):
            lv = [f['motors'][0][key] * unit for f in feedback]
            rv = [f['motors'][1][key] * unit for f in feedback]
            setattr(run, name, _clean(ft, lv, rv, quality, name))
        run.extra['current_raw'] = [[f['motors'][i]['present_current'] for i in (0, 1)] for f in feedback]
        run.extra['pwm_raw'] = [[f['motors'][i]['present_pwm'] for i in (0, 1)] for f in feedback]
        run.extra['input_voltage_v'] = [f['motors'][0]['present_input_voltage'] * .1 for f in feedback]
    else:
        run.goal = _clean(pc_t, pc_goal[:, 0], pc_goal[:, 1], quality, 'goal')
        if joints:
            j = np.array(joints)
            run.present = _clean(j[:, 0], j[:, 1], j[:, 2], quality, 'present')
        quality['delay_includes'] = 'Wi-Fi both ways + watchdog hold (<=50 ms) + driver'
    # Relative time origin: unix-sized absolute times (~1.8e9 s) leave float64 steps of
    # ~0.24 us, far coarser than optimizer perturbations of the delay.
    origin = float(run.goal.t[0]) if run.goal is not None and len(run.goal.t) else 0.
    run.extra['time_origin'] = origin
    for name in ('goal', 'trajectory', 'present'):
        series = getattr(run, name)
        if series is not None:
            series.t = series.t - origin
    for name in ('trajectory', 'present'):
        s = getattr(run, name)
        if s is not None:
            (sl, sr), how = _sign(run.goal, s)
            s.left, s.right = s.left * sl, s.right * sr
            quality[name + '_sign'] = {'left': sl, 'right': sr, 'method': how}
    return run


# ---------------------------------------------------------------- simulation
def simulate(goal_t, goal_u, t_out, tau, accel=None, delay=0., w0=0., r0=0., hold='zoh'):
    """Ramp(+)first-order response at t_out; accel None = no ramp; tau 0 = ramp only.

    hold='zoh' for commands (step signals); hold='linear' for a sampled continuous input such
    as the measured Velocity Trajectory (treating its 30 Hz samples as steps would add a
    half-sample lag that biases tau).
    """
    t0, t1 = min(goal_t[0], t_out[0]), t_out[-1]
    grid = np.arange(t0, t1 + STEP, STEP)
    # Input of each 5 ms step = exact mean of the delayed zero-order-hold command over the
    # step (integral of the step signal): continuous in the delay, so it can be estimated.
    goal_t, goal_u = np.asarray(goal_t, float), np.asarray(goal_u, float)
    te = np.r_[goal_t, goal_t[-1] + 1e3]
    cum = np.r_[0., np.cumsum(goal_u * np.diff(te))]
    lo = grid - delay
    if hold == 'linear':
        u = np.interp(lo + STEP / 2, goal_t, goal_u)
    else:
        u = (np.interp(lo + STEP, te, cum, left=0.) - np.interp(lo, te, cum, left=0.)) / STEP
    out, ref = np.empty_like(grid), np.empty_like(grid)
    w, r = w0, r0
    decay = math.exp(-STEP / tau) if tau > 0 else 0.
    da = accel * STEP if accel else None
    for k, uk in enumerate(u):
        nxt = r + max(-da, min(da, uk - r)) if da is not None else uk
        w = decay * w + (1 - decay) * (r + nxt) / 2 if tau > 0 else nxt
        r = nxt
        out[k], ref[k] = w, r
    return np.interp(t_out, grid, out), np.interp(t_out, grid, ref)


MODELS = {      # name -> (uses ramp, uses delay)
    'first_order': (False, False),
    'first_order_delay': (False, True),
    'ramp_first_order': (True, False),
    'ramp_first_order_delay': (True, True),       # the MPC model + effective input delay
}


def _unpack(model, p):
    ramp, delay = MODELS[model]
    out = {'tau_left': p[0], 'tau_right': p[1]}
    i = 2
    if ramp:
        out['motor_accel'] = p[i]
        i += 1
    if delay:
        out['delay'] = p[i]
    return out


def residuals(model, p, runs, source='goal', target='present'):
    q = _unpack(model, p)
    res = []
    for run in runs:
        u, y = getattr(run, source), getattr(run, target)
        if u is None or y is None or len(y.t) < 3:
            continue
        for i, tau in ((0, q['tau_left']), (1, q['tau_right'])):
            pred, _ = simulate(u.t, u.wheel(i), y.t, tau, q.get('motor_accel'), q.get('delay', 0.),
                               hold='linear' if source == 'trajectory' else 'zoh')
            res.append(pred - y.wheel(i))
    return np.concatenate(res) if res else np.zeros(0)


def _search(res_fn, names, lo, hi, starts):
    """Delay grid (5 ms, then 1 ms) x multistart least squares on the other parameters.

    Sparse 20-30 Hz samples make the cost surface rugged in (ramp rate, delay); a local
    solver from one start can stop in a wrong valley, so the delay is searched explicitly.
    """
    from scipy.optimize import least_squares
    has_delay = names[-1] == 'delay'
    k = len(names) - (1 if has_delay else 0)

    def solve(delay):
        best = None
        for x0 in starts:
            f = (lambda p: res_fn(np.r_[p, delay])) if has_delay else res_fn
            sol = least_squares(f, x0[:k], bounds=(lo[:k], hi[:k]),
                                x_scale=np.abs(np.array(x0[:k])) + .01)
            if best is None or sol.cost < best.cost:
                best = sol
        return best
    if not has_delay:
        return solve(None).x
    coarse = np.arange(lo[-1], hi[-1] + 1e-9, .01)
    costs = [(solve(d).cost, d) for d in coarse]
    d0 = min(costs)[1]
    fine = np.arange(max(lo[-1], d0 - .01), min(hi[-1], d0 + .01) + 1e-9, .002)
    best = min(((solve(d), d) for d in fine), key=lambda v: v[0].cost)
    return np.r_[best[0].x, best[1]]


def fit(model, runs, source='goal', target='present', accel0=74.9):
    """Least squares with identifiability diagnostics (std, correlation, bound hits)."""
    from scipy.optimize import least_squares
    ramp, delay = MODELS[model]
    x0, lo, hi, names = [.05, .05], [.003, .003], [.6, .6], ['tau_left', 'tau_right']
    if ramp:
        x0.append(accel0), lo.append(5.), hi.append(400.), names.append('motor_accel')
    if delay:
        x0.append(.02), lo.append(0.), hi.append(.15), names.append('delay')
    if residuals(model, np.array(x0), runs, source, target).size < 3 * len(x0):
        return {'model': model, 'status': 'insufficient_data'}

    def res_fn(p):
        return residuals(model, p, runs, source, target)
    starts = [x0] + ([[.05, .05, 150.] + x0[3:]] if ramp else [])
    start = _search(res_fn, names, lo, hi, starts)
    # Final joint refinement (no grid) for the covariance / identifiability diagnostics.
    sol = least_squares(res_fn, start, bounds=(lo, hi), x_scale=np.abs(start) + .01,
                        diff_step=1e-3)
    if sol.cost > .5 * float(res_fn(start) @ res_fn(start)) + 1e-12:
        sol.x = start
        sol.fun = res_fn(start)
    r = sol.fun
    dof = max(1, r.size - len(x0))
    s2 = float(r @ r) / dof
    out = {'model': model, 'status': 'fitted', 'samples': int(r.size),
           'rmse': math.sqrt(float(r @ r) / r.size), 'max_abs_error': float(np.max(np.abs(r))),
           'parameters': dict(zip(names, map(float, sol.x)))}
    jtj = sol.jac.T @ sol.jac
    flags = {}
    try:
        cov = np.linalg.inv(jtj) * s2
        std = np.sqrt(np.clip(np.diag(cov), 0, None))
        corr = cov / np.outer(std, std)
        out['std'] = dict(zip(names, map(float, std)))
        out['condition_number'] = float(np.linalg.cond(jtj))
        for i, n in enumerate(names):
            rel = std[i] / max(abs(sol.x[i]), 1e-9)
            worst = max((abs(corr[i, j]) for j in range(len(names)) if j != i), default=0.)
            at_bound = (sol.x[i] <= lo[i] + 1e-9 and lo[i] > 0) or sol.x[i] >= hi[i] * .999
            flags[n] = ('identification_insufficient' if rel > .25 or worst > .95 or at_bound
                        else 'estimated')
            if at_bound:
                flags[n] += ' (at bound)'
        out['correlation'] = {f'{names[i]}~{names[j]}': float(corr[i, j])
                              for i in range(len(names)) for j in range(i + 1, len(names))}
    except np.linalg.LinAlgError:
        flags = {n: 'identification_insufficient (singular)' for n in names}
    out['parameter_status'] = flags
    out['aic'] = r.size * math.log(max(float(r @ r) / r.size, 1e-12)) + 2 * len(names)
    return out


def evaluate(model, params, runs, source='goal', target='present', hold_tail=.5):
    """Per-run metrics of a fitted model on (independent) runs."""
    ramp, delay = MODELS[model]
    rows = []
    for run in runs:
        u, y = getattr(run, source), getattr(run, target)
        if u is None or y is None or len(y.t) < 3:
            continue
        for i, side in ((0, 'left'), (1, 'right')):
            pred, _ = simulate(u.t, u.wheel(i), y.t, params[f'tau_{side}'],
                               params.get('motor_accel') if ramp else None,
                               params.get('delay', 0.) if delay else 0.)
            err = pred - y.wheel(i)
            amp = max(float(np.max(np.abs(u.wheel(i)))), 1e-6)
            moving = np.flatnonzero(np.abs(np.interp(y.t, u.t, u.wheel(i))) > 1e-6)
            steady = None
            if len(moving):
                end = y.t[moving[-1]]
                tail = (y.t > end - hold_tail) & (y.t <= end)
                steady = float(np.mean(err[tail])) if tail.any() else None
            rows.append({'experiment_id': run.experiment_id, 'wheel': side,
                         'rmse': float(np.sqrt(np.mean(err ** 2))),
                         'relative_rmse': float(np.sqrt(np.mean(err ** 2)) / amp),
                         'max_abs_error': float(np.max(np.abs(err))),
                         'steady_state_error': steady,
                         'initial_error': float(np.mean(np.abs(err[:max(3, len(err) // 10)])))})
    return rows


def identify(runs, models=tuple(MODELS)):
    """Fit every model on identification runs, evaluate on independent validation runs."""
    ident = [r for r in runs if r.purpose != 'validation']
    valid = [r for r in runs if r.purpose == 'validation']
    robot = [r for r in ident if r.timeline == 'robot_clock' and r.trajectory is not None]
    result = {'model_version': MODEL_VERSION,
              'identification_experiments': [r.experiment_id for r in ident],
              'validation_experiments': [r.experiment_id for r in valid],
              'timelines': sorted({r.timeline for r in runs}), 'models': {}, 'stages': {}}
    for model in models:
        fitted = fit(model, ident)
        if fitted['status'] == 'fitted' and valid:
            fitted['validation'] = evaluate(model, fitted['parameters'], valid)
        result['models'][model] = fitted
    if robot:   # stage-wise with measured Velocity Trajectory (robot clock only)
        ramp = fit_ramp(robot)
        lag = fit('first_order_delay', robot, source='trajectory', target='present')
        result['stages'] = {'goal_to_trajectory': ramp, 'trajectory_to_present': lag}
    ok = [m for m in result['models'].values() if m.get('status') == 'fitted']
    if valid and ok:
        key = (lambda m: np.mean([v['rmse'] for v in m.get('validation', [])]) if m.get('validation')
               else math.inf)
        result['best_on_validation'] = min(ok, key=key)['model']
    result['validated'] = bool(valid) and any(m.get('validation') for m in ok)
    if not valid:
        result['validation_note'] = 'no independent validation runs: results are unvalidated'
    return result


def fit_ramp(runs):
    """Goal -> Velocity Trajectory: ramp rate and delay (no lag); same delay search."""
    def res(p):
        out = []
        for run in runs:
            for i in (0, 1):
                _, ref = simulate(run.goal.t, run.goal.wheel(i), run.trajectory.t, .05,
                                  p[0], p[1])           # the profile does not depend on tau
                out.append(ref - run.trajectory.wheel(i))
        return np.concatenate(out)
    x = _search(res, ['motor_accel', 'delay'], [5., 0.], [400., .15],
                [[74.9, .02], [150., .02]])
    r = res(x)
    return {'status': 'fitted', 'parameters': {'motor_accel': float(x[0]), 'delay': float(x[1])},
            'rmse': math.sqrt(float(r @ r) / max(r.size, 1)), 'samples': int(r.size),
            'note': 'Velocity Trajectory is the motor profile target (measured, robot clock)'}
