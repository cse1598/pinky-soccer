"""Motor-response identification from saved motor_response experiments (offline, read-only).

Model (the physics of pinky_soccer_dynamics.profile_model / ramp_predictor):
    profile  dr/dt = clip-to-slope(u(t - d) - r, a)     (Dynamixel Profile Acceleration)
    wheel    tau dw/dt = r - w
a = profile ramp rate [rad/s^2], tau = first-order lag per wheel [s], d = effective input
delay [s]. Comparison models drop the ramp and/or the delay. Fitting uses the exact
continuous-time solution (simulate); the MPC integrates the same equations with a 5 ms map
(simulate_map), whose difference is reported separately. The 5 ms step is an integration
step, never a measurement resolution: data are used only at their recorded sample times.

Data selection per experiment (one physical sample is used once):
  feedback  1 motor/diagnostics feedback block (robot clock, serial read start/done,
              Present Velocity + Velocity Trajectory + Position, current/PWM/voltage)
            2 /wheel_joint_states_raw (the driver's own JointState: same registers, header
              stamp taken by the driver just before the serial read; velocity + position)
            3 /joint_states (joint_state_publisher re-samples the relay at 20 Hz with its
              own stamps and holds old values: held repeats are dropped)
  goal      driver receipts in motor/diagnostics (robot clock, after the serial write) or
            the PC command log (PC clock; robot stamps are then only combined with it when
            the clock check was verified for the run).
Timelines (never mixed):
  robot_clock  driver receipts + diagnostics feedback, both robot unix time: network
               excluded; delay = serial write done -> modelled ramp start.
  synced_clock PC publish unix time + robot header stamps (verified clock only): delay
               includes Wi-Fi, the watchdog (forwards the latest command every 50 ms) and
               the driver.
  pc_receive   PC publish -> PC receive (monotonic): delay additionally includes the
               feedback's way back over Wi-Fi and the receive scheduling.
Raw files are only read. Parameters that the data cannot separate are flagged, not forced.
"""
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np

from . import drive_metadata as dm

RPM_UNIT = dm.RPM_UNIT
RPM2RAD = dm.RPM2RAD
UNIT_RAD = dm.UNIT_RAD
PULSE_RAD = 2 * math.pi / 4096        # Present Position unit
MAX_WHEEL = 25.                       # |rad/s| above this is an invalid sample
STEP = .005                           # the MPC integration substep (simulate_map only)
MODEL_VERSION = 'ramp_first_order_continuous-v2 (physics of pinky_soccer_dynamics.profile_model)'
DRIVER_DEFAULT = {'radius_m': .027, 'separation_m': .0961, 'max_rpm': 100.,
                  'source': 'assumed_default (official pinky_params.yaml)'}
WHEEL_NAMES = (('l_wheel_joint', 'left_wheel_joint'), ('r_wheel_joint', 'right_wheel_joint'))
FEEDBACK_TOPICS = ('/motor/diagnostics', '/wheel_joint_states_raw', '/joint_states')


def driver_goal(v, w, driver=DRIVER_DEFAULT):
    """Goal wheel speeds [rad/s, both + forward] the official driver writes for (v, w)."""
    r, b = driver['radius_m'], driver['separation_m']
    rpm = np.array([(v - w * b / 2) / r, (v + w * b / 2) / r]) / RPM2RAD
    peak = np.max(np.abs(rpm))
    if peak > driver['max_rpm']:
        rpm = rpm * driver['max_rpm'] / peak
    if not driver.get('quantize', True):          # simulators: no register quantisation
        return rpm * RPM2RAD
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
    present: Series = None            # measured wheel speed (selected source)
    position: Series = None           # measured wheel angle [rad, + forward] if available
    extra: dict = field(default_factory=dict)
    quality: dict = field(default_factory=dict)
    sources: dict = field(default_factory=dict)


def _clean(t, left, right, quality, name, limit=MAX_WHEEL):
    t, left, right = (np.asarray(v, float) for v in (t, left, right))
    ok = np.isfinite(t) & np.isfinite(left) & np.isfinite(right)
    if limit is not None:
        ok &= (np.abs(left) <= limit) & (np.abs(right) <= limit)
    quality[name + '_invalid_samples'] = int((~ok).sum())
    t, left, right = t[ok], left[ok], right[ok]
    if np.any(np.diff(t) < 0):
        quality[name + '_out_of_order'] = True
    order = np.argsort(t, kind='stable')
    t, left, right = t[order], left[order], right[order]
    keep = np.r_[True, np.diff(t) > 0]
    quality[name + '_duplicate_times'] = int((~keep).sum())
    t, left, right = t[keep], left[keep], right[keep]
    if len(t) > 2:
        gaps = np.diff(t)
        med = float(np.median(gaps))
        quality[name + '_rate_hz'] = 1 / med if med > 0 else None
        quality[name + '_max_gap_s'] = float(gaps.max())
        quality[name + '_dropouts'] = int((gaps > 1.8 * med).sum())
    quality[name + '_samples'] = int(len(t))
    return Series(t, left, right)


def _sign(goal, meas):
    """Per-wheel sign that maps a measured series to the + forward convention."""
    if goal is None or meas is None or len(meas.t) < 3 or len(goal.t) < 2:
        return None
    out = []
    for i in (0, 1):
        g = np.interp(meas.t, goal.t, goal.wheel(i))
        dot = float(np.dot(g, meas.wheel(i)))
        out.append(0. if abs(dot) < 1e-9 else (-1. if dot < 0 else 1.))
    return tuple(out)


def _joint_rows(obs, topic):
    """(t_receive, t_header, pos_l, pos_r, vel_l, vel_r) for one JointState topic."""
    rows = []
    for row in obs:
        if row['topic'] != topic:
            continue
        data = json.loads(row['observation_json'])
        names = data.get('name') or []
        pos, vel = data.get('position') or [], data.get('velocity') or []
        idx = [next((names.index(a) for a in al if a in names), None) for al in WHEEL_NAMES]
        if None in idx:
            continue
        p = [pos[i] if i < len(pos) else math.nan for i in idx]
        v = [vel[i] if i < len(vel) else math.nan for i in idx]
        try:
            header = float(row.get('header_seconds') or 'nan')
        except ValueError:
            header = math.nan
        rows.append((float(row['receive_monotonic']), header, *p, *v))
    return np.array(rows) if rows else None


def _read(folder):
    cmds = list(csv.DictReader((folder / 'commands.csv').open()))
    obs = list(csv.DictReader((folder / 'observations.csv').open()))
    return cmds, obs


def _diag_payloads(obs):
    out = []
    for row in obs:
        if row['topic'] == '/motor/diagnostics':
            payload = json.loads(row['observation_json']).get('payload')
            if isinstance(payload, dict):
                out.append(payload)
    return out


def steady_segments(receipts, feedback, min_samples=3):
    """(v, w, units_l, units_r) where the trajectory stayed constant after a command."""
    out = []
    if not receipts or not feedback:
        return out
    ft = np.array([f['read_start_unix_ns'] for f in feedback], float)
    tr = np.array([[f['motors'][0]['velocity_trajectory'], f['motors'][1]['velocity_trajectory']]
                   for f in feedback], float)
    times = [c['written_unix_ns'] for c in receipts]
    for j, c in enumerate(receipts):
        if j and (receipts[j - 1]['v'], receipts[j - 1]['w']) == (c['v'], c['w']):
            continue
        end = next((times[k] for k in range(j + 1, len(receipts))
                    if (receipts[k]['v'], receipts[k]['w']) != (c['v'], c['w'])), math.inf)
        sel = tr[(ft > times[j]) & (ft < end)]
        if len(sel) < min_samples:
            continue
        tail = sel[-min_samples:]
        if np.all(tail == tail[0]):
            out.append((float(c['v']), float(c['w']), int(tail[0, 0]), int(tail[0, 1])))
    return out


def load_run(folder, meta, driver=None, override=None):
    """Read one motor_response experiment (raw files are only read).

    driver: explicit driver values (tests, simulators); otherwise resolved from the stored
    drive configuration, an approved override, the register data and finally the default.
    """
    folder = Path(folder)
    quality, sources = {}, {}
    cmds, obs = _read(folder)
    plan = meta.get('plan', {})
    run = Run(meta.get('id', folder.name), meta.get('purpose', 'identification'),
              plan.get('kind', '?'), 'pc_receive', quality=quality, sources=sources)
    diag = _diag_payloads(obs)
    # one physical feedback sample = one serial read: drop repeated blocks
    seen, feedback = set(), []
    for d in diag:
        f = d.get('feedback')
        if not f or not isinstance(f.get('motors'), list) or len(f['motors']) < 2:
            continue
        key = f.get('read_start_unix_ns')
        if key in seen:
            quality['diagnostics_repeated_blocks'] = quality.get('diagnostics_repeated_blocks', 0) + 1
            continue
        seen.add(key)
        feedback.append(f)
    quality['diag_read_failures'] = max([d.get('diag_read_failures', 0) for d in diag], default=0)
    quality['diagnostics_messages'] = len(diag)
    quality['diagnostics_without_feedback'] = sum(1 for d in diag if not d.get('feedback'))
    receipts = [c for d in diag for c in d.get('commands', []) if float(c.get('tag') or 0) > 0]
    raw = _joint_rows(obs, '/wheel_joint_states_raw')
    joints = _joint_rows(obs, '/joint_states')
    available = {'/motor/diagnostics': len(feedback),
                 '/wheel_joint_states_raw': 0 if raw is None else len(raw),
                 '/joint_states': 0 if joints is None else len(joints)}
    sources['available_feedback'] = available
    sync = bool((meta.get('clock_sync') or {}).get('relative_verified')
                and (meta.get('clock_sync_end') or meta.get('clock_sync') or {}).get('relative_verified'))
    # ---- driver values used to rebuild the Goal Velocity from (v, w)
    inferred = dm.infer_geometry(steady_segments(receipts, feedback))
    if driver is None:
        stored = (meta.get('drive_configuration') or {})
        res = dm.resolve(stored, override, inferred)
        driver = {'radius_m': res['wheel_radius']['value'],
                  'separation_m': res['wheel_separation']['value'], 'max_rpm': dm.OFFICIAL['max_rpm']}
        sources['driver_values'] = res
    else:
        sources['driver_values'] = {'explicit': dict(driver)}
    sources['driver_inferred'] = inferred
    # ---- goal
    if receipts and len(feedback) > 3:
        run.timeline = 'robot_clock'
        gt = [c['written_unix_ns'] * 1e-9 for c in receipts]
        gv = np.array([driver_goal(float(c['v']), float(c['w']), driver) for c in receipts])
        run.goal = _clean(gt, gv[:, 0], gv[:, 1], quality, 'goal')
        sources['goal'] = 'motor/diagnostics driver receipts (serial write done, robot clock)'
    else:
        use_unix = sync and raw is not None and len(raw) > 3 and cmds and cmds[0].get('publish_unix_ns')
        key = 'publish_unix_ns' if use_unix else 'publish_monotonic'
        scale = 1e-9 if use_unix else 1.
        gt = [float(c[key]) * scale for c in cmds]
        gv = np.array([driver_goal(float(c['v_mps']), float(c['w_radps']), driver) for c in cmds])             if cmds else np.zeros((0, 2))
        run.goal = _clean(gt, gv[:, 0], gv[:, 1], quality, 'goal') if cmds else None
        run.timeline = 'synced_clock' if use_unix else 'pc_receive'
        sources['goal'] = f'PC command log ({key})'
    # ---- feedback (single source)
    if run.timeline == 'robot_clock':
        ft = [(f['read_start_unix_ns'] + f['read_done_unix_ns']) * 5e-10 for f in feedback]
        for name, key in (('trajectory', 'velocity_trajectory'), ('present', 'present_velocity')):
            lv = [f['motors'][0][key] * UNIT_RAD for f in feedback]
            rv = [f['motors'][1][key] * UNIT_RAD for f in feedback]
            setattr(run, name, _clean(ft, lv, rv, quality, name))
        run.position = _clean(ft, [f['motors'][0]['present_position'] * PULSE_RAD for f in feedback],
                              [f['motors'][1]['present_position'] * PULSE_RAD for f in feedback],
                              quality, 'position', limit=None)
        read = np.array([(f['read_done_unix_ns'] - f['read_start_unix_ns']) * 1e-9 for f in feedback])
        quality['serial_read_duration_s'] = {'median': float(np.median(read)), 'max': float(read.max())}
        run.extra['current_raw'] = [[f['motors'][i].get('present_current') for i in (0, 1)] for f in feedback]
        run.extra['pwm_raw'] = [[f['motors'][i].get('present_pwm') for i in (0, 1)] for f in feedback]
        run.extra['input_voltage_v'] = [f['motors'][0].get('present_input_voltage', 0) * .1 for f in feedback]
        sources['present'] = ('/motor/diagnostics Present Velocity (read midpoint, robot clock); '
                              'same registers as /wheel_joint_states_raw, which is not used again')
    elif raw is not None and len(raw) > 3:
        col = 1 if run.timeline == 'synced_clock' else 0
        run.present = _clean(raw[:, col], raw[:, 4], raw[:, 5], quality, 'present')
        run.position = _clean(raw[:, col], raw[:, 2], raw[:, 3], quality, 'position', limit=None)
        sources['present'] = ('/wheel_joint_states_raw velocity (' + (
            'driver header stamp = robot clock just before the serial read' if col else
            'PC receive time') + ')')
    elif joints is not None and len(joints) > 3:
        # joint_state_publisher re-publishes the last relay value at 20 Hz: a repeated
        # (position, velocity) is the same physical sample, keep its first copy only.
        rep = np.r_[False, np.all(np.diff(joints[:, 2:6], axis=0) == 0, axis=1)]
        quality['joint_states_held_repeats'] = int(rep.sum())
        j = joints[~rep]
        run.present = _clean(j[:, 0], j[:, 4], j[:, 5], quality, 'present')
        run.position = _clean(j[:, 0], j[:, 2], j[:, 3], quality, 'position', limit=None)
        sources['present'] = ('/joint_states (20 Hz re-sample with its own stamps; PC receive '
                              'time; held repeats dropped) - lowest priority')
        run.timeline = 'pc_receive'
        if run.goal is not None and sources['goal'].endswith('(publish_unix_ns)'):
            gt = [float(c['publish_monotonic']) for c in cmds]
            run.goal = _clean(gt, gv[:, 0], gv[:, 1], quality, 'goal')
            sources['goal'] = 'PC command log (publish_monotonic)'
    else:
        sources['present'] = 'none: no usable wheel feedback in this experiment'
    if run.timeline != 'robot_clock':
        quality['delay_includes'] = ('Wi-Fi + watchdog forward (<= 50 ms) + driver' if
                                     run.timeline == 'synced_clock' else
                                     'Wi-Fi both ways + watchdog (<= 50 ms) + driver + receive')
    # ---- odom, IMU, battery (context only)
    odom, imu, battery = [], [], []
    for row in obs:
        data = json.loads(row['observation_json'])
        t = float(row['receive_monotonic'])
        if row['topic'] == '/odom' and 'v' in data:
            odom.append((t, data['v'], data['w'], data.get('x'), data.get('y'), data.get('yaw')))
        elif row['topic'] == '/imu_raw':
            imu.append((t, data.get('angular_velocity_z')))
        elif row['topic'] in ('/batt_state', '/battery/voltage'):
            battery.append(data.get('voltage', data.get('value')))
    run.extra.update(battery_voltage=battery[-1] if battery else None, driver=driver,
                     odom=np.array(odom, float) if odom else None,
                     imu=np.array(imu, float) if imu else None, clock_verified=sync)
    # Relative time origin: unix-sized absolute times (~1.8e9 s) leave float64 steps of
    # ~0.24 us, far coarser than optimizer perturbations of the delay.
    origin = float(run.goal.t[0]) if run.goal is not None and len(run.goal.t) else 0.
    run.extra['time_origin'] = origin
    for name in ('goal', 'trajectory', 'present', 'position'):
        series = getattr(run, name)
        if series is not None:
            series.t = series.t - origin
    # ---- sign convention: official code (left +, right raw -), checked against the data.
    # A wheel whose measured speed anti-correlates with its command under that convention
    # (e.g. a simulator with + forward joints) uses the measured sign, and says so.
    signs = [dm.ENCODER_SIGNS['left'], dm.ENCODER_SIGNS['right']]
    info = {'convention': 'code_analysis (official driver: right raw -)'}
    ref = run.present if run.present is not None else run.trajectory
    if ref is not None:
        check = _sign(run.goal, Series(ref.t, ref.left * signs[0], ref.right * signs[1]))
        if check is None or 0. in check:
            info['data_check'] = 'not checkable (no motion)'
        elif all(c > 0 for c in check):
            info['data_check'] = 'consistent'
        else:
            signs = [sg * c for sg, c in zip(signs, check)]
            info['data_check'] = ('CONTRADICTED by command correlation; measured sign used '
                                  f'(left {signs[0]:+.0f}, right {signs[1]:+.0f})')
    info.update(left=signs[0], right=signs[1])
    quality['sign'] = info
    for name in ('trajectory', 'present', 'position'):
        series = getattr(run, name)
        if series is not None:
            series.left, series.right = series.left * signs[0], series.right * signs[1]
    if run.position is not None and len(run.position.t):
        run.position.left = run.position.left - run.position.left[0]
        run.position.right = run.position.right - run.position.right[0]
    return run


# ---------------------------------------------------------------- simulation
def simulate_map(goal_t, goal_u, t_out, tau, accel=None, delay=0., w0=0., r0=0., hold='zoh'):
    """The MPC's 5 ms numerical map (profile_model.motor_step) evaluated at t_out.

    Kept to quantify the MPC discretisation against the continuous model (simulate). Not
    used for fitting: with a saturated ramp the 5 ms map cannot tell delays inside one
    step apart (the clipped increment is the same), so delays would be quantised to 5 ms.

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
        # Sampled reference state (measured Velocity Trajectory): value at the step end.
        u = np.interp(lo + STEP, goal_t, goal_u)
    else:
        u = (np.interp(lo + STEP, te, cum, left=0.) - np.interp(lo, te, cum, left=0.)) / STEP
    out, ref = np.empty_like(grid), np.empty_like(grid)
    w, r = w0, r0
    decay = math.exp(-STEP / tau) if tau > 0 else 0.
    da = accel * STEP if accel else None
    exact_zoh = da is None and hold != 'linear'
    for k, uk in enumerate(u):
        nxt = r + max(-da, min(da, uk - r)) if da is not None else uk
        if tau <= 0:
            w = nxt
        elif exact_zoh:     # no ramp, step command held over the step: exact discretisation
            w = decay * w + (1 - decay) * uk
        else:               # the MPC map: reference states at both step ends (trapezoid)
            w = decay * w + (1 - decay) * (r + nxt) / 2
        r = nxt
        out[k], ref[k] = w, r
    # State after step k belongs to grid[k] + STEP (the MPC map: x_{k+1} = f(x_k, u_k)).
    # Storing it at grid[k] would predict one step (5 ms) early and bias the delay.
    times = np.r_[grid[0], grid + STEP]
    return (np.interp(t_out, times, np.r_[w0, out]), np.interp(t_out, times, np.r_[r0, ref]))


def _ref_knots(goal_t, goal_u, accel, delay, r0, hold, t_end):
    """Reference (profile output) as a piecewise-linear function: (times, values)."""
    gt = np.asarray(goal_t, float) + delay
    gu = np.asarray(goal_u, float)
    if hold == 'linear':                # sampled reference state, linear between samples
        return np.r_[gt[0] - 1e3, gt, max(t_end, gt[-1]) + 1e3], np.r_[gu[0], gu, gu[-1]]
    # Delayed zero-order-hold command; before the first command the reference rests at r0.
    starts = np.r_[gt, max(t_end, gt[-1]) + 1e3]
    times, vals = [gt[0] - 1e3, gt[0]], [r0, r0]
    r = r0
    for j, c in enumerate(gu):
        s, e = starts[j], starts[j + 1]
        if e <= s:
            continue
        if accel is None:           # a jump: the new value starts 1 ns later (strict knots)
            times += [s + 1e-9, e]
            vals += [c, c]
            r = c
            continue
        reach = s + abs(c - r) / accel
        if reach < e:
            times += [reach, e]
            vals += [c, c]
            r = c
        else:
            r = r + math.copysign(accel * (e - s), c - r)
            times.append(e)
            vals.append(r)
    return np.asarray(times), np.asarray(vals)


def simulate(goal_t, goal_u, t_out, tau, accel=None, delay=0., w0=0., r0=0., hold='zoh',
             t_start=None):
    """Continuous-time ramp(+)first-order response at t_out (exact, used for fitting).

    Reference: the delayed zero-order-hold command limited to slope accel (Dynamixel
    profile), or for hold='linear' the sampled reference itself (measured Velocity
    Trajectory). Wheel: tau dw/dt = ref - w, solved exactly on every linear piece. Same
    physics as the MPC model; the MPC integrates it with a 5 ms map (simulate_map).
    """
    t_out = np.asarray(t_out, float)
    if len(goal_t) == 0 or len(t_out) == 0:
        return np.zeros_like(t_out), np.zeros_like(t_out)
    kt, kv = _ref_knots(goal_t, goal_u, accel, delay, r0, hold, float(t_out[-1]))
    if t_start is not None:
        if hold == 'linear':
            r_start = float(np.interp(t_start, kt, kv))
        else:
            r_start = r0
            # rebuild from the measured reference at t_start under the command then active
            gt = np.asarray(goal_t, float)
            j = int(np.searchsorted(gt + delay, t_start, side='right')) - 1
            gt2 = np.r_[t_start - delay, gt[j + 1:]] if j >= 0 else np.r_[t_start - delay, gt]
            gu2 = np.r_[np.asarray(goal_u, float)[max(j, 0)] if j >= 0 else r0,
                        np.asarray(goal_u, float)[j + 1:] if j >= 0 else np.asarray(goal_u, float)]
            kt, kv = _ref_knots(gt2, gu2, accel, delay, r_start, hold, float(t_out[-1]))
        keep = kt > t_start
        kt, kv = np.r_[t_start, kt[keep]], np.r_[r_start, kv[keep]]
    keep = np.r_[True, np.diff(kt) > 0]
    kt, kv = kt[keep], kv[keep]
    ref_out = np.interp(t_out, kt, kv)
    if tau <= 0:
        return ref_out, ref_out
    # wheel state at every knot, then the exact solution from the knot before each output
    dt = np.diff(kt)
    slope = np.diff(kv) / dt
    e = np.exp(-dt / tau)
    wk = np.empty_like(kt)
    wk[0] = w0
    for i in range(len(dt)):            # w(t) = u(t) - c1 tau + (w_s - u_s + c1 tau) e^{-(t-s)/tau}
        wk[i + 1] = kv[i + 1] - slope[i] * tau + (wk[i] - kv[i] + slope[i] * tau) * e[i]
    i = np.clip(np.searchsorted(kt, t_out, side='right') - 1, 0, len(dt) - 1)
    x = t_out - kt[i]
    pred = (kv[i] + slope[i] * x - slope[i] * tau
            + (wk[i] - kv[i] + slope[i] * tau) * np.exp(-x / tau))
    return pred, ref_out


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
    """Delay grid (5 ms, then 1 ms; the continuous model makes the cost smooth in it) x multistart least squares on the other parameters.

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
    coarse = np.arange(lo[-1], hi[-1] + 1e-9, .005)
    costs = [(solve(d).cost, d) for d in coarse]
    d0 = min(costs)[1]
    fine = np.arange(max(lo[-1], d0 - .005), min(hi[-1], d0 + .005) + 1e-9, .001)
    best = min(((solve(d), d) for d in fine), key=lambda v: v[0].cost)
    return np.r_[best[0].x, best[1]]


def _diagnose(sol, names, lo, hi):
    """RMSE, local covariance, 95 % local intervals and identifiability flags."""
    r = sol.fun
    dof = max(1, r.size - len(names))
    s2 = float(r @ r) / dof
    out = {'status': 'fitted', 'samples': int(r.size),
           'rmse': math.sqrt(float(r @ r) / r.size), 'max_abs_error': float(np.max(np.abs(r))),
           'parameters': dict(zip(names, map(float, sol.x)))}
    jtj = sol.jac.T @ sol.jac
    flags = {}
    try:
        cov = np.linalg.inv(jtj) * s2
        std = np.sqrt(np.clip(np.diag(cov), 0, None))
        corr = cov / np.outer(std, std)
        out['std'] = dict(zip(names, map(float, std)))
        # local (linearised) 95 % interval: valid only near the optimum and for the model
        # structure; experiment-level bootstrap is reported separately (uncertainty())
        out['ci95_local'] = {n: [float(sol.x[i] - 1.96 * std[i]), float(sol.x[i] + 1.96 * std[i])]
                             for i, n in enumerate(names)}
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
    return out


def fit(model, runs, source='goal', target='present', accel0=74.9, starts=None):
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
    if starts is None:
        starts = [x0] + ([[.05, .05, 150.] + x0[3:]] if ramp else [])
    start = _search(res_fn, names, lo, hi, starts)
    # Final joint refinement (no grid) for the covariance / identifiability diagnostics.
    sol = least_squares(res_fn, start, bounds=(lo, hi), x_scale=np.abs(start) + .01,
                        diff_step=1e-3)
    if sol.cost > .5 * float(res_fn(start) @ res_fn(start)) + 1e-12:
        sol.x = start
        sol.fun = res_fn(start)
    out = {'model': model, **_diagnose(sol, names, lo, hi)}
    r = sol.fun
    out['aic'] = r.size * math.log(max(float(r @ r) / r.size, 1e-12)) + 2 * len(names)
    out['bic'] = (r.size * math.log(max(float(r @ r) / r.size, 1e-12))
                  + len(names) * math.log(max(r.size, 2)))
    out['experiments'] = [run.experiment_id for run in runs]
    return out


HORIZONS = (.1, .25, .5, 1., 2.)     # s; 2.0 s = current MPC horizon (20 x 0.1 s)


def structure(name):
    """Model structure of a result key ('..._stagewise' = same model, stage-wise estimate)."""
    return name[:-len('_stagewise')] if name.endswith('_stagewise') else name


def _params_for(model, params, i):
    ramp, delay = MODELS[structure(model)]
    return (params['tau_left' if i == 0 else 'tau_right'], params.get('motor_accel') if ramp else None,
            params.get('delay', 0.) if delay else 0.)


def _phases(u, i, t, window=.4, hold_tail=.5):
    """Masks of measured samples: after a rising / falling command change, steady tails."""
    ut, uu = u.t, u.wheel(i)
    rise, fall, steady = (np.zeros(len(t), bool) for _ in range(3))
    changes = [k for k in range(1, len(uu)) if uu[k] != uu[k - 1]]
    for k in changes:
        win = (t >= ut[k]) & (t < ut[k] + window)
        (rise if abs(uu[k]) > abs(uu[k - 1]) else fall)[win] = True
        if abs(uu[k - 1]) > 1e-6:
            steady |= (t > ut[k] - hold_tail) & (t <= ut[k])
    return rise, fall, steady


def _stats(e):
    e = np.abs(np.asarray(e, float))
    return None if not e.size else {'mean': float(e.mean()), 'p95': float(np.percentile(e, 95)),
                                    'max': float(e.max())}


def horizon_errors(model, params, run, source='goal', horizons=HORIZONS, every=.1):
    """Predict from measured states over MPC-like horizons; wheel angle and end-speed errors."""
    y, pos, ref = run.present, run.position, run.trajectory
    u = getattr(run, source)
    if y is None or u is None or len(y.t) < 5:
        return {}
    out = {}
    for h in horizons:
        ang, spd = [], []
        for t0 in np.arange(y.t[0], y.t[-1] - h, every):
            for i in (0, 1):
                tau, acc, d = _params_for(model, params, i)
                w0 = float(np.interp(t0, y.t, y.wheel(i)))
                r0 = float(np.interp(t0, ref.t, ref.wheel(i))) if ref is not None else w0
                inside = y.t[(y.t > t0) & (y.t < t0 + h)]
                tt = np.r_[t0, inside, t0 + h]
                pred, _ = simulate(u.t, u.wheel(i), tt, tau, acc, d, w0=w0, r0=r0, t_start=t0)
                spd.append(pred[-1] - float(np.interp(t0 + h, y.t, y.wheel(i))))
                if pos is not None:
                    meas = (float(np.interp(t0 + h, pos.t, pos.wheel(i)))
                            - float(np.interp(t0, pos.t, pos.wheel(i))))
                    ang.append(float(np.sum(np.diff(tt) * (pred[1:] + pred[:-1]) / 2)) - meas)
        out[f'{h:g}s'] = {'starts': len(spd) // 2, 'speed_error_rad_s': _stats(spd),
                          'wheel_angle_error_rad': _stats(ang) if ang else None}
    return out


def evaluate(model, params, runs, source='goal', target='present', hold_tail=.5,
             horizons=HORIZONS):
    """Per-run metrics of a fitted model (identification runs: in-sample; validation runs:
    independent). Angles and body motion use the measured encoder positions only
    (encoder_only); no external ground truth is part of a motor experiment."""
    rows = []
    for run in runs:
        u, y = getattr(run, source), getattr(run, target)
        if u is None or y is None or len(y.t) < 3:
            continue
        preds = []
        for i, side in ((0, 'left'), (1, 'right')):
            tau, acc, d = _params_for(model, params, i)
            pred, _ = simulate(u.t, u.wheel(i), y.t, tau, acc, d,
                               hold='linear' if source == 'trajectory' else 'zoh')
            preds.append(pred)
            err = pred - y.wheel(i)
            amp = max(float(np.max(np.abs(u.wheel(i)))), 1e-6)
            rise, fall, steady = _phases(u, i, y.t, hold_tail=hold_tail)
            row = {'experiment_id': run.experiment_id, 'wheel': side,
                   'rmse': float(np.sqrt(np.mean(err ** 2))), 'mae': float(np.mean(np.abs(err))),
                   'relative_rmse': float(np.sqrt(np.mean(err ** 2)) / amp),
                   'max_abs_error': float(np.max(np.abs(err))),
                   'steady_state_error': float(np.mean(err[steady])) if steady.any() else None,
                   'rise_rmse': float(np.sqrt(np.mean(err[rise] ** 2))) if rise.any() else None,
                   'decel_rmse': float(np.sqrt(np.mean(err[fall] ** 2))) if fall.any() else None,
                   'peak_speed_error': float(np.max(np.abs(pred)) - np.max(np.abs(y.wheel(i)))),
                   'initial_error': float(np.mean(np.abs(err[:max(3, len(err) // 10)])))}
            if run.position is not None and len(run.position.t) > 2:
                meas = np.interp(y.t, run.position.t, run.position.wheel(i))
                meas = meas - meas[0]
                ang = np.r_[0., np.cumsum(np.diff(y.t) * (pred[1:] + pred[:-1]) / 2)]
                row['wheel_angle_error_final_rad'] = float(ang[-1] - meas[-1])
                row['wheel_angle_error_max_rad'] = float(np.max(np.abs(ang - meas)))
            rows.append(row)
        if run.position is not None and len(run.position.t) > 2:
            drv = run.extra.get('driver') or DRIVER_DEFAULT
            r, b = drv['radius_m'], drv['separation_m']
            ang_p = [np.r_[0., np.cumsum(np.diff(y.t) * (p[1:] + p[:-1]) / 2)] for p in preds]
            ang_m = [np.interp(y.t, run.position.t, run.position.wheel(i)) for i in (0, 1)]
            ang_m = [a - a[0] for a in ang_m]
            dist = r * (ang_p[0] + ang_p[1]) / 2 - r * (ang_m[0] + ang_m[1]) / 2
            head = r * (ang_p[1] - ang_p[0]) / b - r * (ang_m[1] - ang_m[0]) / b
            rows.append({'experiment_id': run.experiment_id, 'wheel': 'body (encoder_only)',
                         'distance_error_final_m': float(dist[-1]),
                         'distance_error_max_m': float(np.max(np.abs(dist))),
                         'heading_error_final_rad': float(head[-1]),
                         'heading_error_max_rad': float(np.max(np.abs(head))),
                         'external_reference': 'none (motor experiments record encoders only)'})
        if horizons:
            rows.append({'experiment_id': run.experiment_id, 'wheel': 'horizons',
                         'horizons': horizon_errors(model, params, run, source, horizons)})
    return rows


def summarize(rows):
    """Mean of the per-wheel metrics over runs (None-safe)."""
    wheel = [r for r in rows if r.get('wheel') in ('left', 'right')]
    keys = ('rmse', 'mae', 'max_abs_error', 'steady_state_error', 'rise_rmse', 'decel_rmse',
            'peak_speed_error', 'wheel_angle_error_final_rad')
    out = {}
    for k in keys:
        v = [abs(r[k]) if k != 'rmse' and k != 'mae' else r[k] for r in wheel if r.get(k) is not None]
        if v:
            out[k] = float(np.mean(v))
    body = [r for r in rows if r.get('wheel', '').startswith('body')]
    for k in ('distance_error_final_m', 'heading_error_final_rad'):
        v = [abs(r[k]) for r in body]
        if v:
            out[k] = float(np.mean(v))
    hz = {}
    for r in rows:
        for h, m in (r.get('horizons') or {}).items():
            if m.get('wheel_angle_error_rad'):
                hz.setdefault(h, []).append(m['wheel_angle_error_rad']['mean'])
    if hz:
        out['horizon_wheel_angle_error_rad'] = {h: float(np.mean(v)) for h, v in hz.items()}
    return out


def uncertainty(model, runs, boot=20, seed=0, rel_limit=.25):
    """Experiment-level bootstrap, per-run spread, start-point sensitivity, amplitude split.

    Adds evidence on top of the local covariance flags; any of these marking a parameter
    as unstable sets it to identification_insufficient (never the other way round).
    """
    ident = [r for r in runs if r.present is not None and r.goal is not None]
    base = fit(model, ident)
    if base.get('status') != 'fitted':
        return {'status': base.get('status')}
    names = list(base['parameters'])
    out = {'model': model, 'experiments': len(ident)}
    per = []
    for run in ident:
        f = fit(model, [run])
        if f.get('status') == 'fitted':
            per.append({'experiment_id': run.experiment_id, **f['parameters'],
                        'status': f['parameter_status']})
    out['per_experiment'] = per

    def spread(rows):
        res = {}
        for n in names:
            # a single experiment that cannot observe a parameter says nothing about its
            # spread: only locally estimated values enter, the rest are counted
            v = np.array([r[n] for r in rows if (r.get('status') or {}).get(n, 'estimated')
                          == 'estimated'])
            excluded = len(rows) - len(v)
            if excluded:
                res.setdefault('_excluded', {})[n] = excluded
            if len(v) > 1:
                m = float(np.mean(v))
                res[n] = {'mean': m, 'std': float(np.std(v, ddof=1)), 'n': int(len(v)),
                          'p2.5': float(np.percentile(v, 2.5)), 'p97.5': float(np.percentile(v, 97.5)),
                          'relative_spread': float(np.std(v, ddof=1) / max(abs(m), 1e-9))}
        return res
    out['per_experiment_spread'] = spread(per)
    if len(ident) >= 3 and boot > 0:
        rng = np.random.default_rng(seed)
        draws = []
        for _ in range(boot):
            pick = [ident[k] for k in rng.integers(0, len(ident), len(ident))]
            f = fit(model, pick)
            if f.get('status') == 'fitted':
                draws.append(f['parameters'])
        out['bootstrap'] = {'draws': len(draws), 'by_experiment': True, **{'parameters': spread(draws)}}
    else:
        out['bootstrap'] = {'status': 'skipped: needs >= 3 experiments'}
    ramp, delay = MODELS[model]
    alt = []
    for tau0 in (.01, .1):
        x0 = [tau0, tau0] + ([30.] if ramp else []) + ([.06] if delay else [])
        f = fit(model, ident, starts=[x0])
        if f.get('status') == 'fitted':
            alt.append({'start': x0, 'parameters': f['parameters'], 'rmse': f['rmse']})
    near = [a for a in alt if a['rmse'] <= base['rmse'] * 1.02]
    out['start_sensitivity'] = {'alternatives': alt, 'equally_good': len(near),
                                'parameters': spread([base['parameters']] + [a['parameters'] for a in near])
                                if near else {}}
    amps = [max(float(np.max(np.abs(r.goal.left))), float(np.max(np.abs(r.goal.right))))
            for r in ident]
    if len(set(np.round(amps, 3))) > 1:
        cut = float(np.median(amps))
        small = [r for r, a in zip(ident, amps) if a <= cut]
        large = [r for r, a in zip(ident, amps) if a > cut]
        if small and large:
            fs, fl = fit(model, small), fit(model, large)
            if fs.get('status') == fl.get('status') == 'fitted':
                out['amplitude_split'] = {
                    'threshold_rad_s': cut, 'small': fs['parameters'], 'large': fl['parameters'],
                    'relative_difference': {n: abs(fs['parameters'][n] - fl['parameters'][n])
                                            / max(abs(base['parameters'][n]), 1e-9) for n in names}}
    status = dict(base['parameter_status'])
    reasons = {}
    checks = [('bootstrap', (out['bootstrap'].get('parameters') or {})),
              ('per_experiment', out['per_experiment_spread']),
              ('start_sensitivity', out['start_sensitivity']['parameters'])]
    for label, sp in checks:
        for n, v in sp.items():
            if n.startswith('_'):
                continue
            if n == 'delay' and abs(v['mean']) < .002:
                continue          # a delay at ~0 has no meaningful relative spread
            if v['relative_spread'] > rel_limit:
                reasons.setdefault(n, []).append(f'{label} relative spread {v["relative_spread"]:.2f}')
    for n, d in (out.get('amplitude_split') or {}).get('relative_difference', {}).items():
        if d > rel_limit and not (n == 'delay' and base['parameters'][n] < .002):
            reasons.setdefault(n, []).append(f'small vs large steps differ by {d:.2f} '
                                             '(model structure / amplitude dependence)')
    for n, why in reasons.items():
        status[n] = 'identification_insufficient (' + '; '.join(why) + ')'
    out['parameter_status'] = status
    out['base'] = base
    return out


def stage_uncertainty(robot, boot=20, seed=0, rel_limit=.25):
    """Per-experiment and experiment-bootstrap spread of the stage-wise estimate."""
    def composed(runs):
        ramp = fit_ramp(runs)
        lag = fit('first_order_delay', runs, source='trajectory', target='present')
        if ramp.get('status') != 'fitted' or lag.get('status') != 'fitted':
            return None
        st = {'tau_left': lag['parameter_status']['tau_left'],
              'tau_right': lag['parameter_status']['tau_right'],
              'motor_accel': ramp['parameter_status']['motor_accel'],
              'delay': 'estimated' if 'estimated' in (ramp['parameter_status']['delay'],
                                                      lag['parameter_status']['delay']) else 'x'}
        return {'tau_left': lag['parameters']['tau_left'], 'tau_right': lag['parameters']['tau_right'],
                'motor_accel': ramp['parameters']['motor_accel'],
                'delay': ramp['parameters']['delay'] + lag['parameters']['delay'], 'status': st}

    def spread(rows):
        out = {}
        for n in ('tau_left', 'tau_right', 'motor_accel', 'delay'):
            v = np.array([r[n] for r in rows if r['status'][n] == 'estimated'])
            if len(rows) - len(v):
                out.setdefault('_excluded', {})[n] = len(rows) - len(v)
            if len(v) > 1:
                m = float(v.mean())
                out[n] = {'mean': m, 'std': float(v.std(ddof=1)), 'n': int(len(v)),
                          'p2.5': float(np.percentile(v, 2.5)),
                          'p97.5': float(np.percentile(v, 97.5)),
                          'relative_spread': float(v.std(ddof=1) / max(abs(m), 1e-9))}
        return out
    per = [dict(c, experiment_id=r.experiment_id) for r in robot for c in [composed([r])] if c]
    out = {'per_experiment': per, 'per_experiment_spread': spread(per)}
    if len(robot) >= 3 and boot > 0:
        rng = np.random.default_rng(seed)
        draws = [c for _ in range(boot) for c in
                 [composed([robot[k] for k in rng.integers(0, len(robot), len(robot))])] if c]
        out['bootstrap'] = {'draws': len(draws), 'by_experiment': True, 'parameters': spread(draws)}
    else:
        out['bootstrap'] = {'status': 'skipped: needs >= 3 experiments'}
    unstable = {}
    for label, sp in (('bootstrap', out['bootstrap'].get('parameters') or {}),
                      ('per_experiment', out['per_experiment_spread'])):
        for n, v in sp.items():
            if n.startswith('_'):
                continue
            if v['relative_spread'] > rel_limit and not (n == 'delay' and abs(v['mean']) < .002):
                unstable[n] = f'{label} relative spread {v["relative_spread"]:.2f}'
    out['unstable'] = unstable
    return out


DELAY_MEANING = {
    'robot_clock': 'serial Goal write done (robot clock) -> start of the modelled ramp: motor '
                   'internal application + model error; network and watchdog excluded',
    'synced_clock': 'PC publish -> modelled ramp start using the robot header clock: Wi-Fi '
                    'one way + watchdog forward (<= 50 ms) + driver + motor',
    'pc_receive': 'PC publish -> modelled response at PC receive time: Wi-Fi both ways + '
                  'watchdog + driver + motor + receive scheduling',
    'truth': 'simulator internal time (no transport)',
}


def recommend(models):
    """Simplest fully identifiable model with the lowest BIC on identification data.

    Validation runs are not used here (they stay an independent final check). A more
    complex model is preferred only when its BIC is lower by more than 10.
    """
    order = ['first_order', 'first_order_delay', 'ramp_first_order', 'ramp_first_order_delay',
             'ramp_first_order_delay_stagewise']
    ok = [m for m in order if models.get(m, {}).get('status') == 'fitted'
          and all(v == 'estimated' for v in models[m].get('parameter_status', {}).values())]
    if not ok:
        return None, 'no candidate has all parameters identifiable'
    best = ok[0]
    for m in ok[1:]:
        if models[m]['bic'] < models[best]['bic'] - 10:
            best = m
    return best, ('lowest BIC on identification data among fully identifiable models; '
                  'complexity accepted only for a BIC gain > 10')


def identify(runs, models=tuple(MODELS), boot=20, uncertainty_for='recommended'):
    """Fit every model on identification runs; validation runs only for final evaluation."""
    ident = [r for r in runs if r.purpose != 'validation']
    valid = [r for r in runs if r.purpose == 'validation']
    robot = [r for r in ident if r.timeline == 'robot_clock' and r.trajectory is not None]
    timelines = sorted({r.timeline for r in runs})
    result = {'model_version': MODEL_VERSION,
              'identification_experiments': [r.experiment_id for r in ident],
              'validation_experiments': [r.experiment_id for r in valid],
              'timelines': timelines, 'models': {}, 'stages': {},
              'delay_meaning': {t: DELAY_MEANING.get(t, '?') for t in timelines},
              'data_sources': {r.experiment_id: r.sources for r in runs},
              'data_quality': {r.experiment_id: r.quality for r in runs},
              'resolution_note': ('model integration is continuous; data are used only at their '
                                  'recorded times (no up-sampling). tau shorter than the sample '
                                  'interval is weakly observable: see uncertainty')}
    if len(timelines) > 1:
        result['warning'] = 'experiments use different timelines; delays are not comparable'
    for model in models:
        fitted = fit(model, ident)
        if fitted['status'] == 'fitted':
            fitted['in_sample'] = summarize(evaluate(model, fitted['parameters'], ident))
            if valid:
                rows = evaluate(model, fitted['parameters'], valid)
                fitted['validation'] = rows
                fitted['validation_summary'] = summarize(rows)
        result['models'][model] = fitted
    if robot:   # stage-wise with measured Velocity Trajectory (robot clock only)
        ramp = fit_ramp(robot)
        lag = fit('first_order_delay', robot, source='trajectory', target='present')
        result['stages'] = {'goal_to_trajectory': ramp, 'trajectory_to_present': lag}
        if ramp.get('status') == lag.get('status') == 'fitted':
            # Same structure as ramp_first_order_delay, each parameter from the stage that
            # observes it: ramp from the measured trajectory, lag from trajectory -> wheel.
            p = {'tau_left': lag['parameters']['tau_left'], 'tau_right': lag['parameters']['tau_right'],
                 'motor_accel': ramp['parameters']['motor_accel'],
                 'delay': ramp['parameters']['delay'] + lag['parameters']['delay']}
            res = residuals('ramp_first_order_delay', np.array([p[k] for k in
                            ('tau_left', 'tau_right', 'motor_accel', 'delay')]), ident)
            st = {k: lag['parameter_status'][k] for k in ('tau_left', 'tau_right')}
            st['motor_accel'] = ramp['parameter_status']['motor_accel']
            st['delay'] = ('estimated' if ramp['parameter_status']['delay'] == 'estimated' or
                           lag['parameter_status']['delay'] == 'estimated' else
                           'identification_insufficient')
            n = max(res.size, 2)
            comp = {'model': 'ramp_first_order_delay', 'method': 'stage-wise (robot clock)',
                    'status': 'fitted', 'parameters': p, 'parameter_status': st,
                    'rmse': float(np.sqrt(np.mean(res ** 2))), 'max_abs_error': float(np.max(np.abs(res))),
                    'samples': int(res.size),
                    'bic': n * math.log(max(float(res @ res) / n, 1e-12)) + 4 * math.log(n),
                    'experiments': [r.experiment_id for r in robot],
                    'delay_parts': {'goal_to_trajectory': ramp['parameters']['delay'],
                                    'trajectory_to_present': lag['parameters']['delay']}}
            comp['in_sample'] = summarize(evaluate('ramp_first_order_delay', p, ident))
            if valid:
                rows = evaluate('ramp_first_order_delay', p, valid)
                comp['validation'], comp['validation_summary'] = rows, summarize(rows)
            result['models']['ramp_first_order_delay_stagewise'] = comp
    def check(m):
        if m.endswith('_stagewise'):
            u = stage_uncertainty(robot, boot=boot)
            result['models'][m]['uncertainty'] = u
            for k, why in u.get('unstable', {}).items():
                result['models'][m]['parameter_status'][k] = 'identification_insufficient (' + why + ')'
        elif result['models'].get(m, {}).get('status') == 'fitted':
            u = uncertainty(m, ident, boot=boot)
            result['models'][m]['uncertainty'] = {k: v for k, v in u.items() if k != 'base'}
            result['models'][m]['parameter_status'] = u['parameter_status']
    if uncertainty_for == 'all':
        for m in list(result['models']):
            check(m)
    rec, why = recommend(result['models'])
    checked = set()
    # the stricter checks may reject the candidate: re-check the next one (each model once)
    while uncertainty_for != 'none' and rec and rec not in checked and 'uncertainty' not in \
            result['models'][rec]:
        checked.add(rec)
        check(rec)
        rec, why = recommend(result['models'])
    result['recommended_model'] = rec
    result['recommendation_basis'] = why
    if rec:
        p = result['models'][rec]['parameters']
        diffs = []
        for run in ident:
            if run.present is None:
                continue
            for i in (0, 1):
                tau, acc, d = _params_for(rec, p, i)
                a, _ = simulate(run.goal.t, run.goal.wheel(i), run.present.t, tau, acc, d)
                b, _ = simulate_map(run.goal.t, run.goal.wheel(i), run.present.t, tau, acc, d)
                diffs.append(np.abs(b - a))
        if diffs:
            d = np.concatenate(diffs)
            result['mpc_5ms_map_vs_continuous'] = {'max_rad_s': float(d.max()),
                                                   'rms_rad_s': float(np.sqrt(np.mean(d ** 2)))}
    ok = [m for m in result['models'].values() if m.get('validation_summary')]
    if ok:
        best = min(ok, key=lambda m: m['validation_summary'].get('rmse', math.inf))['model']
        result['lowest_validation_rmse'] = best
        result['lowest_validation_rmse_note'] = 'reference only: not used for the recommendation'
    result['validated'] = bool(valid) and bool(ok)
    if not valid:
        result['validation_note'] = 'no independent validation runs: results are unvalidated'
    return result


def fit_ramp(runs):
    """Goal -> Velocity Trajectory: ramp rate and delay (no lag); same delay search."""
    from scipy.optimize import least_squares

    def res(p):
        out = []
        for run in runs:
            for i in (0, 1):
                _, ref = simulate(run.goal.t, run.goal.wheel(i), run.trajectory.t, .05,
                                  p[0], p[1])           # the profile does not depend on tau
                out.append(ref - run.trajectory.wheel(i))
        return np.concatenate(out)
    lo, hi = [5., 0.], [400., .15]
    x = _search(res, ['motor_accel', 'delay'], lo, hi, [[74.9, .02], [150., .02]])
    sol = least_squares(res, x, bounds=(lo, hi), x_scale=np.abs(x) + .01, diff_step=1e-3)
    if sol.cost > .5 * float(res(x) @ res(x)) + 1e-12:
        sol.x, sol.fun = x, res(x)
    out = _diagnose(sol, ['motor_accel', 'delay'], lo, hi)
    out['note'] = 'Velocity Trajectory is the motor profile target (measured, robot clock)'
    return out
