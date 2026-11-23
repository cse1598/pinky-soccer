"""
Wheel response identification and model validation from logged experiments.

Input logs are plain arrays (from a bag/CSV converter or a simulator): time t [s], wheel
targets u = [Omega_L_cmd, Omega_R_cmd] and measured wheel speeds w = [Omega_L, Omega_R]
(model convention, i.e. /joint_states with joint_sign applied).

Wheel model fitted per wheel:  dead time d, first-order lag tau, optional slope limit a
    Omega'(t) = clip((Omega_cmd(t - d) - Omega(t)) / tau, -a, a)
by grid search over d and a, and a 1-D search over tau (deterministic, no extra deps).

Identification results are written in our own file (identification_schema 1), never into the
calibration profile, and only marked accepted when an independent validation set passes.
"""

from datetime import datetime, timezone
import math

import numpy as np
import yaml

from .params import IDENTIFICATION_SCHEMA


def simulate_wheel(t, u, w0, tau, delay, accel_max=None):
    """Simulate one wheel with zero-order-hold commands sampled at t."""
    t = np.asarray(t, float)
    u = np.asarray(u, float)
    out = np.empty_like(t)
    out[0] = w = float(w0)
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        tc = t[k - 1] - delay
        j = np.searchsorted(t, tc, side='right') - 1
        cmd = u[j] if j >= 0 else u[0] * 0.0
        sub = max(1, int(math.ceil(dt / 0.002)))
        h = dt / sub
        for _ in range(sub):
            a = (cmd - w) / tau
            if accel_max:
                a = max(-accel_max, min(accel_max, a))
            w += h * a
        out[k] = w
    return out


def _rms(a, b):
    return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))


def fit_wheel(t, u, w, delays=None, accel_limits=(None,), taus=None):
    """Return the best {tau, delay, accel_max, rms} for one wheel."""
    t = np.asarray(t, float)
    dt = float(np.median(np.diff(t)))
    delays = delays if delays is not None else np.arange(0.0, 0.2001, max(dt, 0.005))
    taus = taus if taus is not None else np.geomspace(0.005, 1.0, 60)
    best = None
    for d in delays:
        for a in accel_limits:
            for tau in taus:
                err = _rms(simulate_wheel(t, u, w[0], tau, d, a), w)
                if best is None or err < best['rms']:
                    best = {'tau': float(tau), 'delay': float(d),
                            'accel_max': None if a is None else float(a), 'rms': err}
    # Local refinement of tau around the grid optimum.
    lo, hi = best['tau'] / 1.15, best['tau'] * 1.15
    for tau in np.linspace(lo, hi, 25):
        err = _rms(simulate_wheel(t, u, w[0], tau, best['delay'], best['accel_max']), w)
        if err < best['rms']:
            best.update(tau=float(tau), rms=err)
    return best


def fit_response(t, u, w, **kw):
    """Fit both wheels; u and w are (N, 2) arrays. Returns per-wheel results."""
    u, w = np.asarray(u, float), np.asarray(w, float)
    return {'left': fit_wheel(t, u[:, 0], w[:, 0], **kw),
            'right': fit_wheel(t, u[:, 1], w[:, 1], **kw)}


def wheel_validation(fit, t, u, w):
    """Return RMS and max error of a fitted wheel model on an independent data set."""
    u, w = np.asarray(u, float), np.asarray(w, float)
    out = {}
    for i, side in enumerate(('left', 'right')):
        f = fit[side]
        pred = simulate_wheel(t, u[:, i], w[0, i], f['tau'], f['delay'], f['accel_max'])
        err = pred - w[:, i]
        scale = max(1e-6, float(np.max(np.abs(w[:, i]))))
        out[side] = {'rms': float(np.sqrt(np.mean(err ** 2))),
                     'max_abs': float(np.max(np.abs(err))),
                     'rms_relative': float(np.sqrt(np.mean(err ** 2)) / scale)}
    return out


def trajectory_validation(params, t, u, x_meas, horizon=1.0):
    """
    Open-loop prediction error of the full model over sliding windows.

    Starting from each measured state, predict `horizon` seconds with the logged inputs and
    compare the end pose. Returns position/heading error statistics (m, rad).
    """
    from .model import Simulator
    t = np.asarray(t, float)
    u, x_meas = np.asarray(u, float), np.asarray(x_meas, float)
    dt = float(np.median(np.diff(t)))
    n = max(1, int(round(horizon / dt)))
    pos, ang = [], []
    for k in range(0, len(t) - n, max(1, n // 2)):
        sim = Simulator(params, x_meas[k], dt)
        for j in range(k, k + n):
            sim.step(u[j])
        e = sim.x - x_meas[k + n]
        pos.append(math.hypot(e[0], e[1]))
        ang.append(abs(math.atan2(math.sin(e[2]), math.cos(e[2]))))
    if not pos:
        raise ValueError('log shorter than the validation horizon')
    pos, ang = np.array(pos), np.array(ang)
    return {'horizon_s': horizon, 'windows': len(pos),
            'position_rms_m': float(np.sqrt(np.mean(pos ** 2))),
            'position_max_m': float(pos.max()),
            'heading_rms_rad': float(np.sqrt(np.mean(ang ** 2))),
            'heading_max_rad': float(ang.max())}


def identification_document(target, robot_id, fit, validation, thresholds, notes=''):
    """Build the identification file content; accepted only if validation passes."""
    ok = all(validation[s]['rms_relative'] <= thresholds['rms_relative'] for s in validation)
    params = {'tau_left': fit['left']['tau'], 'tau_right': fit['right']['tau'],
              'delay': 0.5 * (fit['left']['delay'] + fit['right']['delay'])}
    limits = [fit[s]['accel_max'] for s in ('left', 'right') if fit[s]['accel_max']]
    if limits:
        params['wheel_accel_max'] = float(min(limits))
    return {'identification_schema': IDENTIFICATION_SCHEMA, 'target': target,
            'robot_id': robot_id,
            'experiment_utc': datetime.now(timezone.utc).isoformat(),
            'units': {'tau': 's', 'delay': 's', 'wheel_accel_max': 'rad/s^2'},
            'model': 'first-order wheel response with dead time and optional slope limit',
            'parameters': params, 'fit': fit, 'validation': validation,
            'thresholds': thresholds, 'accepted': bool(ok), 'notes': notes}


def write_identification(path, document):
    """Write an identification YAML file."""
    with open(path, 'w') as fh:
        yaml.safe_dump(document, fh, sort_keys=False, allow_unicode=True)
    return path
