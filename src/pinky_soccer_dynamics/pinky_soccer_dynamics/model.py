"""
Pinky Pro wheel-level dynamics.

State   X = [x, y, theta, Omega_L, Omega_R]   (map frame, wheel speeds in rad/s,
                                               positive = that wheel drives the robot forward)
Input   U = [Omega_L_cmd, Omega_R_cmd]

    v      = (r_R * Omega_R + r_L * Omega_L) / 2
    omega  = (r_R * Omega_R - r_L * Omega_L) / b
    x'     = v cos(theta)
    y'     = v sin(theta)
    theta' = omega
    Omega_L' = (Omega_L_cmd - Omega_L) / tau_L
    Omega_R' = (Omega_R_cmd - Omega_R) / tau_R

Optional extensions (both off when their parameter is non-positive / disabled):
  * wheel acceleration limit |Omega'| <= wheel_accel_max (the XL330 velocity profile ramp),
    applied as saturation in the numeric model and as a path constraint in the NMPC
  * command dead time `delay`: the input reaches the wheels `delay` seconds later

The NumPy functions are used for simulation, validation and identification; the CasADi
function builds the same right-hand side symbolically for acados.
"""

from collections import deque
import math

import numpy as np

NX, NU = 5, 2


def body_twist(params, omega_l, omega_r):
    """Return (v, w) of the body for wheel speeds (rad/s)."""
    rl, rr, b = params.m('left_radius'), params.m('right_radius'), params.m('track')
    return 0.5 * (rr * omega_r + rl * omega_l), (rr * omega_r - rl * omega_l) / b


def wheel_speeds(params, v, w):
    """Inverse of body_twist with the model geometry (rad/s)."""
    rl, rr, b = params.m('left_radius'), params.m('right_radius'), params.m('track')
    return (v - 0.5 * b * w) / rl, (v + 0.5 * b * w) / rr


def wheel_accel(params, x, u, limit=True):
    """Return wheel accelerations [dOmega_L, dOmega_R] (rad/s^2)."""
    al = (u[0] - x[3]) / params.m('tau_left')
    ar = (u[1] - x[4]) / params.m('tau_right')
    if limit:
        a = params.m('wheel_accel_max')
        al, ar = min(max(al, -a), a), min(max(ar, -a), a)
    return al, ar


def f(params, x, u, accel_limit=True):
    """Continuous dynamics dX/dt as a numpy array."""
    v, w = body_twist(params, x[3], x[4])
    al, ar = wheel_accel(params, x, u, accel_limit)
    return np.array([v * math.cos(x[2]), v * math.sin(x[2]), w, al, ar])


def rk4(params, x, u, dt, accel_limit=True):
    """Return the state after dt with constant input u (one RK4 step)."""
    x = np.asarray(x, float)
    k1 = f(params, x, u, accel_limit)
    k2 = f(params, x + 0.5 * dt * k1, u, accel_limit)
    k3 = f(params, x + 0.5 * dt * k2, u, accel_limit)
    k4 = f(params, x + dt * k3, u, accel_limit)
    out = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    out[2] = math.atan2(math.sin(out[2]), math.cos(out[2]))
    return out


class Simulator:
    """Discrete simulation with optional command dead time (for validation/tests)."""

    def __init__(self, params, x0, dt, substeps=4, accel_limit=True):
        self.p = params
        self.x = np.asarray(x0, float).copy()
        self.dt = dt
        self.sub = substeps
        self.accel_limit = accel_limit
        n_delay = int(round(params.m('delay') / dt))
        self.queue = deque([np.zeros(NU)] * n_delay)

    def step(self, u):
        """Apply u (after the dead time) for one period; return the new state."""
        self.queue.append(np.asarray(u, float))
        applied = self.queue.popleft()
        h = self.dt / self.sub
        for _ in range(self.sub):
            self.x = rk4(self.p, self.x, applied, h, self.accel_limit)
        return self.x.copy()


def predict(params, x0, inputs, dt, accel_limit=True):
    """Open-loop prediction for a sequence of inputs; returns an (N+1, 5) array."""
    sim = Simulator(params, x0, dt, accel_limit=accel_limit)
    out = [sim.x.copy()]
    for u in inputs:
        out.append(sim.step(u))
    return np.array(out)


def casadi_model(params, name='pinky_wheel'):
    """
    Return (x, u, xdot, f_expl, wheel_acc) CasADi symbols/expressions.

    Parameters are baked in as constants so a changed calibration produces a different
    generated solver; the MPC package regenerates when the parameter hash changes.
    """
    import casadi as ca
    x = ca.SX.sym('x', NX)
    u = ca.SX.sym('u', NU)
    xdot = ca.SX.sym('xdot', NX)
    rl, rr, b = params.m('left_radius'), params.m('right_radius'), params.m('track')
    v = 0.5 * (rr * x[4] + rl * x[3])
    w = (rr * x[4] - rl * x[3]) / b
    acc = ca.vertcat((u[0] - x[3]) / params.m('tau_left'),
                     (u[1] - x[4]) / params.m('tau_right'))
    f_expl = ca.vertcat(v * ca.cos(x[2]), v * ca.sin(x[2]), w, acc)
    return x, u, xdot, f_expl, acc
