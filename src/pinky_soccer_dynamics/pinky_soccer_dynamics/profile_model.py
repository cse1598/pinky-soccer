"""Seven physical states plus previous-command memory for motor-profile NMPC."""
import math

import numpy as np

NX = 9
MODEL_VERSION = 'profile-v1'


def profile_accel(params):
    """Configured Dynamixel speed ramp; not a measured physical acceleration."""
    return float(params.limits['motor_accel'].value)


def motor_step(params, wheel, ref, command, dt):
    """Integrate the profile and loaded first-order response."""
    wheel, ref, command = [np.asarray(v, float).copy() for v in (wheel, ref, command)]
    n = max(1, int(math.ceil(dt / .005 - 1e-10)))
    h = dt / n
    decay = np.exp(-h / np.array([params.m('tau_left'), params.m('tau_right')]))
    a = profile_accel(params)*h
    for _ in range(n):
        nxt = ref + np.clip(command-ref, -a, a)
        wheel = decay*wheel + (1-decay)*(ref+nxt)/2
        ref = nxt
    return wheel, ref


def step(params, x, u, dt):
    """Numeric prediction, using the same substep map as symbolic_step."""
    x = np.asarray(x, float).copy()
    u = np.asarray(u, float)
    n = max(1, int(math.ceil(dt / .005 - 1e-10)))
    h = dt/n
    decay = np.exp(-h / np.array([params.m('tau_left'), params.m('tau_right')]))
    r = np.array([params.m('left_radius'), params.m('right_radius')])
    a = profile_accel(params)*h
    for _ in range(n):
        ref = x[5:7] + np.clip(u-x[5:7], -a, a)
        wheel = decay*x[3:5] + (1-decay)*(x[5:7]+ref)/2
        l, rr = r*(x[3:5]+wheel)/2
        v, w = (l+rr)/2, (rr-l)/params.m('track')
        mid = x[2]+h*w/2
        x[:3] += h*np.array([v*math.cos(mid), v*math.sin(mid), w])
        x[3:5], x[5:7] = wheel, ref
    x[7:9] = u
    return x


def symbolic_step(params, dt, with_disturbance=False):
    """CasADi discrete map; heading is continuous, not wrapped within dynamics."""
    import casadi as ca
    x, u = ca.SX.sym('x', NX), ca.SX.sym('u', 2)
    d = ca.SX.sym('d',2) if with_disturbance else ca.SX.zeros(2)
    tau = ca.DM([params.m('tau_left'),params.m('tau_right')])
    pos, wheel, ref = x[:3], x[3:5], x[5:7]
    n = max(1, int(math.ceil(dt / .005 - 1e-10)))
    h = dt/n
    decay = ca.DM(np.exp(-h / np.array([params.m('tau_left'), params.m('tau_right')])))
    r = ca.DM([params.m('left_radius'), params.m('right_radius')])
    a = profile_accel(params)*h
    for _ in range(n):
        nxt = ref + ca.fmin(ca.fmax(u-ref, -a), a)
        wh = decay*wheel + (1-decay)*((ref+nxt)/2+tau*d)
        lin = r*(wheel+wh)/2
        v, w = (lin[0]+lin[1])/2, (lin[1]-lin[0])/params.m('track')
        mid = pos[2]+h*w/2
        pos = pos+h*ca.vertcat(v*ca.cos(mid),v*ca.sin(mid),w)
        wheel, ref = wh, nxt
    result = ca.vertcat(pos,wheel,ref,u)
    return (x,u,d,result) if with_disturbance else (x,u,result)


def limit_command(params, command, v_cap, w_cap, direction=0):
    """Enforce body and wheel command bounds; scale both wheels for RPM cap."""
    cmd = np.asarray(command,float)
    if not np.all(np.isfinite(cmd)):
        return np.zeros(2)
    rl, rr, b = (params.m(k) for k in ('left_radius','right_radius','track'))
    v = (rl*cmd[0]+rr*cmd[1])/2
    w = (rr*cmd[1]-rl*cmd[0])/b
    if direction and direction*v < 0:
        v = 0.
    v, w = np.clip(v,-v_cap,v_cap), np.clip(w,-w_cap,w_cap)
    cmd = np.array([(v-b*w/2)/rl,(v+b*w/2)/rr])
    return cmd*min(1.,params.wheel_cmd_max()/max(np.max(np.abs(cmd)),1e-12))
