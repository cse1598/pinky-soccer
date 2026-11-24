"""Point-to-pose NMPC in closed loop with the wheel model (no ROS)."""

import math

import numpy as np
import pytest

pytest.importorskip('casadi')


@pytest.fixture(scope='module')
def setup():
    from pinky_soccer_dynamics import params as P
    from pinky_soccer_mpc.pose_mpc import PoseMpc
    params = P.load('mujoco')
    return params, PoseMpc(params)


def drive(setup, x0, goal, forward_only=False, w_cap=0.0, limit=8.0, dt=0.05):
    """Return (arrival time or None, trajectory rows (x, u))."""
    from pinky_soccer_dynamics import profile_model as M
    params, mpc = setup
    mpc.reset()
    x, t, rows = np.r_[np.asarray(x0,float), np.zeros(4)], 0.0, []
    while t < limit:
        res = mpc.solve(x, goal, w_cap=w_cap, forward_only=forward_only)
        assert res['ok'], res['status']
        x = M.step(params,x,res['u'],dt)
        t += dt
        rows.append((x.copy(), np.array(res['u'])))
        err = math.atan2(math.sin(x[2] - goal[2]), math.cos(x[2] - goal[2]))
        if math.hypot(x[0] - goal[0], x[1] - goal[1]) < 0.03 and abs(err) < 0.08 \
                and abs(x[3]) + abs(x[4]) < 0.5:
            return t, rows
    return None, rows


@pytest.mark.parametrize('goal', [(0.6, 0.0, 0.0), (0.6, 0.3, math.pi / 2),
                                  (0.0, 0.4, math.pi), (-0.4, 0.2, 0.0)])
def test_reaches_pose_with_heading_fast(setup, goal):
    _, mpc = setup
    t, rows = drive(setup, (0, 0, 0, 0, 0), goal)
    assert t is not None
    # Lower bound: straight-line distance at full speed. Turns and braking add a little.
    assert t < math.hypot(goal[0], goal[1]) / mpc.v_limit + 1.6


def test_wheel_speed_and_acceleration_limits(setup):
    params, mpc = setup
    _, rows = drive(setup, (0, 0, 0, 0, 0), (0.0, 0.4, math.pi))
    us = np.array([u for _, u in rows])
    xs = np.array([x for x, _ in rows])
    assert np.max(np.abs(us)) <= mpc.wmax + 1e-6                     # 90 % of 103 rpm
    assert mpc.wmax == pytest.approx(0.9 * 103 * 2 * math.pi / 60)
    acc = np.abs(np.diff(xs[:, 3:5], axis=0)) / 0.05
    assert np.max(acc) <= mpc.amax * 1.05                             # 90 % of 74.9 rad/s^2


def test_forward_only_and_turn_cap(setup):
    params, mpc = setup
    r = 0.5 * (params.m('left_radius') + params.m('right_radius'))
    b = params.m('track')
    t, rows = drive(setup, (0, 0, 0, 0, 0), (-0.3, 0.0, 0.0), forward_only=True, w_cap=2.0)
    assert t is not None
    xs = np.array([x for x, _ in rows])
    v = r * (xs[:, 3] + xs[:, 4]) / 2
    w = r * (xs[:, 4] - xs[:, 3]) / b
    assert np.min(v) > -0.03                    # soft bound: no real reversing
    assert np.max(np.abs(w)) < 2.0 * 1.1


def test_no_stall_when_goal_is_behind(setup):
    # MuJoCo regression: at rest, goal behind with the same heading and a 2 rad/s turn cap.
    # Without the time-to-go stage cost the solver postponed the turn forever.
    t, _ = drive(setup, (0.77, 0.12, math.radians(70), 0, 0),
                 (0.55, -0.18, math.radians(70)), forward_only=True, w_cap=2.0)
    assert t is not None
