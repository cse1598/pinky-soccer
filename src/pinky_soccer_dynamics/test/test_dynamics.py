"""Unit tests of the Pinky dynamics package (no ROS, no simulator)."""

import math

import numpy as np
from pinky_soccer_dynamics import identification as ident
from pinky_soccer_dynamics import model, params as P
from pinky_soccer_dynamics.command_adapter import (CommandAdapter, RPM_TO_RAD,
                                                   wheels_from_joint_state)
from pinky_soccer_dynamics.tools import sign_verdict
import pytest
import yaml


def with_model(p, **values):
    """Return params with model values replaced (source kept as 'assumption')."""
    for k, v in values.items():
        p.model[k] = P.Value(v, 'assumption', 'test')
    return P.validate(p)


# ----------------------------------------------------------------------------- params
@pytest.mark.parametrize('target', ['real', 'mujoco', 'gazebo'])
def test_defaults_load_and_flag_unmeasured(target):
    p = P.load(target)
    assert set(P.MODEL_KEYS) <= set(p.model)
    # Response parameters are never measured by default.
    assert {'tau_left', 'tau_right', 'delay'} <= set(p.unmeasured())
    if target == 'real':
        assert 'left_radius' in p.unmeasured()          # official value, not measured
        assert p.sign_right == -1                        # from code analysis, to verify


def _profile(tmp_path, env='real', wheels=True, launch=True):
    data = {'schema_version': 1, 'environment': env, 'robot_id': 'pinky-01',
            'human_verified': True, 'exported_utc': '2026-11-23T00:00:00+00:00',
            'project_revision': 3, 'results': {}, 'launch': {}}
    if wheels:
        data['results']['wheels'] = {'left_radius_m': 0.0272, 'right_radius_m': 0.0268,
                                     'separation_m': 0.0975, 'trials': 12,
                                     'equation_residual': {'mean': 0.001},
                                     'condition_number': 30.0}
    if launch:
        data['launch'] = {'wheel_radius': 0.027, 'wheel_separation': 0.0975}
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def test_profile_sets_measured_geometry_and_driver_separately(tmp_path):
    p = P.load('real', profile=_profile(tmp_path))
    assert p.model['left_radius'].source == 'measured'
    assert p.m('left_radius') == pytest.approx(0.0272)
    assert p.m('right_radius') == pytest.approx(0.0268)
    assert p.model['left_radius'].meta['robot_id'] == 'pinky-01'
    # Driver gets the launch values (what the official driver is started with).
    assert p.d('wheel_radius') == pytest.approx(0.027)
    assert p.driver['wheel_separation'].source == 'measured'
    # Response still unmeasured.
    assert 'tau_left' in p.unmeasured() and 'left_radius' not in p.unmeasured()


def test_rehearsal_profile_rejected_on_real(tmp_path):
    with pytest.raises(ValueError):
        P.load('real', profile=_profile(tmp_path, env='rehearsal'))
    sim = P.load('mujoco', profile=_profile(tmp_path, env='rehearsal'))
    assert sim.model['track'].source == 'simulation_identified'


def test_unreviewed_profile_rejected(tmp_path):
    path = _profile(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['human_verified'] = False
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        P.load('real', profile=path)


def test_identification_merge_requires_acceptance(tmp_path):
    doc = {'identification_schema': 1, 'target': 'real', 'robot_id': 'pinky-01',
           'experiment_utc': 'x', 'accepted': False,
           'parameters': {'tau_left': 0.08, 'tau_right': 0.09, 'delay': 0.03}}
    path = tmp_path / 'id.yaml'
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError):
        P.load('real', identification=path)
    doc['accepted'] = True
    path.write_text(yaml.safe_dump(doc))
    p = P.load('real', identification=path)
    assert p.m('tau_right') == pytest.approx(0.09)
    assert p.model['delay'].source == 'measured'
    doc['parameters']['left_radius'] = 0.03            # geometry is not a response key
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError):
        P.load('real', identification=path)


def test_validate_rejects_nonsense():
    p = P.load('real')
    with pytest.raises(ValueError):
        with_model(p, track=-0.1)


# ----------------------------------------------------------------------------- model
def test_straight_spin_and_curvature():
    p = with_model(P.load('mujoco'), left_radius=0.03, right_radius=0.03, track=0.1)
    assert model.body_twist(p, 10.0, 10.0) == pytest.approx((0.3, 0.0))
    assert model.body_twist(p, -5.0, 5.0) == pytest.approx((0.0, 3.0))
    # Unequal radii with equal wheel speeds -> curve towards the smaller wheel.
    q = with_model(P.load('mujoco'), left_radius=0.027, right_radius=0.029, track=0.1)
    v, w = model.body_twist(q, 10.0, 10.0)
    assert v == pytest.approx(0.28) and w == pytest.approx(0.02 / 0.1)
    assert model.wheel_speeds(q, v, w) == pytest.approx((10.0, 10.0))


def test_first_order_response_and_steady_state():
    p = with_model(P.load('mujoco'), tau_left=0.1, tau_right=0.2, wheel_accel_max=1e3)
    sim = model.Simulator(p, np.zeros(5), dt=0.005)
    n_tau = int(0.1 / 0.005)
    for _ in range(n_tau):
        x = sim.step([10.0, 10.0])
    assert x[3] == pytest.approx(10.0 * (1 - math.exp(-1)), rel=0.02)
    assert x[4] == pytest.approx(10.0 * (1 - math.exp(-0.5)), rel=0.02)
    for _ in range(400):
        x = sim.step([10.0, 10.0])
    assert x[3] == pytest.approx(10.0, rel=1e-3) and x[4] == pytest.approx(10.0, rel=1e-3)
    v, _ = model.body_twist(p, x[3], x[4])
    # The slower right wheel lags while accelerating, so the robot veers right (y < 0).
    assert x[0] > 0 and x[1] < 0 and v == pytest.approx(10 * p.m('left_radius'), rel=1e-3)


def test_accel_limit_and_delay():
    p = with_model(P.load('mujoco'), tau_left=0.01, tau_right=0.01, wheel_accel_max=50.0,
                   delay=0.04)
    sim = model.Simulator(p, np.zeros(5), dt=0.01, substeps=10)
    xs = [sim.step([10.0, 10.0]) for _ in range(30)]
    assert xs[3][3] == pytest.approx(0.0, abs=1e-9)        # still inside the dead time
    slope = (xs[12][3] - xs[8][3]) / 0.04
    assert slope == pytest.approx(50.0, rel=0.05)           # ramp at the limit


def test_spin_in_place_heading():
    p = with_model(P.load('mujoco'), tau_left=0.005, tau_right=0.005, wheel_accel_max=1000.0)
    traj = model.predict(p, [0, 0, 0, -5.0, 5.0], [[-5.0, 5.0]] * 100, 0.01)
    _, w = model.body_twist(p, -5.0, 5.0)
    assert traj[-1][2] == pytest.approx(math.atan2(math.sin(w), math.cos(w)), abs=1e-3)
    assert np.hypot(traj[-1][0], traj[-1][1]) < 1e-9


def test_casadi_matches_numpy():
    ca = pytest.importorskip('casadi')
    p = with_model(P.load('real'), left_radius=0.0272, right_radius=0.0268, tau_left=0.07,
                   tau_right=0.09)
    x, u, _, f_expl, _ = model.casadi_model(p)
    fn = ca.Function('f', [x, u], [f_expl])
    xs = np.array([0.1, -0.2, 0.7, 3.0, -2.0])
    us = np.array([5.0, 4.0])
    ref = model.f(p, xs, us, accel_limit=False)
    assert np.allclose(np.array(fn(xs, us)).ravel(), ref, atol=1e-12)


# ----------------------------------------------------------------------------- adapter
def test_adapter_uses_driver_parameters_not_model():
    # Model radii measured as 0.0272 / 0.0268, the driver still converts with 0.027.
    p = with_model(P.load('real'), left_radius=0.0272, right_radius=0.0268)
    a = CommandAdapter(p)
    v, w = a.to_twist(6.0, 7.0)
    assert a.driver_wheels(v, w) == pytest.approx((6.0, 7.0))   # exact wheel targets
    wl, wr, sat = a.round_trip(6.0, 7.0)
    q = 0.229 * RPM_TO_RAD
    assert not sat and abs(wl - 6.0) < q and abs(wr - 7.0) < q   # only 0.229 rpm truncation


def test_adapter_reproduces_driver_rpm_scaling():
    a = CommandAdapter(P.load('real'))
    wl, wr, sat = a.round_trip(20.0, 10.0)                       # 191 rpm > 100 rpm
    assert sat
    assert max(abs(wl), abs(wr)) <= 100 * RPM_TO_RAD + 1e-9
    assert wl / wr == pytest.approx(2.0, rel=0.01)               # ratio kept by the driver


def test_mujoco_emulates_real_driver_scaling():
    a = CommandAdapter(P.load('mujoco'))
    wl, wr, sat = a.round_trip(20.0, 10.0)                       # 191 rpm > 100 rpm
    assert sat and max(abs(wl), abs(wr)) == pytest.approx(100 * RPM_TO_RAD)
    assert wl / wr == pytest.approx(2.0)                         # no separate v/w clamp


def test_command_limits_from_manual():
    for target in ('real', 'mujoco', 'gazebo'):
        p = P.load(target)
        assert p.wheel_cmd_max() == pytest.approx(0.9 * 103 * RPM_TO_RAD)   # 9.708 rad/s
        assert p.wheel_accel_cmd_max() == pytest.approx(0.9 * 74.9)        # 67.41 rad/s^2
        assert p.limits['motor_rated_rpm'].source == 'manual'
        assert p.wheel_cmd_max() <= p.m('wheel_speed_max')


def test_joint_sign_conversion():
    p = P.load('real')
    names = ['l_wheel_joint', 'r_wheel_joint']
    assert wheels_from_joint_state(p, names, [3.0, -3.0]) == (3.0, 3.0)
    s = P.load('mujoco')
    assert wheels_from_joint_state(s, ['r_wheel_joint', 'l_wheel_joint'], [2.0, 1.0]) == (1.0, 2.0)


def test_sign_verdict():
    assert sign_verdict(1.2, -1.2, 0.2, 0.01) == {'left': 1, 'right': -1}
    with pytest.raises(ValueError):
        sign_verdict(1.2, -1.2, 0.01, 0.0)


# ----------------------------------------------------------------------------- identification
def _steps(dt=0.01):
    t, u = [], []
    for k, level in enumerate([0, 8, 8, 0, -6, -6, 0, 4]):
        for _ in range(40):
            t.append(len(t) * dt)
            u.append(level)
    return np.array(t), np.array(u, float)


def test_identification_recovers_tau_and_delay():
    t, u = _steps()
    w = ident.simulate_wheel(t, u, 0.0, tau=0.08, delay=0.03)
    w += np.random.default_rng(0).normal(0, 0.02, len(w))
    fit = ident.fit_wheel(t, u, w)
    assert fit['tau'] == pytest.approx(0.08, rel=0.1)
    assert fit['delay'] == pytest.approx(0.03, abs=0.011)


def test_identification_document_and_validation():
    t, u = _steps()
    U = np.c_[u, -u]
    W = np.c_[ident.simulate_wheel(t, u, 0, 0.06, 0.02),
              ident.simulate_wheel(t, -u, 0, 0.07, 0.02)]
    fit = ident.fit_response(t, U, W)
    val = ident.wheel_validation(fit, t, U, W)
    doc = ident.identification_document('mujoco', 'sim', fit, val, {'rms_relative': 0.05})
    assert doc['accepted'] and doc['parameters']['tau_right'] == pytest.approx(0.07, rel=0.1)


def test_trajectory_validation_zero_for_own_model():
    p = with_model(P.load('mujoco'), tau_left=0.05, tau_right=0.05)
    dt = 0.02
    inputs = [[6.0, 6.0]] * 50 + [[6.0, 2.0]] * 50
    x = model.predict(p, [0, 0, 0, 0, 0], inputs, dt)
    t = np.arange(len(x)) * dt
    res = ident.trajectory_validation(p, t[:-1], inputs, x[:-1], horizon=0.5)
    assert res['position_max_m'] < 1e-9 and res['heading_max_rad'] < 1e-9
