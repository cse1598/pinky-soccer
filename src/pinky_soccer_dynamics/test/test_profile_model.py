"""Pure motor model regressions (no ROS/acados runtime required)."""
import numpy as np
import pytest
from pinky_soccer_dynamics import params, profile_model as M


def test_profile_ramp_and_lag():
    p = params.load('mujoco')
    x = np.zeros(9)
    u = np.array([9.,9.])
    y = M.step(p,x,u,.05)
    assert y[5] == pytest.approx(74.9*.05)
    assert 0 < y[3] < y[5]
    assert y[0] > 0 and y[1] == pytest.approx(0.)
    assert y[7:9] == pytest.approx(u)
    wheel,ref = M.motor_step(p,x[3:5],x[5:7],u,.05)
    assert y[3:5] == pytest.approx(wheel)
    assert y[5:7] == pytest.approx(ref)


def test_symbolic_matches_numeric():
    ca = pytest.importorskip('casadi')
    p = params.load('mujoco')
    x,u,f = M.symbolic_step(p,.05)
    fn = ca.Function('step',[x,u],[f])
    rng = np.random.default_rng(2)
    for _ in range(30):
        state = rng.uniform(-5,5,9)
        command = rng.uniform(-9,9,2)
        assert np.asarray(fn(state,command)).ravel() == pytest.approx(M.step(p,state,command,.05),abs=1e-10)


@pytest.mark.parametrize('direction',[-1,1])
def test_caps(direction):
    p = params.load('mujoco')
    for cmd in ([10,-10],[10,10],[-10,-10],[3,9]):
        l,r = M.limit_command(p,cmd,.12,2.,direction)
        v = (p.m('left_radius')*l+p.m('right_radius')*r)/2
        w = (p.m('right_radius')*r-p.m('left_radius')*l)/p.m('track')
        assert direction*v >= -1e-9
        assert abs(v) <= .12+1e-9 and abs(w) <= 2.+1e-9
        assert max(abs(l),abs(r)) <= p.wheel_cmd_max()+1e-9
