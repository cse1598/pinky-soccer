"""Delay-observer regression; no native acados or ROS node required."""
import numpy as np
import pytest

from pinky_soccer_dynamics import params as P
from pinky_soccer_dynamics import profile_model as profile
from pinky_soccer_mpc.delay_observer import DelayObserver, DISTURBANCE_LIMIT
from pinky_soccer_mpc.ramp_predictor import RampPredictor


@pytest.fixture
def params():
    return P.load('mujoco')


@pytest.mark.parametrize('delay',[0.,.04,.09])
def test_nominal_response_not_misidentified_as_disturbance(params,delay):
    obs=DelayObserver(params)
    plant=RampPredictor(params,.02)
    x=np.zeros(7)
    obs.command(0.,[0.,0.])
    assert obs.measure(0.,delay,x[3:5])
    for k in range(1,51):
        t=k*.02
        u=np.array([4.,-3.]) if k<21 else np.array([-2.,5.])
        obs.command(t-.02,u)
        x=plant.step(x,u,.02)
        assert obs.measure(t,t+delay,x[3:5])
    assert np.max(np.abs(obs.d))<1e-8


def test_delayed_constant_disturbance_estimation(params):
    obs=DelayObserver(params)
    plant=RampPredictor(params,.02)
    x=np.zeros(7)
    d=np.array([3.,-5.])
    u=np.array([4.,-3.])
    obs.command(0.,[0.,0.])
    assert obs.measure(0.,.06,x[3:5])
    for k in range(1,61):
        obs.command((k-1)*.02,u)
        x=plant.step(x,u,.02,disturbance=d)
        assert obs.measure(k*.02,k*.02+.06,x[3:5])
    assert np.allclose(obs.d,d,atol=.01)
    current=obs.current(1.26)
    expected=plant.step(x,u,.06,disturbance=d)
    assert current is not None
    assert np.allclose(current[0],expected[3:7],atol=.001)


def test_pose_alignment_with_wheel_updates(params):
    obs=DelayObserver(params,enabled=False)
    plant=RampPredictor(params,.02)
    x=np.zeros(7)
    obs.command(0.,[0.,0.])
    assert obs.measure(0.,.04,x[3:5])
    pose=None
    for k in range(1,31):
        t=k*.02
        u=np.array([2.,5.]) if k<24 else np.array([-1.,3.])
        obs.command(t-.02,u)
        x=plant.step(x,u,.02)
        if k<=28:
            assert obs.measure(t,t+.04,x[3:5])
        if k==25:
            pose=x[:3].copy()
    aligned=obs.align_pose(pose,.5,.6)
    assert aligned is not None
    assert np.allclose(aligned,x,atol=2e-4)
    assert np.allclose(obs.d,0.)


def test_reject_stale_duplicate_and_future_feedback(params):
    obs=DelayObserver(params)
    assert obs.measure(1.,1.01,[0.,0.])
    assert not obs.measure(1.,1.02,[1.,1.])
    assert not obs.measure(.9,1.02,[1.,1.])
    assert not obs.measure(1.1,1.4,[1.,1.])
    assert not obs.measure(1.2,1.1,[1.,1.])
    assert obs.current(1.3) is None
    assert obs.accepted==1 and obs.rejected==4
    obs.reset()
    assert obs.current(1.0) is None


def test_observer_bound(params):
    obs=DelayObserver(params)
    assert obs.measure(0.,0.,[0.,0.])
    assert obs.measure(.02,.02,[100.,-100.])
    assert np.max(np.abs(obs.d))<=DISTURBANCE_LIMIT


@pytest.mark.parametrize('dt',[.02,.1,.075])
def test_disturbed_symbolic_and_scalar_maps_match(params,dt):
    import casadi as ca
    sx,su,sd,nxt=profile.symbolic_step(params,dt,with_disturbance=True)
    f=ca.Function('test_disturbed_map',[sx,su,sd],[nxt])
    predictor=RampPredictor(params,dt)
    rng=np.random.default_rng(12)
    for _ in range(12):
        x=rng.uniform(-5.,5.,9)
        u=rng.uniform(-8.,8.,2)
        d=rng.uniform(-10.,10.,2)
        expected=np.asarray(f(x,u,d)).ravel()
        actual=predictor.step(x[:7],u,disturbance=d)
        assert np.allclose(actual,expected[:7],atol=1e-10,rtol=1e-10)
