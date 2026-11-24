"""Pure timing and model parity tests, no acados solver required."""
import numpy as np
import pytest
from pinky_soccer_dynamics import params as P
from pinky_soccer_dynamics import profile_model as profile
from pinky_soccer_mpc.ramp_predictor import RampPredictor
from pinky_soccer_mpc.scheduled_input import ScheduledInput


def test_early_result_waits_for_boundary():
    s=ScheduledInput(.1,.025)
    s.begin(1.)
    assert s.offer(1.02,'new')
    assert s.take(1.08)==(False,None,None)
    due,result,jitter=s.take(1.1)
    assert due and result=='new' and abs(jitter)<1e-8


def test_late_result_never_applies():
    s=ScheduledInput(.1,.025)
    s.begin(1.)
    assert not s.offer(1.14,'old')
    due,result,_=s.take(1.14)
    assert due and result is None
    s.begin(1.14)
    s.clear()
    assert not s.offer(1.2,'cancelled')


def test_missing_result_at_boundary_is_not_reused():
    s=ScheduledInput(.1,.025)
    s.begin(0.)
    assert s.take(.1)[1] is None
    assert not s.offer(.11,'late')


@pytest.mark.parametrize('dt',[.02,.1,.075])
def test_ramp_predictor_matches_independent_profile(dt):
    p=P.load('mujoco')
    predictor=RampPredictor(p,.1)
    rng=np.random.default_rng(42)
    for _ in range(20):
        x=rng.uniform(-8,8,9)
        u=rng.uniform(-9,9,2)
        assert np.allclose(predictor.step(x[:7],u,dt),
                           profile.step(p,x,u,dt)[:7],rtol=1e-12,atol=1e-12)
