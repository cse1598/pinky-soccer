import numpy as np
from pinky_soccer_world_model.hybrid_ball import HybridBallFilter


def test_immediate_agreement_and_disagreement():
    f=HybridBallFilter()
    f.receive_referee(1.,[.3,0.],[0.,0.])
    f.step(1.)
    assert f.source=='referee_camera'
    f.receive_local(1.01,[.34,0.],.9,.2)
    f.step(1.01)
    assert f.source=='robot_camera'
    assert np.linalg.norm(f.x[:2]-[.3,0.])<=.025+1e-9
    f.receive_local(1.02,[.6,0.],.95,.2)
    f.receive_referee(1.02,[.3,0.],[0.,0.])
    f.step(1.02)
    assert f.source=='referee_camera'


def test_missing_or_low_confidence_immediately_uses_referee():
    f=HybridBallFilter()
    f.receive_referee(1.,[.3,0.])
    f.receive_local(1.,[.31,0.],.9,.2)
    f.step(1.)
    assert f.source=='robot_camera'
    f.receive_local(1.01,None)
    f.step(1.01)
    assert f.source=='referee_camera'
    f.receive_local(1.02,[.31,0.],.2,.2)
    f.step(1.02)
    assert f.source=='referee_camera'


def test_capture_time_compensation_and_velocity():
    f=HybridBallFilter()
    f.receive_referee(1.,[.3,0.],[1.,0.])
    f.receive_local(1.1,[.4,0.],.9,.2)
    f.step(1.1)
    assert f.source=='robot_camera'
    assert f.difference<1e-9
    assert f.x[2]>.8


def test_no_repeated_update_or_unconfirmed_local_without_referee():
    f=HybridBallFilter()
    f.receive_referee(1.,[.3,0.])
    assert f.step(1.)
    P=f.P.copy()
    assert not f.step(1.)
    np.testing.assert_allclose(f.P,P)
    f.receive_referee(1.01,None)
    f.receive_local(1.01,[.3,0.],.9,.2)
    assert not f.step(1.01)
    assert f.source=='none'
    assert f.last_seen==1.


def test_clock_and_covariance():
    f=HybridBallFilter()
    f.receive_referee(1.,[.3,0.],[0.,0.])
    f.step(1.)
    for i in range(1,20):
        t=1+i*.02
        f.receive_referee(t,[.3,0.],[0.,0.])
        f.receive_local(t,[.34 if i%2 else .6,0.],.9,.2)
        f.step(t)
        assert np.linalg.eigvalsh(f.P).min()>=-1e-12
