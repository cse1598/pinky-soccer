"""Numeric and symbolic damping checks, independent of native acados."""
import math

import numpy as np
import pytest

from pinky_soccer_mpc import damping


def test_distance_and_heading_gate():
    near=damping.weights(0.,0.)
    half=damping.weights(damping.RADIUS_M**2,0.)
    far=damping.weights(.5**2,0.)
    turn=damping.weights(0.,math.pi)
    assert near==pytest.approx((1.,1.))
    assert half==pytest.approx((.5,.5))
    assert 0<far[0]<.03
    assert turn[0]==1. and turn[1]<.05
    assert damping.weights(0.,-.5)==pytest.approx(damping.weights(0.,.5))


def test_symbolic_and_numeric_cost_agree():
    ca=pytest.importorskip('casadi')
    e=ca.SX.sym('e',3)
    vw=ca.SX.sym('vw',2)
    scale=ca.SX.sym('scale')
    r=damping.symbolic_residual(e,vw,.25,2.,scale)
    f=ca.Function('damp_test',[e,vw,scale],[.5*ca.sumsqr(r)])
    for error in ([0.,0.,0.],[.08,0.,.4],[.5,.1,math.pi]):
        gv,gw=damping.weights(error[0]**2+error[1]**2,error[2])
        expected=damping.Q_V*gv*(.15/.25)**2+damping.Q_W*gw*(-1./2.)**2
        assert float(f(error,[.15,-1.],1.))==pytest.approx(expected)
        assert float(f(error,[.15,-1.],0.))==0.
        assert float(f(error,[.15,-1.],math.sqrt(2)))==pytest.approx(2*expected)


def test_replay_stage_cost_and_zero_velocity():
    B=np.array([[.125,.125],[-2.,2.]])
    xs=np.zeros((3,7))
    xs[0,3:5]=[-2.5,2.5]
    xs[1,3:5]=[5.,5.]
    cost=damping.trajectory_cost(xs,np.zeros(3),B,10.,.1,.25,2.,1.)
    assert cost==pytest.approx(.1*(damping.Q_W*.5**2+damping.Q_V*.5**2))
    assert damping.trajectory_cost(xs,np.zeros(3),B,10.,.1,.25,2.,0.)==0.
    assert damping.trajectory_cost(np.zeros_like(xs),np.zeros(3),B,10.,.1,.25,2.,1.)==0.
