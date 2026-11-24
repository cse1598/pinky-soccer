"""Phase-1 numerical regression against a separate motor-profile plant (no ROS)."""
import math
import numpy as np
import pytest

pytest.importorskip('casadi')
pytest.importorskip('scipy')

from pinky_soccer_dynamics import params as P
from pinky_soccer_dynamics import profile_model as plant
from pinky_soccer_mpc.pose_mpc import PoseMpc
from pinky_soccer_mpc.arrival import Arrival


@pytest.fixture(scope='module')
def setup():
    params = P.load('mujoco')
    return params, PoseMpc(params)


@pytest.mark.parametrize('goal', [
    (.241,-.003,0.), (.6,0.,0.), (-.3,0.,0.), (.3,.3,0.),
    (0.,.4,math.pi), (0.,0.,math.pi), (.55,-.18,1.22)])
def test_closed_loop(setup, goal):
    p, m = setup
    m.reset()
    x = np.zeros(9)
    arrival = Arrival()
    for i in range(120):
        v = (m.rl*x[3]+m.rr*x[4])/2
        w = (m.rr*x[4]-m.rl*x[3])/m.b
        inside,done,_,_ = arrival.update(i*m.dt,x[:3],goal,v,w,x[3:5],True)
        if done:
            return
        if inside:
            u=np.zeros(2)
        else:
            res=m.solve(np.r_[x[:5],x[7:9]],goal,w_cap=2.)
            assert res['ok'], res
            u=np.asarray(res['u'])
            assert np.max(np.abs(u))<=m.wmax+1e-6
            assert abs((m.rr*u[1]-m.rl*u[0])/m.b)<=2.+1e-6
            assert np.max(np.abs(u-x[7:9]))<=m.amax*m.dt+1e-5
        x=plant.step(p,x,u,m.dt)
    pytest.fail(f'No settled arrival: {x}, target {goal}')


def test_arrival_requires_rest_and_dwell():
    a=Arrival()
    goal=(0.,0.,0.)
    assert not a.update(0.,goal,goal,0.,1.,[2.,-2.],True)[1]
    assert not a.update(.1,goal,goal,0.,0.,[0.,0.],True)[1]
    assert not a.update(.3,goal,goal,0.,0.,[0.,0.],True)[1]
    assert not a.update(.35,(.03,0.,0.),goal,0.,0.,[0.,0.],True)[1]
    assert not a.update(.4,goal,goal,0.,0.,[0.,0.],False)[1]
    assert not a.update(.5,goal,goal,0.,0.,[0.,0.],True)[1]
    assert a.update(.81,goal,goal,0.,0.,[0.,0.],True)[1]
