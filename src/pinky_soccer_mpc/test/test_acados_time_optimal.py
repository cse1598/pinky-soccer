"""Explicit acados integration and independent closed-loop regression."""
import math
import json

import numpy as np
import pytest

from pinky_soccer_dynamics import params as P
from pinky_soccer_dynamics import profile_model as plant
from pinky_soccer_mpc.acados_time_optimal import AcadosTimeOptimalMpc
from pinky_soccer_mpc.arrival import Arrival


@pytest.fixture(scope='module')
def setup():
    p = P.load('mujoco')
    return p, AcadosTimeOptimalMpc(p)


def test_generated_solver_resizes(setup):
    _, m = setup
    for n in (20, 6, 30, 2, 12):
        m.solver.set_new_time_steps(np.full(n, m.dt))
        assert m.solver.get(n,'x').shape == (9,)
        m.current_n = n


def test_replay_rejects_bad_command(setup):
    _, m = setup
    _, violation = m.validate_candidate(np.zeros(7), np.zeros(3),
        np.zeros(2), np.full((3,2),m.wmax*2), m.v_limit, 2., False, False)
    assert violation > 0.


@pytest.mark.parametrize('goal', [
    (.241,-.003,0.), (.6,0.,0.), (-.3,0.,0.), (.3,.3,0.),
    (0.,.4,math.pi), (0.,0.,math.pi), (.55,-.18,1.22)])
def test_acados_closed_loop(setup,goal):
    p,m=setup
    m.reset()
    x=np.zeros(9)
    arrival=Arrival()
    failures=0
    durations=[]
    feasible_count=0
    for i in range(120):
        v=(m.rl*x[3]+m.rr*x[4])/2
        w=(m.rr*x[4]-m.rl*x[3])/m.b
        inside,done,_,_=arrival.update(i*m.dt,x[:3],goal,v,w,x[3:5],True)
        if done:
            assert feasible_count>0
            print(f'goal={goal}, arrival={i*m.dt:.2f}s, '
                  f'median={np.median(durations):.1f}ms, '
                  f'p95={np.percentile(durations,95):.1f}ms, '
                  f'max={max(durations):.1f}ms')
            return
        if inside and arrival.since is not None:
            u=np.zeros(2)
        else:
            res=m.solve(x,goal,w_cap=2.)
            durations.append(res['ms'])
            failures=0 if res['ok'] else failures+1
            assert failures<3,json.dumps(dict(goal=goal,step=i,state=x.tolist(),
                attempts=res['attempt_statuses'],message=res['message']),indent=2)
            u=np.asarray(res['u'])
            if res['ok']:
                assert res['backend']=='discrete_time_optimal_acados'
                assert np.max(np.abs(u))<=m.wmax+1e-5
                assert abs((m.rr*u[1]-m.rl*u[0])/m.b)<=2.+1e-5
                assert np.max(np.abs(u-x[7:9]))<=m.amax*m.dt+1e-4
                if res['terminal_feasible']:
                    feasible_count+=1
                    assert res['planned_time']==pytest.approx(res['horizon_steps']*m.dt)
                    end=res['xs'][-1]
                    assert np.linalg.norm(end[:2]-goal[:2])<=m.pos_tol+1e-5
                    assert abs(math.atan2(math.sin(end[2]-goal[2]),
                                          math.cos(end[2]-goal[2])))<=m.ang_tol+1e-5
                    assert np.max(np.abs(end[3:7]))<=m.wheel_tol+1e-5
        x=plant.step(p,x,u,m.dt)
    pytest.fail(f'No settled arrival: {x}, target {goal}')


def test_elastic_terminal_does_not_relax_acceptance(setup):
    _,m=setup
    assert len(m.ocp.constraints.idxsh_e)==11
    assert np.all(m.ocp.cost.Zu_e>0)
    assert len(m.ocp.constraints.idxsh)==0
    x=np.zeros(7)
    goal=np.array([.6,0.,0.])
    u=np.zeros((2,2))
    _,v_terminal=m.validate_candidate(x,goal,np.zeros(2),u,m.v_limit,2.,False,True)
    _,v_path=m.validate_candidate(x,goal,np.zeros(2),u,m.v_limit,2.,False,False)
    assert v_terminal>0 and v_path==0


def test_internal_state_scaling(setup):
    _,m=setup
    x=np.array([.2,-.1,.4,3.,-4.,2.,-3.])
    prev=np.array([2.,-1.])
    z=m.pack_state(x,prev)
    assert np.allclose(z[:3],x[:3])
    assert np.allclose(z[3:5]*m.wmax,x[3:5])
    assert np.allclose(z[5:7]*m.wmax,x[5:7])
    assert np.allclose(z[7:9]*m.wmax,prev)


@pytest.mark.parametrize('goal',[(.3,.15,1.2),(0.,0.,math.pi)])
def test_one_period_delayed_closed_loop(setup,goal):
    p,m=setup
    m.reset()
    x=np.zeros(9)
    command=np.zeros(2)
    arrival=Arrival()
    failures=0
    for i in range(150):
        v=(m.rl*x[3]+m.rr*x[4])/2
        w=(m.rr*x[4]-m.rl*x[3])/m.b
        inside,done,_,_=arrival.update(i*m.dt,x[:3],goal,v,w,x[3:5],True)
        if done:
            return
        if inside and arrival.since is not None:
            command=np.zeros(2)
            m.reset()
            x=plant.step(p,x,command,m.dt)
            continue
        predicted=m.predictor.step(x[:7],command)
        res=m.solve(np.r_[predicted,command],goal,w_cap=2.)
        failures=0 if res['ok'] else failures+1
        assert failures<3,json.dumps(res['attempt_statuses'],indent=2)
        x=plant.step(p,x,command,m.dt)
        command=np.asarray(res['u'])
    pytest.fail(f'No delayed settled arrival: {x}, target {goal}')
