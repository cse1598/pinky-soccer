"""PART B behavior contracts: noisy arrival, real handoff gates and safe trajectory reuse."""
import math
import numpy as np
import pytest
from pinky_soccer_dynamics import params as P
from pinky_soccer_mpc.arrival import Arrival
from pinky_soccer_mpc.motion_management import ContinuousState,Completion,CompletionPolicy,TrajectoryBuffer,should_replan
from pinky_soccer_mpc.collision_monitor import CollisionMonitor


def test_state_rejects_jump_then_recovers_and_absolute_correction_converges():
    s=ContinuousState();s.predict(0.,0.,0.);assert s.correct((0,0,0),0,0)
    for i in range(1,61):
        t=i*.02;s.predict(t,0.,0.);s.correct((.02*(-1)**i,0,0),t,t)
    assert abs(s.pose[0])<.005
    for i in range(61,300):
        t=i*.02;s.predict(t,0,0);s.correct((.1,0,0),t,t)
    assert abs(s.pose[0]-.1)<1e-5  # No permanent global offset.
    assert not s.correct((1,0,0),6.,6.)
    assert not s.correct((1,0,0),6.02,6.02)
    assert s.correct((1,0,0),6.04,6.04)
    assert not s.correct((1,0,0),5.,7.)  # Stale is not fresh just because receipt is new.


def test_prediction_wrap_and_source_order():
    s=ContinuousState();s.predict(0,0,0);s.correct((0,0,math.pi-.01),0,0)
    s.predict(.1,.1,.2)
    assert -math.pi<=s.pose[2]<=math.pi
    before=s.pose.copy();s.correct((9,9,0),0,.1)
    assert np.allclose(s.pose,before)


def test_strict_brakes_before_standstill_without_relaxing_pose_precision():
    c=Completion(Arrival());p=CompletionPolicy()
    out=c.update(0,(.01,0,0),(.01,0,0),(0,0,0),.08,0,(3,3),True,p)
    assert out['brake'] and not out['done']
    assert not c.update(.2,(.01,0,0),(.01,0,0),(0,0,0),0,0,(0,0),True,p)['done']
    assert c.update(.51,(.01,0,0),(.01,0,0),(0,0,0),0,0,(0,0),True,p)['done']
    out=c.update(.6,(.04,0,0),(.04,0,0),(0,0,0),0,0,(0,0),True,p)
    assert not out['brake'] and not out['done']


def test_small_noise_does_not_repeatedly_release_brake_or_allow_false_success():
    c=Completion(Arrival());p=CompletionPolicy(mode='PRACTICAL_STOP')
    for i in range(20):
        x=.019 if i%2 else .023
        out=c.update(i*.02,(.019,0,0),(x,0,0),(0,0,0),0,0,(0,0),True,p)
        if i>0:assert out['brake']
        if out['done']:assert x<=.02
    assert not c.update(1,(.019,0,0),(.019,0,0),(0,0,0),0,0,(0,0),False,p)['done']


def test_legacy_completion_is_identical():
    legacy=Arrival();c=Completion(Arrival());p=CompletionPolicy(mode='LEGACY')
    for i in range(40):
        x=.019 if i%3 else .021;t=i*.02
        inside,done,d,a=legacy.update(t,(x,0,0),(0,0,0),0,0,(0,0),True)
        result=c.update(t,(x,0,0),(x,0,0),(0,0,0),0,0,(0,0),True,p)
        assert result['done']==done and result['brake']==(inside and legacy.since is not None)


@pytest.mark.parametrize('mode',['PASS_THROUGH','HANDOFF'])
def test_successor_contract_is_not_itself_success(mode):
    c=Completion(Arrival());policy=CompletionPolicy(mode=mode,next_target=(1.,0.,0.),next_ready=True)
    c.update(0,(-.01,0,0),(-.01,0,0),(0,0,0),.1,0,(3,3),True,policy)
    result=c.update(.1,(0,0,0),(0,0,0),(0,0,0),.1,0,(3,3),True,policy)
    assert result['eligible'] and not result['done'] and not result['brake']
    policy=CompletionPolicy(mode=mode,next_target=(1.,0.,0.),next_ready=False)
    assert not c.update(.2,(0,0,0),(0,0,0),(0,0,0),.1,0,(3,3),True,policy)['eligible']


def test_wrong_way_or_lateral_crossing_is_not_pass_through():
    for start,end in [((.01,0,0),(-.01,0,0)),((-.01,.1,0),(.01,.1,0)),((-.01,0,math.pi),(.01,0,math.pi))]:
        c=Completion(Arrival());p=CompletionPolicy(mode='PASS_THROUGH',next_target=(1,0,0),next_ready=True)
        c.update(0,start,start,(0,0,0),.1,0,(3,3),True,p)
        assert not c.update(.1,end,end,(0,0,0),.1,0,(3,3),True,p)['eligible']


def test_buffer_version_limits_and_independent_validation():
    b=TrajectoryBuffer(.1);result=dict(us=np.array([[1.,1.],[2.,2.],[3.,3.],[4.,4.]]))
    b.offer(result,'old');assert b.activate(0,10,'new',lambda us:True) is None
    b.offer(result,'new');assert b.activate(0,10,'new',lambda us:False) is None
    b.offer(result,'new');assert b.activate(0,10,'new',lambda us:True)==(1,1)
    assert b.continuation(.1,10.1,'new',lambda us:True)==(2,2)
    assert b.continuation(.2,10.2,'new',lambda us:False) is None  # Hazard overrides all hysteresis.
    assert b.continuation(.31,10.31,'new',lambda us:True) is None
    assert b.continuation(.1,10.31,'new',lambda us:True) is None


def test_collision_events_override_small_change_hysteresis():
    assert should_replan(True,.001,.001,.001,.01)==(False,'retain safe trajectory')
    assert should_replan(False,0,0,0,-10)==(True,'collision')
    assert should_replan(True,0,.3,0,0)[0]


def test_raw_monitor_stops_unconfirmed_obstacle_and_stale_scan():
    monitor=CollisionMonitor(P.load('mujoco'));state=np.zeros(7)
    command,diag=monitor.check(state,(0,0),(3,3),np.array([[.045,0.]]),0.)
    assert command==(0,0) and diag['state']=='STOP'
    command,diag=monitor.check(state,(0,0),(3,3),np.array([[1,1]]),.31)
    assert command==(0,0) and diag['state']=='STOP'
    command,diag=monitor.check(state,(0,0),(3,3),np.array([[1,1]]),0.)
    assert command==(3,3) and diag['state']=='CLEAR'


def test_moving_obstacle_and_rotating_swept_footprint():
    monitor=CollisionMonitor(P.load('mujoco'));state=np.zeros(7)
    command,diag=monitor.check(state,(0,0),(0,0),np.array([[1,1]]),0.,obstacles=[(.3,0,.08,-1,0)])
    assert diag['state']=='STOP'
    # Beside the static body but within its rotational swept area.
    command,diag=monitor.check(state,(0,0),(-3,3),np.array([[-.06,-.062]]),0.)
    assert diag['state'] in ('STOP','SLOW','REPLAN')


@pytest.mark.parametrize('payload',[dict(mode='invalid'),dict(position=float('nan')),dict(next_target=[1,2]),dict(next_ready='true')])
def test_invalid_contract_fails_closed(payload):
    with pytest.raises(ValueError):CompletionPolicy.parse(payload)


def test_nonconverging_reversals_never_become_success():
    from pinky_soccer_mpc.motion_management import ConvergenceGuard
    guard=ConvergenceGuard()
    for i in range(20):result=guard.update(i*.1,.03,.1,.02*(-1)**i,.1*(-1)**i)
    assert result
    guard.reset()
    for i in range(20):result=guard.update(i*.1,.03-i*.001,.1,.02,0.)
    assert not result


def test_vectorized_safety_prediction_matches_independent_scalar_motor_replay():
    monitor=CollisionMonitor(P.load('mujoco'))
    state=np.array([.1,-.2,.7,1.5,-1.2,2.,-2.])
    previous=(2.,-2.);command=(4.,1.)
    xs,times=monitor.rollout(state,previous,command,.1,.03)
    delay=math.ceil((monitor.latency+.03)/.02-1e-9)*.02
    current=state.copy();index=1
    for k in range(1,round(times[-1]/.005)+1):
        t=(k-1)*.005
        u=previous if t<delay-1e-9 else command if t<delay+.1-1e-9 else (0.,0.)
        current=monitor.model.step(current,u,.005)
        if abs(k*.005-times[index])<1e-9:
            assert np.max(np.abs(current-xs[index]))<1e-11
            index+=1
            if index==len(times):break
    assert index==len(times)


def test_reverse_pass_and_same_position_rotation_handoff():
    c=Completion(Arrival());p=CompletionPolicy(mode='PASS_THROUGH',next_ready=True,next_target=(1,0,math.pi))
    c.update(0,(-.01,0,math.pi),(-.01,0,math.pi),(0,0,math.pi),-.1,0,(-3,-3),True,p)
    assert c.update(.1,(.01,0,math.pi),(.01,0,math.pi),(0,0,math.pi),-.1,0,(-3,-3),True,p)['eligible']
    p=CompletionPolicy(mode='HANDOFF',next_ready=True,next_target=(0,0,math.pi/2))
    assert c.update(.2,(0,0,0),(0,0,0),(0,0,0),0,0,(0,0),True,p)['eligible']


def test_brake_latch_waits_until_zero_input_stop_is_inside_task_region():
    c=Completion(Arrival());p=CompletionPolicy()
    # Being inside at high angular speed is insufficient if coasting would overshoot.
    result=c.update(0,(0,0,.02),(0,0,.02),(0,0,0),0,1.,(-2,2),True,p,can_brake=False)
    assert not result['brake'] and not result['done']
    result=c.update(.1,(0,0,.01),(0,0,.01),(0,0,0),0,.03,(0,0),True,p,can_brake=True)
    assert result['brake'] and not result['done']
