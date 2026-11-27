"""ROS adapter checks: measurement-time TF queue and successor acquisition contract."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
from pinky_soccer_mpc.motion_runtime import MotionRuntime
from pinky_soccer_mpc.motion_management import Completion,CompletionPolicy,ConvergenceGuard,TrajectoryBuffer
from pinky_soccer_mpc.arrival import Arrival


def test_scan_waits_for_exact_timestamp_transform_instead_of_using_latest():
    msg=LaserScan();msg.header.stamp.sec=10;msg.header.frame_id='laser'
    msg.range_min=.1;msg.range_max=5.;msg.ranges=[1.]*12;msg.angle_increment=.1
    times=[]
    class Transforms:
        ready=False
        def lookup_transform(self,target,source,stamp):
            times.append(stamp.nanoseconds)
            if not self.ready:raise RuntimeError('TF is still arriving')
            return NS(transform=NS(rotation=NS(x=0,y=0,z=0,w=1),translation=NS(x=.1,y=0)))
    motion=MotionRuntime.__new__(MotionRuntime)
    motion.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10020000000)))
    motion.tf=Transforms();motion.pending_scan=None;motion.scan=None
    motion.on_scan(msg)
    assert motion.scan is None and motion.pending_scan is msg
    motion.tf.ready=True;motion.consume_scan()
    assert motion.pending_scan is None and motion.scan[1]==10.
    assert times==[10000000000,10000000000]
    assert np.allclose(motion.scan[0][0],[1.1,0])


def runtime():
    n=NS(goal=NS(action='GoToPose',goal_id=1),arrival=Arrival(),dt=.1)
    m=MotionRuntime.__new__(MotionRuntime);m.node=n;m.enabled=True
    m.default_policy=CompletionPolicy();m.policy=CompletionPolicy(mode='HANDOFF')
    m.policies={};m.invalid_policies={};m.completion=Completion(n.arrival);m.guard=ConvergenceGuard()
    m.buffer=TrajectoryBuffer(.1);m.handoff_at=m.handoff_wall=m.handoff_target=None
    m.last_eligible=True;m.successor_prepared=False
    m.owner_goal_id=1;m.default='STRICT_STOP'
    return m


def test_only_prepared_matching_successor_can_adopt_active_trajectory():
    import time
    motion=runtime();now=time.monotonic()
    result=dict(us=np.array([[1.,1.],[1.,1.]]))
    motion.buffer.offer(result,(.2,0,0));motion.buffer.activate(1.,now,(.2,0,0),lambda us:True)
    motion.pending_target=(.2,0,0)
    motion.activated(1.,(.2,0,0))
    motion.node.goal.goal_id=2
    assert motion.select((.2,0,0),1.05)
    assert motion.buffer.active is not None
    assert motion.buffer.continuation(1.1,now+.1,(.2,0,0),lambda us:True)==(1,1)
    # No confirmed successor => the next request gets a fresh plan, not an arbitrary command.
    assert not motion.select((.4,0,0),1.15)
    assert motion.buffer.active is None


@pytest.mark.parametrize('offset',[-.01,.21])
def test_clock_discontinuity_or_expired_handoff_is_not_adopted(offset):
    import time
    motion=runtime();motion.pending_target=(.2,0,0);motion.activated(1.,(.2,0,0))
    motion.handoff_wall=time.monotonic()
    assert not motion.select((.2,0,0),1.+offset)


def test_emergency_override_invalidates_pending_generation_and_prepared_handoff():
    motion=runtime();n=motion.node
    n.diag={};n.last_solution=(1.,1.);n.active_solve_id=7;n.generation=4
    cleared=[];n.schedule=NS(clear=lambda:cleared.append(True))
    motion.handoff_at=1.;motion.handoff_wall=1.;motion.replan_reason='initial'
    result=motion.protect((0.,0.),dict(state='STOP',reason='hazard',replan=True),(1.,1.))
    assert result==(0.,0.) and n.last_solution is None and n.generation==5
    assert n.active_solve_id is None and cleared
    assert motion.handoff_at is None and motion.error


def test_successor_policy_arriving_before_goal_tick_does_not_cancel_handoff():
    import json
    import time
    motion=runtime()
    result=dict(us=np.array([[1.,1.],[1.,1.]]))
    motion.buffer.offer(result,(.2,0,0));motion.buffer.activate(1.,time.monotonic(),(.2,0,0),lambda us:True)
    motion.pending_target=(.2,0,0);motion.activated(1.,(.2,0,0))
    motion.node.goal.goal_id=2  # ROS has delivered B, but the control tick still owns A.
    motion.on_policy(NS(data=json.dumps(dict(goal_id='2',mode='STRICT_STOP'))))
    assert motion.handoff_at is not None and motion.owner_goal_id==1
    assert motion.select((.2,0,0),1.05)
    assert motion.owner_goal_id==2 and motion.buffer.active is not None


def test_private_replay_matches_solver_acceptance_without_sharing_worker_state():
    from pinky_soccer_dynamics import params as P
    from pinky_soccer_mpc.acados_time_optimal import AcadosTimeOptimalMpc
    from pinky_soccer_mpc.collision_monitor import CollisionMonitor
    from pinky_soccer_mpc.obstacle_snapshot import AvoidanceOptions
    params=P.load('mujoco');solver=AcadosTimeOptimalMpc(params)
    n=NS(pose=(0.,0.,0.),omega=(0.,0.),ref=(0.,0.),last_cmd=(0.,0.),dt=.1,
         pose_valid=True,time_mpc=solver,goal=NS(action='GoToPose'),caps=lambda:(0.,2.),
         forward_only=set(),avoid=AvoidanceOptions(),observer_disturbance=np.zeros(2))
    motion=MotionRuntime.__new__(MotionRuntime);motion.node=n;motion.raw=n.pose
    motion.monitor=CollisionMonitor(params)
    rng=np.random.default_rng(7)
    for us in [np.zeros((4,2)),np.ones((4,2)),*[rng.uniform(-10,10,(6,2)) for _ in range(20)]]:
        xs,violation=solver.validate_candidate(np.zeros(7),np.zeros(3),np.zeros(2),us,
            solver.v_limit,2.,False,False)
        assert motion.validate(us)==(violation<=1e-6)
        assert np.max(abs(motion.replay(us)-xs))<1e-10
    # A claimed terminal-feasible flag cannot replace the actual successor endpoint check.
    assert not motion.validate(np.zeros((4,2)),terminal_target=(.1,0.,0.))


def test_pose_recovery_drops_pre_correction_scan_cache():
    from pinky_soccer_mpc.motion_management import ContinuousState
    motion=MotionRuntime.__new__(MotionRuntime);motion.enabled=True;motion.filter=ContinuousState()
    motion.scan=motion.pending_scan=object()
    def world(x,t):
        return NS(self=NS(pose=NS(x=x,y=0.,theta=0.),confidence=1.,covariance=[0.]*9,
                          last_seen=NS(sec=0,nanosec=round(t*1e9))))
    motion.on_world(world(0.,.01),.01)
    assert motion.filter.valid and motion.scan is None
    motion.scan=object();motion.on_world(world(1.,.02),.02)
    assert not motion.filter.valid and motion.scan is None
    motion.on_world(world(1.,.04),.04)
    motion.scan=object();motion.on_world(world(1.,.06),.06)
    assert motion.filter.valid and motion.scan is None and motion.pending_scan is None


def test_stale_feedback_stop_discards_completion_dwell_and_prepared_transfer():
    from pinky_soccer_mpc.mpc_controller import MpcController
    motion=runtime();policy=CompletionPolicy(mode='STRICT_STOP')
    motion.completion.update(0,(0,0,0),(0,0,0),(0,0,0),0,0,(0,0),True,policy)
    motion.pending_target=(.2,0,0);motion.successor_prepared=True;motion.handoff_at=.1
    sent=[];n=NS(motion=motion,send=lambda u:sent.append(u),report=lambda *a,**k:None)
    MpcController.stop(n,'stale wheel feedback')
    assert sent==[(0.,0.)] and motion.completion.since is None
    assert motion.pending_target is None and not motion.successor_prepared and motion.handoff_at is None
    assert not motion.completion.update(.5,(0,0,0),(0,0,0),(0,0,0),0,0,(0,0),True,policy)['done']
    assert motion.completion.update(.81,(0,0,0),(0,0,0),(0,0,0),0,0,(0,0),True,policy)['done']


@pytest.mark.parametrize('inside',[True,False])
def test_approach_reversals_do_not_fail_a_braking_completion(inside):
    motion=runtime();n=motion.node
    n.pose=(.005,0.,.01) if inside else (.03,0.,.1)
    n.omega=(0.,0.);n.ref=(0.,0.);n.last_cmd=(0.,0.)
    n.adapter=NS(to_twist=lambda *u:(0.,0.))
    motion.raw=n.pose;motion.policy=CompletionPolicy(mode='STRICT_STOP')
    motion.monitor=NS(rollout=lambda *args:(np.array([[0.]*7]),None))
    motion.pending_target=None;motion.error=None
    motion.guard.best=0.;motion.guard.improved=0.
    motion.guard.reversals.extend([.1,.2,.3,.4])
    out=motion.arrival(1.5,(0.,0.,0.),0.,0.,True)
    assert out['brake']==inside
    assert motion.oscillating is not inside
    if inside:
        assert motion.error is None and not motion.guard.reversals
    else:
        assert motion.error is None  # Report reversals; do not preempt the original stall guard.
