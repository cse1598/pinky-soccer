"""Trace distinguishes schedule failure causes without accepting a late command."""
from pinky_soccer_mpc.scheduled_input import ScheduledInput
from pinky_soccer_mpc.timing_trace import TimingTrace


def test_late_activation_still_discards_ready_command():
    trace=TimingTrace();trace.goal(7)
    policy=ScheduledInput(.1,.025);due=policy.begin(0.)
    assert policy.offer(.08,'valid command')
    had_offer=policy.ready is not None
    ready,result,jitter=policy.take(.14)
    assert ready and result is None  # Existing safety policy is unchanged.
    trace.activation(.14,due,jitter,result is not None,had_offer,True,None)
    snap=trace.snapshot()
    assert snap['track_missed']==snap['missed']==1
    assert snap['last_slot']['had_offer']
    assert snap['last_slot']['jitter_sim_s']>.025


def test_pending_late_worker_remains_visible_after_generation_cancel():
    trace=TimingTrace();trace.goal(8)
    trace.activation(.1,.1,0.,False,False,False,42)
    trace.completed(42,8,1,2,10.,0.,.1,
        dict(ok=True,status=0,ms=110.,job_start_wall=10.002,job_end_wall=10.112,
             attempt_statuses=[dict(acados_ms=55.)]),10.12)
    snap=trace.snapshot()
    assert snap['init_missed']==1 and snap['track_missed']==0
    assert snap['last_job']['canceled'] and snap['canceled_jobs']==1
    assert abs(snap['last_job']['worker_run_ms']-110)<1e-8
    assert abs(snap['last_job']['result_pickup_ms']-8)<1e-8
    # A late result from the preceding goal must keep its own attribution.
    trace.goal(9)
    assert trace.snapshot()['slots']==0
    assert trace.snapshot()['last_job']['goal_id']=='8'


def test_exception_result_and_snapshot_do_not_require_worker_timestamps():
    trace=TimingTrace();trace.goal(1)
    trace.completed(1,1,0,0,3.,2.,None,dict(ok=False,status=-1),3.2)
    snap=trace.snapshot()
    assert snap['last_job']['worker_run_ms'] is None
    assert snap['last_job']['status']==-1 and snap['canceled_jobs']==0
