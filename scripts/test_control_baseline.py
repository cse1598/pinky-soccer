"""Measured-evidence semantics: goal attribution, censored metrics, and zero intervals."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))

import pytest
from control_baseline_metrics import metrics,stats,wrap
from analyze_control_baseline import cpu_rates


def row(role,t,**kw):
    return dict(role=role,wall=t,bag_ns=round(t*1e9),**kw)


def episode():
    return dict(case='straight',start_wall=0.,end_wall=1.,goal=[1.,0.,0.],result='failure')


def goal():
    return row('goal',.01,action='GoToPose',goal_id='new',x=1.,y=0.,yaw=0.)


def test_old_goal_and_repeated_solve_are_not_attributed_to_new_goal():
    rows=[goal(),row('status',.02,data=dict(goal_id='old',solve_id=1,ms=999)),
          *[row('status',t,data=dict(goal_id='new',solve_id=2,ms=40,activation_missed=True)) for t in (.1,.2,.3)]]
    out=metrics(rows,episode())
    assert out['completed_solver_metrics']['ms']==stats([40])
    assert out['missed_slot_count'] is None
    assert out['result']=='failure' and 'truth_final_position_error_m' not in out


def test_zero_between_nonzero_is_separate_from_planned_initial_and_terminal_stop():
    rows=[goal(),*[row('cmd',i/10,v=v,w=0.) for i,v in enumerate([0,0,.1,.1,0,0,.1,.1,0,0])],
          row('status',.401,data=dict(goal_id='new',reason='waiting for scheduled solution',activation_missed=True))]
    out=metrics(rows,episode())
    assert out['zero_run_starts']==3 and out['zero_intervals_between_nonzero']==1
    assert out['zero_intervals'][1]['nearby_activation_missed']
    assert out['zero_intervals'][1]['between_nonzero_commands']


def test_source_age_uses_ros_clock_and_zero_stamp_is_unknown():
    rows=[row('joints',.1,stamp_ns=0),row('joints',.2,stamp_ns=190000000),
          row('joints',.3,stamp_ns=290000000)]
    out=metrics(rows,dict(case='fixed',start_wall=0,end_wall=1,result='profile_completed'))
    assert out['source_age_ms']['joints']==stats([10,10])
    assert out['joints_receipt_period_ms']['mean']==pytest.approx(100)


def test_trace_counter_is_not_sample_count_and_canceled_jobs_are_visible():
    trace=dict(goal_id='new',slots=9,missed=2,init_missed=1,track_missed=1,
               last_job=dict(seq=8,goal_id='new',canceled=True,worker_run_ms=130.))
    rows=[goal(),*[row('status',t,data=dict(goal_id='new',timing_trace=trace)) for t in (.2,.3,.4)]]
    out=metrics(rows,episode())
    assert out['missed_slot_count']==2
    assert out['observed_canceled_jobs']==1
    assert out['all_observed_jobs_metrics']['worker_run_ms']['n']==1


def test_cpu_accounts_for_wall_interval_and_missing_processes():
    samples=[dict(wall=1,pid=3,name='recorder',cpu_s=10.,rss=10),
             dict(wall=3,pid=3,name='recorder',cpu_s=11.,rss=20),
             dict(wall=4,pid=4,name='new',cpu_s=1.,rss=10)]
    assert cpu_rates(samples)==[dict(wall=3,pid=3,name='recorder',cpu_one_core_percent=50.,rss=20)]
    assert abs(float(wrap(2*3.141592653589793+.01))-.01)<1e-12


def test_pose_error_uses_source_time_instead_of_receipt_jitter():
    rows=[row('truth',.1,stamp_ns=100000000,x=.1,y=0.,yaw=0.),
          row('truth',.2,stamp_ns=200000000,x=.2,y=0.,yaw=0.),
          row('world',.9,stamp_ns=150000000,x=.15,y=0.,yaw=0.)]
    out=metrics(rows,dict(case='fixed',start_wall=0,end_wall=1,result='profile_completed'))
    assert out['pose_accuracy_source_aligned']['world']['xy_rmse_m']<1e-12
