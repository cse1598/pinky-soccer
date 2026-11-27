"""Evidence semantics and actual CDR bag decoding; no running robot required."""
import importlib.util
import json
import math
from pathlib import Path
import sqlite3

import pytest

spec = importlib.util.spec_from_file_location('diagnostics', Path(__file__).with_name('analyze_gotopose_bag.py'))
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


def row(role, t, **values):
    return dict(role=role, bag_ns=1_800_000_000_000_000_000+round(t*1e9), **values)


def synthetic_rows():
    """Illustrative missed-slot/zero coincidence. Not a measured robot run."""
    rows = []
    for i in range(501):
        t = i*.02
        missed = 3 <= t < 3.2 or 6 <= t < 6.1
        v = 0 if missed or t < .2 or t > 9 else .08
        rows.append(row('cmd', t, v=v, w=0.))
        rows.append(row('status', t+.001, data=dict(
            goal_id='1', solve_id=i//5, active_solve_id=None if missed else i//5,
            state='waiting' if missed else 'track', activation_missed=missed,
            activation_jitter_sim_s=.004, ms=110 if missed else 30,
            acados_ms=18, worker_wait_ms=2, result_pickup_ms=8,
            solve_input_age_wall_s=.12 if missed else .04, command_state_age_wall_s=.12,
            feedback_age=.02, reason='waiting for scheduled solution' if missed else 'scheduled optimized input')))
    for i in range(201):
        t = i*.05
        c = diag.previous([dict(r, t=(r['bag_ns']-rows[0]['bag_ns'])/1e9)
                           for r in rows if r['role']=='cmd'], t, .1)
        rows.append(row('drive', t+.002, v=c['v'], w=0.))
    for i in range(301):
        t = i/30
        moving = .2 < t < 9 and not (3.05<t<3.25 or 6.05<t<6.15)
        speed = .08/.027 if moving else 0.
        rows.append(row('joints', t+.006, stamp_ns=1_800_000_000_000_000_000+round(t*1e9),
                        raw_l=speed, raw_r=-speed))
        for role in ('odom', 'world', 'amcl'):
            rows.append(row(role, t+.008, x=.07*t+(.1 if role=='amcl' and t>5 else 0),
                            y=0., yaw=0., frame='odom' if role=='odom' else 'map'))
    return rows


def test_sticky_flags_and_solver_metrics_are_not_counted_per_message():
    rows = [row('status', i*.02, data=dict(goal_id='4', solve_id=1,
             activation_missed=True, ms=120)) for i in range(20)]
    summary, solves, _ = diag.summarize(diag.prepare(rows)[0])
    assert len(solves) == 1
    assert summary['timing']['missed_flag_true_runs_lower_bound'] == 1
    assert summary['timing']['exact_missed_slot_count'] is None
    assert summary['timing']['metrics']['ms']['n'] == 1


def test_integer_origin_source_stamps_and_wrapped_pose():
    records = [row('odom', 0, x=0., y=0., yaw=math.pi-.01, frame='odom', stamp_ns=0),
               row('odom', .000000123, x=.01, y=0., yaw=-math.pi+.01, frame='odom')]
    g, _ = diag.prepare(records)
    assert g['odom'][1]['t'] == 123e-9
    assert g['odom'][0]['age_ms'] is None
    assert diag.pose_steps(g['odom'])[0]['angle'] == pytest.approx(.02)
    assert diag.previous(g['odom'], 2, .2) is None


def test_feedback_chain_keeps_raw_and_restamped_clocks_separate():
    from sensor_msgs.msg import JointState
    raw = JointState()
    raw.header.stamp.sec = 1
    raw.name, raw.velocity = ['left_wheel_joint', 'right_wheel_joint'], [2., -2.]
    merged = JointState()
    merged.header.stamp.sec = 2
    merged.name, merged.velocity = ['l_wheel_joint', 'r_wheel_joint'], [2., -2.]
    a = diag.normalize('raw_joints', raw, 'left_wheel_joint', 'right_wheel_joint')
    b = diag.normalize('joints', merged, 'l_wheel_joint', 'r_wheel_joint')
    assert a['raw_r'] == b['raw_r'] == -2.
    assert a['stamp_ns'] == 1_000_000_000
    assert b['stamp_ns'] == 2_000_000_000


def test_nonfinite_solver_diagnostics_preserve_missing_values():
    assert diag.stats([None, math.inf, math.nan, 30.])['n'] == 1
    assert diag.clean_json({'attempts': [{'residual': math.inf}]}) == {'attempts': [{'residual': None}]}


def test_missing_drive_does_not_use_pc_cmd_for_model():
    from pinky_soccer_dynamics import params
    g, _ = diag.prepare([row('cmd', 0, v=.08, w=0.),
                         row('joints', .01, raw_l=2., raw_r=-2.)])
    assert diag.motor_comparison(g, params.load('real')) == []


def test_model_replay_integrates_command_transition_and_right_sign():
    from pinky_soccer_dynamics import params
    from pinky_soccer_dynamics.profile_model import motor_step
    from pinky_soccer_dynamics.command_adapter import CommandAdapter
    p = params.load('real')
    adapter = CommandAdapter(p)
    a, b = adapter.driver_applied(.03, 0)[:2], adapter.driver_applied(.06, 0)[:2]
    w, ref = motor_step(p, (0, 0), (0, 0), a, .02)
    w, ref = motor_step(p, w, ref, b, .03)
    g, _ = diag.prepare([row('drive', 0, v=.03, w=0.), row('drive', .02, v=.06, w=0.),
                        row('joints', 0, raw_l=0., raw_r=0.),
                        row('joints', .05, raw_l=w[0], raw_r=-w[1])])
    replay = diag.motor_comparison(g, p)
    assert replay[-1]['r'] > 0
    assert replay[-1]['pred_l'] == pytest.approx(w[0])
    assert replay[-1]['pred_r'] == pytest.approx(w[1])


def test_decode_readonly_sqlite_namespaced_and_missing_joint(tmp_path):
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    p = tmp_path/'bag.db3'
    con = sqlite3.connect(p)
    con.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);'
                     'CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    con.executemany('INSERT INTO topics VALUES (?,?,?)',
                    [(1, '/yellow/cmd_vel', 'geometry_msgs/msg/Twist'),
                     (2, '/yellow/joint_states', 'sensor_msgs/msg/JointState')])
    twist = Twist()
    twist.linear.x = .08
    joints = JointState()
    joints.name, joints.velocity = ['left_only'], [1.]
    con.executemany('INSERT INTO messages VALUES (?,?,?,?)',
                    [(1, 1, 1000, serialize_message(twist)), (2, 2, 2000, serialize_message(joints))])
    con.commit()
    con.close()
    before = p.read_bytes()
    rows, inventory, errors = diag.read_bag(p,
        {'/yellow/cmd_vel': 'cmd', '/yellow/joint_states': 'joints'}, 'l_wheel_joint', 'r_wheel_joint')
    assert not errors and len(inventory) == 2
    assert rows[0]['v'] == .08 and rows[1]['raw_l'] is None
    assert p.read_bytes() == before


def test_cli_outputs_and_refuses_overwrite(tmp_path):
    fixture = tmp_path/'synthetic.jsonl'
    fixture.write_text('\n'.join(json.dumps(r) for r in synthetic_rows()))
    output = tmp_path/'analysis'
    args = ['--jsonl', str(fixture), '--output', str(output), '--context', 'SYNTHETIC test only']
    diag.main(args)
    summary = json.loads((output/'summary.json').read_text())
    assert summary['timing']['missed_flag_true_runs_lower_bound'] == 2
    assert summary['missing_roles'] == []
    assert summary['motor']['r']['one_step_error_rad_s']['n'] > 10
    assert all((output/name).stat().st_size > 1000 for name in
               ('timeline.png', 'timing.png', 'localization.png', 'motor_scatter.png', 'samples.csv'))
    with pytest.raises(SystemExit):
        diag.main(args)
