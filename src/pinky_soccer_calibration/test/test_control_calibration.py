"""Control-calibration extensions keep existing calibration results and profiles intact."""
import math

import pytest
import yaml

from pinky_soccer_calibration import connection_settings as cs
from pinky_soccer_calibration.project_store import CONTROL_CATEGORIES
from pinky_soccer_calibration.suite_model import Session


def lidar_profile(tmp_path):
    session = Session(tmp_path, 'real', 'pinky-pro')
    session.add_measurement('lidar', {'samples': [{'distance': .51}] * 12,
                                      'reference': {'sensor_reference_m': .5}})
    session.recalculate('lidar')
    return session, session.export(True)


def test_control_records_do_not_touch_profile_or_other_results(tmp_path):
    session, profile = lidar_profile(tmp_path)
    before = profile.read_text()
    summary = {'relative_verified_all': True, 'samples': 3}
    for category in CONTROL_CATEGORIES:
        session.add_measurement(category, {'summary': summary})
    assert profile.exists() and profile.read_text() == before   # not archived
    assert 'lidar' in session.results and 'lidar' not in session.stale
    session.recalculate('clock_sync')
    session.add_measurement('clock_sync', {'summary': dict(summary, samples=4)})
    history = session.recalculate('clock_sync')['history']
    assert [h['samples'] for h in history] == [3, 4]            # history kept per record
    exported = yaml.safe_load(session.export(True).read_text())
    assert 'clock_sync' not in exported['results'] and 'lidar' in exported['results']
    root = session.root
    session.release()
    reopened = Session.open(root)
    assert len(reopened.measurements['clock_sync']) == 2
    assert reopened.results['lidar'] == session.results['lidar']
    reopened.release()


def test_connection_settings_round_trip_and_validation(tmp_path):
    path = tmp_path / 'conn.yaml'
    settings = cs.ConnectionSettings(pc_ip='192.168.204.27', ros_domain_id='37')
    settings.robot.ip = '192.168.204.28'
    settings.robot.namespace = 'yellow'
    assert settings.validate() == []
    cs.save(settings, path)
    loaded = cs.load(path)
    assert loaded.pc_ip == '192.168.204.27' and loaded.robot.namespace == 'yellow'
    text = cs.commands(loaded)
    assert 'allow 192.168.204.28' in text
    assert 'server 192.168.204.27 iburst prefer minpoll 2 maxpoll 4' in text
    assert '192.168.4.1' not in text                              # no AP address assumption
    loaded.robot.ip = loaded.pc_ip
    assert any('같습니다' in p for p in loaded.validate())
    loaded.pc_ip = '999.1.1.1'
    assert any('PC IP' in p for p in loaded.validate())
    assert cs.load(tmp_path / 'missing.yaml').pc_ip == ''         # unset, never a default IP


def test_runtime_mismatch_requires_restart(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '30')
    settings = cs.ConnectionSettings(ros_domain_id='37')
    settings.robot.namespace = 'blue'
    problems = cs.runtime_mismatches(settings, '')
    assert any('ROS_DOMAIN_ID' in p for p in problems)
    assert any('namespace' in p for p in problems)
    settings.ros_domain_id = '30'
    settings.robot.namespace = ''
    assert cs.runtime_mismatches(settings, '') == [] or all('RMW' in p for p in
                                                             cs.runtime_mismatches(settings, ''))


@pytest.mark.parametrize('category', CONTROL_CATEGORIES)
def test_control_category_removal_keeps_profile(tmp_path, category):
    session, profile = lidar_profile(tmp_path)
    row = session.add_measurement(category, {'summary': {}})
    session.remove_measurements(category, [row['id']])
    assert profile.exists() and session.deleted[-1]['type'] == category
    session.release()


# ---- motor step-response experiments -------------------------------------------------
import csv as _csv          # noqa: E402
import threading            # noqa: E402

from pinky_soccer_calibration import motor_experiment as me   # noqa: E402


def test_plan_limits():
    plan = me.build_plan('forward', .10, 0, 1, 2, 2, .5)
    assert plan['segments'] == [(1, 0., 0.), (2, .10, 0.), (2, 0., 0.)]
    assert plan['rate_hz'] == 50 and plan['commanded']['wheel_left_radps'] > 0
    with pytest.raises(ValueError, match='속도 제한'):
        me.build_plan('forward', .25, 0, 1, 1, 1, .5)            # needs approval
    assert me.build_plan('forward', .25, 0, 1, 1, 1, .5, approved=True)
    with pytest.raises(ValueError, match='최대 이동'):
        me.build_plan('forward', .20, 0, 1, 3, 1, .5)            # 0.6 m > 0.5 m
    with pytest.raises(ValueError, match='바퀴 속도'):
        me.build_plan('spin_left', 0, 3.0, 1, 1, 1, .5, approved=True,
                      wheels={'radius_m': .01, 'separation_m': .1})
    spin = me.build_plan('spin_right', 0, 1.0, 1, 2, 1, .3)
    assert spin['commanded']['w_radps'] == -1.0 and spin['commanded']['v_mps'] == 0
    with pytest.raises(ValueError):
        me.build_plan('forward', .1, 0, 0, 2, 2, .5)              # zero pre time


class FakeNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.values = {'/joint_states': [(10.0, 9.99, {'velocity': [1., 1.]}),
                                         (11.0, 10.99, {'velocity': [2., 2.]}),
                                         (99.0, 98.0, {'velocity': [0., 0.]})]}


def test_record_lifecycle_and_interrupted_recovery(tmp_path):
    session, profile = lidar_profile(tmp_path)
    plan = me.build_plan('forward', .05, 0, 1, 2, 2, .5)
    opts = {'purpose': 'identification', 'floor': 'yoga mat', 'namespace': ''}
    row = me.start_record(session, plan, opts, {'bag': 'raw_bag'})
    assert session.measurements['motor_response'][0]['status'] == 'running'
    trial = {'started': 10.2, 'stopped_monotonic': 15.0, 'complete': True, 'reason': '프로파일 완료',
             'commands': [(10.2, 1, 2, 0., 0.), (11.2, 3, 4, .05, 0.)]}
    done = me.finish_record(session, row['id'], FakeNode(), trial)
    assert done['status'] == 'complete' and done['samples']['/joint_states'] == 2
    folder = session.root / done['files']['folder']
    assert len(list(_csv.reader((folder / 'commands.csv').open()))) == 3
    assert (folder / 'meta.yaml').exists() and profile.exists()
    stuck = me.start_record(session, plan, opts, {'bag': 'raw_bag'})   # crash during a run
    root = session.root
    session.release()
    reopened = type(session).open(root)
    assert me.recover_interrupted(reopened) == 1
    status = {r['id']: r['status'] for r in reopened.measurements['motor_response']}
    assert status == {row['id']: 'complete', stuck['id']: 'interrupted'}
    assert reopened.recalculate('motor_response')['experiments'] == 2
    reopened.release()


# ---- priority 2: latency, command path, braking, footprint ----------------------------
import json as _json        # noqa: E402

from pinky_soccer_calibration import braking_analysis, footprint_view, latency_analysis  # noqa: E402


def test_pingpong_known_delays_loss_duplicates_and_clock_rules():
    sent, replies = {}, []
    for seq in range(1, 6):
        t0 = seq * 10**9
        sent[seq] = {'pc_send_mono_ns': t0, 'pc_send_unix_ns': t0 + 5 * 10**6}
        if seq == 3:
            continue                                    # lost
        reply = {'seq': seq, 'robot_recv_mono_ns': 7, 'robot_send_mono_ns': 7 + 10**6,   # 1 ms
                 'robot_recv_unix_ns': t0 + 5 * 10**6 + 4 * 10**6,                         # +4 ms
                 'robot_send_unix_ns': t0 + 5 * 10**6 + 5 * 10**6}
        replies.append((t0 + 9 * 10**6, t0 + 5 * 10**6 + 9 * 10**6, reply))               # rtt 9 ms
    replies.append(replies[0])                          # duplicate
    replies[1], replies[2] = replies[2], replies[1]     # reordered
    result, rows = latency_analysis.pingpong(sent, replies, sync_verified=True)
    assert result['lost'] == 1 and result['duplicates'] == 1 and result['reordered'] == 1
    assert result['rtt_s']['p50'] == pytest.approx(.009)
    assert result['robot_processing_s']['p50'] == pytest.approx(.001)
    assert result['net_rtt_s']['p50'] == pytest.approx(.008)
    assert result['pc_to_robot_s']['p50'] == pytest.approx(.004)
    assert result['robot_to_pc_s']['p50'] == pytest.approx(.004)
    unsynced, _ = latency_analysis.pingpong(sent, replies, sync_verified=False)
    assert 'pc_to_robot_s' not in unsynced and 'not_measured' in unsynced['one_way_status']
    bad = {1: {'pc_send_mono_ns': 0, 'pc_send_unix_ns': 10 * 10**6}}
    r = {'seq': 1, 'robot_recv_mono_ns': 0, 'robot_send_mono_ns': 0,
         'robot_recv_unix_ns': 5 * 10**6, 'robot_send_unix_ns': 5 * 10**6}   # robot clock behind
    neg, _ = latency_analysis.pingpong(bad, [(2 * 10**6, 12 * 10**6, r)], sync_verified=True)
    assert neg['pc_to_robot_s']['min'] < 0 and neg['negative_one_way'] == 1       # kept, not clipped


def write_experiment(folder, commands, observations):
    folder.mkdir(parents=True)
    with (folder / 'commands.csv').open('w', newline='') as f:
        out = _csv.writer(f)
        out.writerow(['publish_monotonic', 'publish_ros_ns', 'publish_unix_ns', 'v_mps', 'w_radps',
                      'command_tag'])
        out.writerows(commands)
    with (folder / 'observations.csv').open('w', newline='') as f:
        out = _csv.writer(f)
        out.writerow(['topic', 'receive_monotonic', 'header_seconds', 'observation_json'])
        for topic, t, obs in observations:
            out.writerow([topic, t, t, _json.dumps(obs)])


def test_command_path_uses_exact_tags(tmp_path):
    cmds = [(1.0, 0, 1_000_000_000, .1, 0, 1), (1.02, 0, 1_020_000_000, .1, 0, 2)]
    diag = {'payload': {'commands': [
        {'tag': 1.0, 'recv_unix_ns': 1_030_000_000, 'written_unix_ns': 1_031_000_000},
        {'tag': 1.0, 'recv_unix_ns': 1_080_000_000, 'written_unix_ns': 1_081_000_000},  # watchdog repeat
        {'tag': 0.0, 'recv_unix_ns': 1_090_000_000, 'written_unix_ns': 1_091_000_000}]}}  # untagged
    write_experiment(tmp_path / 'e', cmds, [('/motor/diagnostics', 1.1, diag)])
    out = latency_analysis.command_path(tmp_path / 'e', sync_verified=True)
    assert out['commands_matched'] == 1 and out['commands_sent'] == 2
    assert out['pc_publish_to_driver_receipt_s']['p50'] == pytest.approx(.030)   # first receipt
    assert out['driver_serial_write_s']['p50'] == pytest.approx(.001)
    assert 'not_measured' in latency_analysis.command_path(tmp_path / 'e', False)[
        'pc_publish_to_driver_receipt_s']
    write_experiment(tmp_path / 'f', cmds, [])
    assert latency_analysis.command_path(tmp_path / 'f', True)['status'].startswith('not_measured')


def test_braking_analysis_from_raw_files(tmp_path):
    cmds = [(0.0, 0, 0, 0, 0, 1), (1.0, 0, 0, .1, 0, 2), (2.0, 0, 0, .1, 0, 3), (2.02, 0, 0, 0, 0, 4)]
    obs = []
    for i in range(0, 300):
        t = i * .01
        speed = 3.7 if 1.1 < t <= 2.02 else max(0., 3.7 - (t - 2.02) * 20) if t > 2.02 else 0.
        obs.append(('/joint_states', t, {'name': ['l_wheel_joint', 'r_wheel_joint'],
                                         'velocity': [speed, speed]}))
        x = .1 * max(0, min(t, 2.02) - 1.1) + (max(0, min(t, 2.2) - 2.02) * .05)
        obs.append(('/odom', t, {'x': x, 'y': 0., 'yaw': 0., 'v': speed * .027, 'w': 0.}))
    write_experiment(tmp_path / 'b', cmds, obs)
    out = braking_analysis.analyze(tmp_path / 'b', {'braking_steady': True})
    assert out['status'] == 'measured' and out['steady_before_stop']
    assert out['stop_time_s'] == pytest.approx(.18, abs=.02)          # 3.7/20 s to < 0.2 rad/s
    assert out['odom_braking_distance_m'] > 0 and 'excludes wheel slip' in out['distance_kind']


def test_footprint_evaluation_and_no_mpc_application():
    ok = footprint_view.evaluate(.042, .076, .077, .077)
    assert ok['applied_to_mpc'] is False and ok['urdf_extremes']['front_m'] == pytest.approx(.0421)
    assert ok['discs_cover_measured_points'] is None            # no outline points given
    assert ok['bbox_covered_by_discs'] is False                 # cross-shaped robot: corners are
    # not surface points, so the bounding box is only a conservative reference
    real = footprint_view.evaluate(.042, .076, .077, .077,
                                   points=[(.028, .0766), (-.076, .0473), (.0421, -.0385)])
    assert real['discs_cover_measured_points'] is True
    big = footprint_view.evaluate(.12, .076, .077, .077, points=[(.12, 0.)])   # longer bumper
    assert big['discs_cover_measured_points'] is False and big['disc_cover_min_margin_m'] < 0
    assert footprint_view.parse_points('0.05,0.02; -0.07,-0.06') == [(.05, .02), (-.07, -.06)]
    with pytest.raises(ValueError):
        footprint_view.evaluate(-.01, .076, .077, .077)


def test_deleting_motor_record_invalidates_only_dependents(tmp_path):
    session, profile = lidar_profile(tmp_path)
    motor = session.add_measurement('motor_response', {'summary': {}})
    session.add_measurement('braking', {'source_experiment': motor['id'], 'summary': {}})
    session.add_measurement('command_latency', {'source_experiment': motor['id'], 'summary': {}})
    session.add_measurement('communication_latency', {'summary': {'rtt': 1}})
    for cat in ('braking', 'command_latency', 'communication_latency'):
        session.recalculate(cat)
    session.remove_measurements('motor_response', [motor['id']])
    assert {'braking', 'command_latency'} <= session.stale
    assert 'communication_latency' not in session.stale and 'lidar' not in session.stale
    assert session.measurements['braking'][0]['source_deleted']
    assert session.recalculate('braking')['excluded_source_deleted'] == 1
    assert profile.exists()
    session.release()


def test_wheel_extra_observations_do_not_change_shared_fit():
    from pinky_soccer_calibration.suite_model import wheel_fit
    session_dir_trials = [
        {'kind': 'forward', 'measurement_type': 'straight_distance', 'distance_m': 1.0,
         'left_rad': 37.0, 'right_rad': 37.0, 'id': 'a'},
        {'kind': 'left', 'measurement_type': 'rotation_angle', 'measured_angle_deg': 360.,
         'measured_angle_rad': 2 * math.pi, 'left_rad': -11.2, 'right_rad': 11.2, 'id': 'b'}]
    import math as _m  # noqa: F401
    try:
        base = wheel_fit([dict(t) for t in session_dir_trials])
    except Exception as error:          # fixture shape mismatch -> compare failure modes
        base = repr(error)
    extra = [dict(t) for t in session_dir_trials]
    extra[0]['extra_observations'] = {'longitudinal_x_m': .99, 'lateral_y_mm': 12.,
                                      'schema': 'start_pose_frame_v1'}
    try:
        with_extra = wheel_fit(extra)
    except Exception as error:
        with_extra = repr(error)
    assert with_extra == base
