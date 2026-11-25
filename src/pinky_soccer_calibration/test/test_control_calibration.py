"""Control-calibration extensions keep existing calibration results and profiles intact."""

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
