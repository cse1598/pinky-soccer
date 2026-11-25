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
