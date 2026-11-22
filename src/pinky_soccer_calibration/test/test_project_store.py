"""Project round trips, editable datasets, stale-result rejection and raw append runs."""

import copy

import cv2
import numpy as np
import pytest
import yaml

from pinky_soccer_calibration.suite_model import Session


def board_image():
    image = np.full((480, 640, 3), 180, np.uint8)
    for row in range(7):
        for col in range(10):
            x, y = 65+40*col, 65+40*row
            cv2.rectangle(image, (x, y), (x+40, y+40), (255,)*3 if (row+col)%2 else (0,)*3, -1)
    return image


def test_resume_split_remove_and_preserve_originals(tmp_path):
    session = Session(tmp_path, 'rehearsal', 'robot')
    session.capture(board_image(), 'training')
    path = session.frames['training'][0]
    session.results['camera'] = {'passed': True}
    session.assign_photos([session.relative(path)], 'validation')
    assert not session.frames['training'] and session.frames['validation'] == [path]
    assert 'camera' not in session.results
    root = session.root
    with pytest.raises(ValueError, match='다른 UI'):
        Session.open(root)
    session.release()
    opened = Session.open(root)
    assert opened.frames['validation'] == [path]
    assert opened.stale == {'camera'}
    opened.assign_photos([opened.relative(path)], 'training')
    opened.assign_photos([opened.relative(path)])
    assert path.exists() and not sum(opened.frames.values(), [])
    opened.capture(board_image(), 'training')
    assert opened.frames['training'][0] != path
    opened.release()
    again = Session.open(root)
    assert len(again.frames['training']) == 1
    assert len(again.deleted) == 1
    again.release()


def test_raw_sensor_history_deletion_and_recalculation(tmp_path):
    session = Session(tmp_path, 'rehearsal', 'robot')
    for distance in (.51, .53):
        session.add_measurement('lidar', {'samples': [{'distance': distance}]*12,
                                        'reference': {'sensor_reference_m': .5}})
    assert session.recalculate('lidar')[1]['bias'] == pytest.approx(.03)
    exported = session.export(True)
    saved_profile = copy.deepcopy(yaml.safe_load(exported.read_text()))
    row = session.measurements['lidar'][1]
    session.remove_measurements('lidar', [row['id']])
    assert 'lidar' not in session.results and not exported.exists()
    assert list((session.root/'exports').glob('previous_*.yaml'))
    result = session.recalculate('lidar')
    assert len(result) == 1 and result[0]['bias'] == pytest.approx(.01)
    assert saved_profile['results']['lidar'][1]['bias'] == pytest.approx(.03)
    root = session.root
    session.release()
    resumed = Session.open(root)
    assert len(resumed.measurements['lidar']) == 1
    assert len(resumed.deleted) == 1
    assert resumed.recalculate('lidar') == result
    resumed.release()


def test_legacy_folder_import_and_invalid_external_paths(tmp_path):
    root = tmp_path/'old'
    root.mkdir()
    (root/'session.yaml').write_text(yaml.safe_dump({'environment': 'real', 'robot_id': 'legacy'}))
    (root/'results.yaml').write_text(yaml.safe_dump({'ultrasonic': [{'mean': .5, 'count': 10}]}))
    session = Session.open(root)
    assert session.measurements['ultrasonic'][0]['legacy_report']['count'] == 10
    session.release()
    state = yaml.safe_load((root/'project.yaml').read_text())
    state['frames']['training'] = ['../outside.png']
    (root/'project.yaml').write_text(yaml.safe_dump(state))
    with pytest.raises(ValueError):
        Session.open(root)


def test_import_duplicates_and_failed_recompute_cannot_keep_old_result(tmp_path):
    path = tmp_path/'source.jpg'
    cv2.imwrite(str(path), board_image())
    session = Session(tmp_path, 'rehearsal', 'robot')
    assert session.import_photos([path])['imported'] == 1
    session.results['camera'] = {'passed': True}
    assert session.import_photos([path])['duplicates_skipped'] == 1
    with pytest.raises(ValueError, match='15장'):
        session.recalculate('camera')
    assert 'camera' not in session.results and session.camera is None
    session.release()


def test_partial_profile_excludes_failed_camera_and_stale_results(tmp_path):
    session = Session(tmp_path, 'rehearsal', 'partial')
    session.results = {'camera': {'passed': False}, 'lidar': [{'mean': .5}]}
    session.stale.add('camera')
    profile = yaml.safe_load(session.export(True).read_text())
    assert profile['results'] == {'lidar': [{'mean': .5}]}
    assert not profile['launch'] and profile['pending_recalculation'] == ['camera']
    session.release()


def test_current_photo_split_recomputes_and_preserves_export_snapshot(tmp_path):
    from test_intrinsics import draw_board
    session = Session(tmp_path, 'rehearsal', 'synthetic')
    for category, count in [('training', 20), ('validation', 14)]:
        for index in range(60):
            image = cv2.cvtColor(draw_board(index, category == 'validation'), cv2.COLOR_GRAY2BGR)
            try:
                session.capture(image, category)
            except ValueError:
                continue
            if len(session.frames[category]) == count:
                break
        assert len(session.frames[category]) == count
    assert session.recalculate('camera')['passed']
    profile = yaml.safe_load(session.export(True).read_text())
    from urllib.parse import unquote, urlparse
    snapshot = unquote(urlparse(profile['launch']['camera_info_url']).path)
    from pathlib import Path
    snapshot_bytes = Path(snapshot).read_bytes()
    root = session.root
    session.release()
    session = Session.open(root)
    assert session.camera[0].shape == (3, 3)
    session.assign_photos([session.relative(session.frames['training'][0])], 'validation')
    assert session.camera is None and 'camera' not in session.results
    assert Path(snapshot).read_bytes() == snapshot_bytes
    assert session.recalculate('camera')['passed']
    assert len(session.frames['training']) == 19
    assert len(session.frames['validation']) == 15
    assert session.export(True).exists()
    session.release()


def test_bag_recording_can_resume_without_overwriting(tmp_path, monkeypatch):
    import rclpy
    from pinky_soccer_calibration.suite_ros import Acquisition
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=174)
    node = Acquisition()
    session = Session(tmp_path, 'rehearsal', 'robot')
    try:
        node.begin(session)
        node.end()
        original = (session.root/'observations.csv').read_bytes()
        metadata = (session.root/'raw_bag/metadata.yaml').read_bytes()
        node.begin(session)
        node.end()
        assert (session.root/'observations.csv').read_bytes() == original
        assert (session.root/'raw_bag/metadata.yaml').read_bytes() == metadata
        assert len(session.meta['recordings']) == 2
        assert session.meta['recordings'][0]['bag'] != session.meta['recordings'][1]['bag']
        assert all(r['completed'] for r in session.meta['recordings'])
    finally:
        node.close()
        session.release()
        rclpy.shutdown()
