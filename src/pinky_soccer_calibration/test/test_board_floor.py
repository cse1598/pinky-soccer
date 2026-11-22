"""Known geometry verifies configurable board shape and optical/mechanical inversion."""

import cv2
import numpy as np
import pytest

from pinky_soccer_calibration.board_geometry import (
    Board, floor_pose, MECHANICAL_FROM_OPTICAL, registered_pose, rpy_matrix)


@pytest.mark.parametrize('cols,rows,mm', [(9, 6, 30), (7, 5, 24.5), (5, 4, 18)])
def test_floor_height_pitch_and_full_registration(cols, rows, mm):
    board = Board(cols, rows, mm)
    # Known ground-truth camera: +X forward, looking down 25 degrees, 0.18m high.
    mechanical = rpy_matrix(.03, .44, .1)
    board_from_camera = mechanical @ MECHANICAL_FROM_OPTICAL
    xyz = np.array([-.3, -.04, .18])
    camera_from_board = board_from_camera.T
    rv = cv2.Rodrigues(camera_from_board)[0]
    tv = -camera_from_board @ xyz
    k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    pixels = cv2.projectPoints(board.points(), rv, tv, k, np.zeros(5))[0]
    result = floor_pose(board.points(), pixels, k, np.zeros(5))
    assert result['height_m'] == pytest.approx(.18, abs=1e-5)
    assert result['mechanical_rpy_rad'] == pytest.approx([.03, .44, .1], abs=1e-4)
    pose = registered_pose(result, .3, .04, 0., .027)
    assert pose[:3] == pytest.approx([0, 0, .153], abs=1e-5)


def test_board_validation_and_immutable_capture_settings(tmp_path):
    from pinky_soccer_calibration.suite_model import Session
    with pytest.raises(ValueError):
        Board(2, 6, 30)
    with pytest.raises(ValueError):
        Board(9, 6, float('nan'))
    session = Session(tmp_path, 'rehearsal', 'board')
    session.configure_board(Board(7, 5, 25))
    session.frames['training'].append(tmp_path/'dummy.png')
    with pytest.raises(ValueError):
        session.configure_board(Board())


def test_height_only_resolves_grid_normal_without_changing_registered_pose_contract():
    board = Board()
    k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    mechanical = rpy_matrix(.03, .44, .1)
    rotation = (mechanical @ MECHANICAL_FROM_OPTICAL).T
    xyz = np.array([-.3, -.04, .18])
    pixels = cv2.projectPoints(board.points(), cv2.Rodrigues(rotation)[0],
                               -rotation @ xyz, k, np.zeros(5))[0]
    reflected = pixels.reshape(board.rows, board.columns, 1, 2)[::-1].reshape(-1, 1, 2).copy()
    with pytest.raises(ValueError):
        floor_pose(board.points(), reflected, k, np.zeros(5))
    result = floor_pose(board.points(), reflected, k, np.zeros(5), allow_normal_flip=True)
    assert result['height_m'] == pytest.approx(.18, abs=1e-5)
    assert result['mechanical_rpy_rad'][:2] == pytest.approx([.03, .44], abs=1e-4)


def test_custom_board_is_detected_and_recorded(tmp_path):
    from pinky_soccer_calibration.suite_model import Session
    board = Board(7, 5, 24.5)
    image = np.full((360, 480, 3), 180, np.uint8)
    for row in range(board.rows+1):
        for col in range(board.columns+1):
            x, y = 70 + 30*col, 70 + 30*row
            cv2.rectangle(image, (x, y), (x+30, y+30),
                          (255, 255, 255) if (row+col)%2 else (0, 0, 0), -1)
    assert len(board.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))) == 35
    session = Session(tmp_path, 'rehearsal', 'custom-board')
    session.configure_board(board)
    assert session.capture(image, 'training') == 1
    assert session.meta['board_inner_corners'] == [7, 5]
    assert session.meta['square_mm'] == 24.5


def test_height_tilt_only_does_not_require_board_registration_or_create_tf(tmp_path, monkeypatch):
    """Board heading and corner reversal must not change floor height/tilt."""
    import yaml
    from pinky_soccer_calibration.suite_model import Session
    session = Session(tmp_path, 'rehearsal', 'height-tilt')
    k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    session.camera = (k, np.zeros(5), (640, 480), 30.)
    session.results['camera'] = {'passed': True}
    session.write('camera.yaml', {'test_only': True})
    frame = np.zeros((480, 640, 3), np.uint8)
    for index in range(5):
        heading = index*.7
        mechanical = rpy_matrix(.03, .44, heading)
        camera_from_board = (mechanical @ MECHANICAL_FROM_OPTICAL).T
        xyz = np.array([.12-.4*np.cos(heading), .075-.4*np.sin(heading), .18])
        pixels = cv2.projectPoints(session.board.points(), cv2.Rodrigues(camera_from_board)[0],
                                   -camera_from_board @ xyz, k, np.zeros(5))[0]
        if index % 2:
            pixels = pixels[::-1].copy()
        monkeypatch.setattr(Board, 'detect', lambda self, gray: pixels)
        result = session.floor_capture(frame)
        assert result['height_m'] == pytest.approx(.18, abs=1e-5)
        assert result['roll_rad'] == pytest.approx(.03, abs=1e-4)
        assert result['pitch_rad'] == pytest.approx(.44, abs=1e-4)
        assert not result['applicable']
        assert 'base_pose' not in result and 'mechanical_rpy_rad' not in result
        assert 'extrinsics' not in session.results
    summary = session.results['camera_mount']
    assert summary['repeatability_passed']
    assert summary['height_m']['count'] == 5
    profile = yaml.safe_load(session.export(True).read_text())
    assert profile['camera_pose'] is None
    assert profile['results']['camera_mount']['height_m']['mean'] == pytest.approx(.18, abs=1e-5)
    assert len(list((session.root/'floor').glob('*.png'))) == 5
