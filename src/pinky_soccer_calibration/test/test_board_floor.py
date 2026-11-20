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
