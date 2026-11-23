"""Preserve intrinsics and mounting board geometry across project reopening."""

import copy

import cv2

import numpy as np

from pinky_soccer_calibration.board_geometry import (
    Board, MECHANICAL_FROM_OPTICAL, rpy_matrix)
from pinky_soccer_calibration.suite_model import Session

import pytest

import yaml


K = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])


def floor_image(board):
    """Render a board with known camera height and angles."""
    image = np.full((480, 640, 3), 150, np.uint8)
    rotation = (rpy_matrix(.02, .55, 0)@MECHANICAL_FROM_OPTICAL).T
    rv, tv = cv2.Rodrigues(rotation)[0], -rotation@np.array([-.4, .08, .25])
    for y in range(-1, board.rows):
        for x in range(-1, board.columns):
            points = np.array([[x, y, 0], [x+1, y, 0], [x+1, y+1, 0],
                               [x, y+1, 0]], np.float32)*board.square_mm/1000
            pixels = cv2.projectPoints(
                points, rv, tv, K, np.zeros(5))[0].reshape(-1, 2)
            cv2.fillConvexPoly(
                image, np.rint(pixels).astype(int),
                (255,)*3 if (x+y) % 2 else (0,)*3)
    return image


@pytest.mark.parametrize('height_override', [None, .26343])
def test_mixed_mount_boards_recalculate_with_each_photos_geometry(tmp_path, height_override):
    """Recompute mixed boards without changing intrinsic calibration."""
    session = Session(tmp_path, 'rehearsal', 'known-camera')
    if height_override is not None:
        session.meta['camera_mount_height_override'] = {'height_m': height_override}
    session.camera = (K.copy(), np.zeros(5), (640, 480), 30.)
    session.results['camera'] = {
        'passed': True, 'marker': 'known synthetic intrinsics'}
    training = session.root/'training.png'
    cv2.imwrite(str(training), np.zeros((480, 640, 3), np.uint8))
    session.frames['training'] = [training]
    previous_bytes = training.read_bytes()
    previous_camera = copy.deepcopy(session.results['camera'])
    profile = session.write('profile.yaml', {'previous_profile': True})
    boards = [Board(8, 5, 40), Board(6, 4, 25)]
    for index, board in enumerate(boards):
        session.configure_floor_board(board)
        if index == 0:
            assert profile.exists()
        result = session.floor_capture(floor_image(board))
        assert result['height_m'] == pytest.approx(.25, abs=.004)
        assert result['roll_rad'] == pytest.approx(.02, abs=.015)
        assert result['pitch_rad'] == pytest.approx(.55, abs=.015)
    expected = [[b.columns, b.rows, b.square_mm] for b in boards]
    assert [r['board'] for r in session.measurements['floor']] == expected
    assert session.board == Board() and np.array_equal(session.camera[0], K)
    assert session.results['camera'] == previous_camera
    assert training.read_bytes() == previous_bytes
    session.configure_floor_board(Board(5, 3, 15))
    root = session.root
    session.release()
    reopened = Session.open(root)
    try:
        assert reopened.floor_board == Board(5, 3, 15)
        assert reopened.board == Board()
        summary = reopened.recalculate('floor')
        if height_override is None:
            assert summary['height_m']['mean'] == pytest.approx(.25, abs=.004)
        else:
            assert summary['height_m']['mean'] == height_override
            assert summary['measured_height_m']['mean'] == pytest.approx(.25, abs=.004)
            assert summary['pitch_rad']['mean'] == pytest.approx(.55, abs=.015)
        actual = [r['board'] for r in reopened.results['floor_camera']]
        assert actual == expected
        assert np.array_equal(reopened.camera[0], K)
        assert training.read_bytes() == previous_bytes
    finally:
        reopened.release()


def test_old_project_uses_its_previous_board_for_floor_history(tmp_path):
    """Assign legacy measurements their original shared board specification."""
    session = Session(tmp_path, 'rehearsal', 'legacy-board')
    session.configure_board(Board(7, 5, 24.5))
    session.measurements['floor'] = [
        {'id': 'old', 'image_file': 'floor/old.png'}]
    session.save()
    root = session.root
    session.release()
    state = yaml.safe_load((root/'project.yaml').read_text())
    state.pop('floor_board')
    (root/'project.yaml').write_text(yaml.safe_dump(state))
    opened = Session.open(root)
    try:
        assert opened.floor_board == Board(7, 5, 24.5)
        opened.configure_floor_board(Board(8, 5, 40))
        assert opened.measurements['floor'][0]['board'] == [7, 5, 24.5]
        assert opened.board == Board(7, 5, 24.5)
    finally:
        opened.release()
