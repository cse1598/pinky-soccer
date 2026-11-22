"""Pixel feedback works before a recording session and leaves source frames intact."""

import cv2
import numpy as np
import pytest

from pinky_soccer_calibration.board_preview import annotate_board


@pytest.mark.parametrize('columns,rows', [(9, 6), (7, 5)])
def test_live_grid_feedback_without_session(columns, rows):
    image = np.full((360, 480, 3), 180, np.uint8)
    for row in range(rows+1):
        for col in range(columns+1):
            x, y = 60+30*col, 60+30*row
            color = (255, 255, 255) if (row+col) % 2 else (0, 0, 0)
            cv2.rectangle(image, (x, y), (x+30, y+30), color, -1)
    source = image.copy()
    preview, text, found = annotate_board(image, str(columns), str(rows))
    assert found and f'{columns*rows}개' in text and '인식 성공' in text
    assert np.array_equal(image, source)
    assert not np.array_equal(preview, source)
    _, _, wrong_pattern = annotate_board(image, columns+2, rows)
    assert not wrong_pattern


def test_missing_board_is_not_a_camera_error():
    image = np.full((240, 320, 3), 180, np.uint8)
    preview, text, found = annotate_board(image, 9, 6)
    assert not found and '미검출' in text
    assert np.array_equal(preview, image)


@pytest.mark.parametrize('columns,rows', [('', '6'), ('3.5', '6'), ('9', '0')])
def test_incomplete_settings_keep_video_visible(columns, rows):
    image = np.zeros((240, 320, 3), np.uint8)
    preview, text, found = annotate_board(image, columns, rows)
    assert not found and '설정 확인' in text
    assert np.array_equal(preview, image)
