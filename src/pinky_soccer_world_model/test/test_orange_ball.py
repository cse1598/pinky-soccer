"""Reject yellow panels/wood-like shapes without losing the orange golf ball."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from pinky_soccer_world_model.geometry import BLOB_SHAPE_DEFAULTS, detect_color_blob
import pytest
import yaml


@pytest.fixture
def detect():
    path = Path(get_package_share_directory('pinky_soccer_world_model'))/'config/ball_detector.yaml'
    cfg = yaml.safe_load(path.read_text())['ball_detector']['ros__parameters']
    flat = cfg['hsv_ranges']
    ranges = [(flat[i:i+3], flat[i+3:i+6]) for i in range(0, len(flat), 6)]
    shape = {key: cfg[key] for key in BLOB_SHAPE_DEFAULTS}
    return lambda image: detect_color_blob(image, ranges, cfg['min_area_px'],
                                           cfg['min_circularity'], **shape)


def bgr(h, s=240, v=240):
    return tuple(int(x) for x in cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0])


@pytest.mark.parametrize('hue', [3, 8, 12, 16, 19])
def test_vivid_orange_survives_logo_and_glare(detect, hue):
    image = np.zeros((240, 320, 3), np.uint8)
    cv2.circle(image, (160, 130), 25, bgr(hue), -1)
    cv2.putText(image, 'P', (152, 132), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 2)
    cv2.circle(image, (170, 119), 5, (255, 255, 255), -1)
    blob = detect(image)
    assert blob is not None
    assert blob[:2] == pytest.approx([160, 130], abs=1)
    assert blob[2] == pytest.approx(25, abs=1)


@pytest.mark.parametrize('hue', [20, 25, 30, 35])
def test_yellow_round_object_is_not_an_orange_ball(detect, hue):
    image = np.zeros((240, 320, 3), np.uint8)
    cv2.circle(image, (160, 130), 35, bgr(hue), -1)
    assert detect(image) is None


@pytest.mark.parametrize('shape', ['square', 'rectangle', 'ellipse', 'ring'])
def test_non_ball_orange_or_wood_like_regions_are_rejected(detect, shape):
    image = np.zeros((240, 320, 3), np.uint8)
    orange = bgr(12)
    if shape == 'square':
        cv2.rectangle(image, (100, 90), (180, 170), orange, -1)
    elif shape == 'rectangle':
        cv2.rectangle(image, (60, 90), (250, 170), orange, -1)
    elif shape == 'ellipse':
        cv2.ellipse(image, (160, 130), (60, 25), 0, 0, 360, orange, -1)
    elif shape == 'ring':
        cv2.circle(image, (160, 130), 35, orange, 5)
    elif shape == 'cropped':
        cv2.circle(image, (5, 130), 25, orange, -1)
    else:
        cv2.circle(image, (160, 130), 30, bgr(12, 100, 110), -1)
    assert detect(image) is None


@pytest.mark.parametrize('case', ['cropped', 'dull'])
def test_partial_frame_and_shadow_colour_can_be_recovered(detect, case):
    image = np.zeros((240, 320, 3), np.uint8)
    center = (5, 130) if case == 'cropped' else (160, 130)
    radius = 25 if case == 'cropped' else 30
    cv2.circle(image, center, radius, bgr(12, 100, 110), -1)
    blob = detect(image)
    assert blob is not None
    assert blob[:2] == pytest.approx(center, abs=2)
    assert blob[2] == pytest.approx(radius, abs=2)


def test_ball_wins_over_a_larger_yellow_panel_and_orange_rectangle(detect):
    image = np.zeros((240, 400, 3), np.uint8)
    cv2.rectangle(image, (20, 15), (180, 80), bgr(28), -1)
    cv2.rectangle(image, (220, 15), (370, 80), bgr(10), -1)
    cv2.circle(image, (200, 170), 15, bgr(10), -1)
    blob = detect(image)
    assert blob[:2] == pytest.approx([200, 170], abs=1)
