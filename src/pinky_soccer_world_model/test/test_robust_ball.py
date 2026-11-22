"""Image evidence survives lighting/occlusion; temporal memory is never a detection."""

import cv2
import numpy as np
from pinky_soccer_world_model.robust_ball import BallImageDetector, observe_ball
import pytest


RANGES = [((3, 160, 70), (19, 255, 255))]


def scene(center=(150, 125), radius=35):
    image = np.full((240, 320, 3), 45, np.uint8)
    cv2.circle(image, center, radius, (15, 100, 245), -1)
    return image


@pytest.mark.parametrize('condition', ['warm', 'cool', 'shadow', 'glare', 'occluded'])
def test_visible_arcs_recover_known_circle(condition):
    image = scene()
    if condition == 'warm':
        image = np.clip(image.astype(float)*[.7, .85, 1.15], 0, 255).astype(np.uint8)
    elif condition == 'cool':
        image = np.clip(image.astype(float)*[1.25, 1.05, .85], 0, 255).astype(np.uint8)
    elif condition == 'shadow':
        image[:, :155] = (image[:, :155].astype(float)*.3).astype(np.uint8)
    elif condition == 'glare':
        cv2.circle(image, (150, 112), 22, (255, 255, 255), -1)
    else:
        cv2.rectangle(image, (165, 80), (220, 170), (60, 70, 80), -1)
    original = image.copy()
    observation = observe_ball(image, RANGES)
    assert observation.blob is not None
    assert observation.blob[:3] == pytest.approx((150, 125, 35), abs=3)
    assert np.array_equal(original, image)
    if condition == 'occluded':
        assert observation.partial


def test_ball_is_separated_from_connected_orange_table():
    image = np.full((240, 320, 3), 45, np.uint8)
    cv2.rectangle(image, (0, 130), (319, 239), (60, 130, 190), -1)
    cv2.circle(image, (150, 125), 35, (15, 100, 245), -1)
    result = observe_ball(image, RANGES)
    assert result.blob[:3] == pytest.approx((150, 125, 35), abs=3)


def test_one_target_continuity_loss_and_reacquisition():
    detector = BallImageDetector(RANGES)
    first = detector.detect(scene(), 1., 'a')
    assert first.blob is not None
    image = scene((153, 126))
    cv2.circle(image, (270, 65), 40, (15, 100, 245), -1)
    next_frame = detector.detect(image, 1.1, 'b')
    assert next_frame.blob[:2] == pytest.approx((153, 126), abs=2)
    assert detector.detect(image, 1.1, 'b') is next_frame
    blank = np.zeros_like(image)
    assert detector.detect(blank, 1.2, 'c').blob is None
    distant = scene((270, 65), 40)
    assert detector.detect(distant, 1.3, 'd').blob is None
    assert detector.detect(distant, 2., 'e').blob[:2] == pytest.approx((270, 65), abs=2)
    # A rewound simulator/camera clock starts a new visual association.
    assert detector.detect(scene(), .5, 'f').blob[:2] == pytest.approx((150, 125), abs=2)


def test_white_circle_and_fully_hidden_ball_are_not_invented():
    image = np.full((240, 320, 3), 45, np.uint8)
    cv2.circle(image, (150, 125), 35, (255, 255, 255), -1)
    assert observe_ball(image, RANGES).blob is None
