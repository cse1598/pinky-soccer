import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import referee_core as core


def test_measured_robot_passes_and_floor_shadow_is_rejected():
    rgb = np.array([[[44, 54, 54], [45, 73, 67]]], dtype=np.uint8)
    assert core.black_robot_mask(rgb).tolist() == [[255, 0]]
    assert np.array_equal(core.black_robot_mask(rgb), core.black_robot_mask(rgb[..., ::-1]))
    # Both used to pass the lightness-only body mask.
    old = cv2.inRange(cv2.cvtColor(rgb[..., ::-1], cv2.COLOR_BGR2HLS),
                      (0, 0, 0), (180, 62, 255))
    assert old.tolist() == [[255, 255]]


def test_inclusive_limits_and_parameter_overrides():
    image = np.array([[[65, 65, 65], [45, 65, 45], [44, 65, 45], [46, 66, 46]]], np.uint8)
    assert core.black_robot_mask(image).tolist() == [[255, 255, 0, 0]]
    assert core.black_robot_mask(np.array([[[67, 73, 45]]], np.uint8), 73, 28)[0, 0] == 255
    assert core.black_robot_mask(np.array([[[54, 54, 44]]], np.uint8), 50, 20)[0, 0] == 0
    with pytest.raises(ValueError):
        core.black_robot_mask(image, 256, 20)


def test_both_body_stages_use_the_shared_mask_with_identical_options(monkeypatch):
    image = np.full((300, 400, 3), (70, 100, 70), np.uint8)
    cv2.rectangle(image, (120, 100), (210, 170), (54, 54, 44), -1)
    field = SimpleNamespace(ppm=550., roi=np.full(image.shape[:2], 255, np.uint8),
                            metric=lambda point: np.asarray(point)/550.)
    calls = []
    original = core.black_robot_mask
    def spy(frame, maximum, spread):
        calls.append((frame.shape, maximum, spread))
        return original(frame, maximum, spread)
    monkeypatch.setattr(core, 'black_robot_mask', spy)
    options = dict(black_max_channel=64, black_max_spread=19)
    _, robots = core.detect(image, field, .02135, **options)
    assert len(robots) == 1
    observations = core.robot_body_observations(image, field, robots, **options)
    assert len(observations) == 1
    assert calls[0] == (image.shape, 64, 19)
    assert len(calls) == 2 and calls[1][1:] == (64, 19)
