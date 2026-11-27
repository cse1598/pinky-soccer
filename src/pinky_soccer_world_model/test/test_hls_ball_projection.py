"""Keep distance/K/D/TF math frozen while replacing pixel evidence and offset wiring."""
import importlib.util
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
import numpy as np
import pytest
import yaml

from pinky_soccer_world_model import ball_detector as module
from pinky_soccer_world_model.ball_offset import apply_ball_offset, load_ball_offset
from pinky_soccer_world_model.ball_vision import project_ball
from pinky_soccer_world_model.robust_ball import BallImageDetector, observe_ball

ROOT = Path(__file__).resolve().parents[3]
OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
K = np.array([[500., 0., 319.5], [0., 500., 239.5], [0., 0., 1.]])


def camera():
    c, s = math.cos(.1562956126), math.sin(.1562956126)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])@OPTICAL, np.array([.02, 0., .059608382])


def image(center=(320, 350), radius=35):
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(frame, center, radius, (4, 57, 230), -1)
    return frame


@pytest.mark.parametrize('center', [(5, 300), (-5, 300), (635, 300), (320, 475), (320, 125), (320, 140)])
def test_crop_and_roi_arcs_restore_full_image_uv(center):
    frame = image(center)
    before = frame.copy()
    result = observe_ball(frame, [])
    assert result.distance_valid and result.bearing_valid
    assert result.roi_top == 130
    assert not result.mask[:130].any()
    assert result.blob[:3] == pytest.approx((*center, 35), abs=1.)
    assert result.partial
    np.testing.assert_array_equal(frame, before)


def test_short_arc_is_direction_only_without_a_fabricated_radius():
    result = observe_ball(image((-25, 300)), [])
    assert result.bearing_valid and result.bearing_uv is not None
    assert not result.distance_valid and result.blob is None
    assert project_ball(result.blob, (480, 640), K, np.zeros(5), camera(),
                        (np.eye(3), np.zeros(3)), .02135, 2.5) is None


@pytest.mark.parametrize('rotated', [False, True])
def test_old_distance_function_is_exactly_preserved_for_same_uvr_and_distortion(rotated):
    path = ROOT/'docs/validation/ball_hls_20261127/baseline_ball_vision.py'
    spec = importlib.util.spec_from_file_location('pinky_soccer_world_model.baseline_ball_vision', path)
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    assert inspect.getsource(old.project_ball) == inspect.getsource(project_ball)
    output = (np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]), np.array([.3, -.1, -.028]))
    for distortion in (np.zeros(5), np.array([-.12, .03, .001, -.002, .01])):
        for u in (0., 120., 320., 639.):
            for v in (130., 220., 350., 479.):
                for radius_px in (5., 35., 100.):
                    args = ((u, v, radius_px, 100.), (480, 640), K, distortion,
                            camera(), output, .02135, 2.5, rotated)
                    a, b = old.project_ball(*args), project_ball(*args)
                    if a is None:
                        assert b is None
                    else:
                        np.testing.assert_array_equal(a, b)


def test_pixel_radius_is_not_used_as_metric_radius_or_diameter():
    points = [project_ball((320., 350., r, 100.), (480, 640), K, np.zeros(5),
                          camera(), (np.eye(3), np.zeros(3)), .02135, 2.5) for r in (5., 35., 150.)]
    np.testing.assert_array_equal(points[0], points[1])
    np.testing.assert_array_equal(points[0], points[2])
    assert points[0][2] == pytest.approx(.02135)  # physical radius in metres, not .0427


def test_offset_uses_base_axes_after_projection_and_never_subtracts_bumper_twice():
    front = np.array([.0458, .01, 0.])
    for actual, raw in ((.1, .087), (.2, .187)):
        point = front+np.array([raw, 0., .02135])
        corrected = apply_ball_offset(point, [.013, 0.])
        assert (corrected-front)[0] == pytest.approx(actual)
        assert corrected[2] == point[2]
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    np.testing.assert_allclose(apply_ball_offset([1., 2., 3.], [.013, 0.], rotation), [1., 2.013, 3.])


def test_reviewed_profile_selection_environment_and_immutable_zero_history(tmp_path):
    profile = dict(schema_version=1, human_verified=True, environment='real',
                   ball_review=dict(offset_xy_m=[.013, 0.]))
    path = tmp_path/'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    camera_file = tmp_path/'camera.yaml'
    camera_file.write_text('camera_matrix: {}\n')
    before = path.read_bytes()
    assert load_ball_offset(camera_file)[0] == pytest.approx([.013, 0.])
    assert path.read_bytes() == before
    with pytest.raises(ValueError):
        load_ball_offset(path, simulation=True)
    profile['human_verified'] = False
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError):
        load_ball_offset(path)
    assert load_ball_offset('none')[0] == pytest.approx([0., 0.])


@pytest.mark.parametrize('partial', [False, True])
def test_runtime_applies_one_offset_or_publishes_bearing_only(partial, monkeypatch):
    frame = image((-25, 300) if partial else (320, 350))
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
    msg.header.frame_id, msg.header.stamp.sec = 'optical', 42
    output = np.eye(3), np.array([0., 0., -.028])
    points, statuses, calls = [], [], []
    monkeypatch.setattr(module, 'transform_matrix', lambda value: value)
    monkeypatch.setattr(module, 'apply_ball_offset', lambda p, delta, r: calls.append(p.copy())
                        or apply_ball_offset(p, delta, r))
    n = SimpleNamespace(K=K, D=np.zeros(5), debug_pub=None, allow_uncalibrated=False,
        visual=BallImageDetector([]), approximate=False, bridge=bridge,
        radius=.02135, max_range=2.5, rotated=False, floor_frame='base_footprint',
        output_frame='base_link', offset_xy=np.array([.013, 0.]), offset_source='reviewed fixture',
        tf=SimpleNamespace(lookup_transform=lambda target, source, *a, **k:
                           camera() if source == 'optical' else output),
        pub=SimpleNamespace(publish=points.append), status_pub=SimpleNamespace(publish=statuses.append))
    module.BallDetector.on_image(n, msg)
    status = json.loads(statuses[-1].data)
    if partial:
        assert not points and not calls and status['bearing_valid'] and not status['range_valid']
    else:
        assert len(points) == len(calls) == 1
        assert points[0].point.x == pytest.approx(calls[0][0]+.013)
        assert status['range_valid'] and status['circle_valid']
