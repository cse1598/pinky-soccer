"""Unknown-start ray scans, clutter/occlusion, HLS consensus and continuity."""
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from pinky_soccer_world_model.geometry import wrap
from pinky_soccer_world_model.goal_hls import HlsGoalDetector
from pinky_soccer_world_model.heading_reference import mirrored
from pinky_soccer_world_model.rectangle_localization import fit_rectangle
from pinky_soccer_world_model.rectangle_tracking import (
    GoalInitializer, RectangleTracker, goal_aims)


def scan_points(pose, noise=.003, obstacles=()):
    """Ray/rectangle/circle intersections are sensor fixtures, not estimator inputs."""
    a = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    directions = np.c_[np.cos(a+pose[2]), np.sin(a+pose[2])]
    with np.errstate(divide='ignore', invalid='ignore'):
        distances = np.array([(x-pose[0])/directions[:, 0] for x in (-.91, .91)]+
                             [(y-pose[1])/directions[:, 1] for y in (-.61, .61)])
    distances[distances <= 0] = np.inf
    ranges = distances.min(axis=0)
    for x, y, radius in obstacles:
        delta = np.asarray(pose[:2])-[x, y]
        b = directions@delta
        disc = b*b-float(delta@delta)+radius*radius
        t = -b-np.sqrt(np.maximum(0, disc))
        ranges = np.minimum(ranges, np.where((disc > 0) & (t > 0), t, np.inf))
    ranges += np.random.default_rng(123).normal(0, noise, len(a))
    return np.c_[ranges*np.cos(a), ranges*np.sin(a)]


POSES = ([(0., 0., 0.), (0., 0., math.pi), (0., 0., math.pi/2),
          (.3, -.2, -.7), (.6, .3, -2.8)]+
         [(x, y, yaw) for x in (-.72, .72) for y in (-.43, .43) for yaw in (-1.1, 2.9)])


@pytest.mark.parametrize('pose', POSES)
@pytest.mark.parametrize('obstacles', [(), ((0., .1, .065),), ((.81, .1, .065),)])
def test_unknown_start_known_rectangle_with_ray_occlusion(pose, obstacles):
    points = scan_points(pose, obstacles=obstacles)
    before = points.copy()
    fit = fit_rectangle(points, 1.82, 1.22)
    assert fit is not None
    best = min(fit.hypotheses, key=lambda p: abs(wrap(p[2]-pose[2])))
    assert math.dist(best[:2], pose[:2]) < .015
    assert abs(wrap(best[2]-pose[2])) < .015
    assert math.dist(fit.hypotheses[0][:2], [-v for v in fit.hypotheses[1][:2]]) < 1e-12
    assert np.array_equal(points, before)


def test_noisy_tracking_fast_path_and_invalid_or_missing_walls():
    pose = (.25, .15, 3.13)
    for noise in (.001, .006, .015):
        points = scan_points(pose, noise)
        points[::20] = np.nan
        fit = fit_rectangle(points, 1.82, 1.22, prior_yaw=pose[2]+.02)
        assert fit is not None
        assert math.dist(fit.pose[:2], pose[:2]) < .015
        assert abs(wrap(fit.pose[2]-pose[2])) < .02
    assert fit_rectangle(scan_points(pose)[:120], 1.82, 1.22) is None


def test_minimum_angle_is_not_minimum_distance_and_is_symmetry_invariant():
    pose = (.6, .3, 2.8)
    aims = goal_aims(pose, 1.82, (.04, .01), .13)
    chosen = min(aims, key=lambda g: abs(g['error']))
    assert chosen['sign'] == -1  # farther goal, smaller turn
    mirrored_aims = goal_aims(mirrored(pose), 1.82, (.04, .01), .13)
    other = min(mirrored_aims, key=lambda g: abs(g['error']))
    assert chosen['error'] == pytest.approx(other['error'])


@pytest.mark.parametrize('color', ['yellow', 'blue'])
@pytest.mark.parametrize('flip', [False, True])
def test_single_goal_corrects_off_axis_camera_and_derives_pi_axes(color, flip):
    sign = 1 if color == 'blue' else -1
    x, y, camera_yaw = .35, -.23, .19
    theta = wrap(math.atan2(-y, sign*.91-x)-camera_yaw)
    actual = (x, y, theta)
    provisional = mirrored(actual) if flip else actual
    init = GoalInitializer(1.82)
    init.step(provisional, 0., camera_yaw=camera_yaw)
    init.step(provisional, .6, camera_yaw=camera_yaw)
    assert init.state == 'READING_GOAL'
    imu = wrap(theta+2.98)
    chosen = init.identify(provisional, color, imu, 10.)
    assert np.allclose(chosen, actual)
    assert init.offset == pytest.approx(2.98)
    assert abs(wrap(init.anchors['yellow']['goal_axis_imu_yaw']-
                    init.anchors['blue']['goal_axis_imu_yaw'])) == pytest.approx(math.pi)
    assert abs(wrap(init.anchors[color]['goal_axis_imu_yaw']-imu)) > .05


def test_first_goal_failure_tries_opposite_and_both_fail_without_choosing_color():
    init = GoalInitializer(1.82, settle=.2, view_timeout=.3)
    init.step((0., 0., 0.), 0.)
    init.step((0., 0., 0.), .6)
    assert init.target == -1 and init.attempted == [1]
    init.step((0., 0., math.pi), 1.)
    init.step((0., 0., math.pi), 1.6)
    assert init.state == 'GOAL_COLOR_FAILED' and init.offset is None
    assert init.step((0., 0., math.pi), 2.) == 0.


def test_hls_crop_one_conversion_area_ratio_and_consecutive_frames(monkeypatch):
    detector = HlsGoalDetector()
    frame = np.zeros((480, 640, 3), np.uint8)
    frame[:130] = (4, 229, 237)
    assert detector.detect(frame)['color'] is None
    frame[200:220, 200:230] = (134, 75, 5)  # 600 blue pixels
    calls = []
    original = cv2.cvtColor
    monkeypatch.setattr(cv2, 'cvtColor', lambda a, c: calls.append(a.shape) or original(a, c))
    assert detector.detect(frame)['color'] is None
    assert detector.detect(frame)['color'] is None
    assert detector.detect(frame)['color'] == 'blue'
    assert calls == [(350, 640, 3)]*3
    frame[230:250, 200:225] = (4, 229, 237)  # ratio 600:500 is ambiguous
    assert detector.detect(frame)['color'] is None and detector.streak == 0
    frame[:] = 0
    frame[200:210, 200:249] = (4, 229, 237)  # 490 below minimum
    assert detector.detect(frame)['color'] is None


def test_tracking_wrap_drift_alias_rejection_occlusion_and_reinitialization():
    tracker = RectangleTracker()
    pose = (.1, -.2, 3.12)
    tracker.seed(pose, (0., 0., 3.12), wrap(pose[2]+.4), 0.)
    for i in range(1, 101):
        th = wrap(3.12+.01*i)
        imu = wrap(th+.4+.0001*i)
        tracker.predict((0., 0., th), imu, i*.1)
        candidate = (.1, -.2, th)
        assert tracker.correct((mirrored(candidate), candidate), imu, i*.1)
        assert math.dist(tracker.pose[:2], pose[:2]) < 1e-12
    assert tracker.offset > .4  # trusted walls slowly remove IMU drift
    assert not tracker.correct(((.8, .4, th), mirrored((.8, .4, th))), imu, 10.1)
    assert tracker.validity(10.5)[0]
    assert tracker.validity(10.9)[1] == 'LOCALIZATION_LOST'
    assert tracker.validity(13.1)[1] == 'REINITIALIZATION_REQUIRED'
    assert not tracker.correct((candidate, mirrored(candidate)), imu, 13.2)


@pytest.mark.parametrize('engine', ['mujoco', 'gazebo'])
@pytest.mark.parametrize('color', ['yellow', 'blue'])
def test_measured_renderer_override_on_saved_original_camera_frames(engine, color):
    root = Path(__file__).resolve().parents[3]
    path = root/'docs/validation/sim_colors_20261127'/f'{engine}_{color}.png'
    if not path.exists():
        pytest.skip('Saved validation images are repository artifacts')
    data = yaml.safe_load((Path(__file__).parents[1]/'config/rectangle_sim_hls.yaml').read_text())
    params = data['/**/rectangle_localization']['ros__parameters']
    detector = HlsGoalDetector(**{k.removeprefix('hls.'): v for k, v in params.items()})
    image = cv2.imread(str(path))
    for _ in range(3):
        result = detector.detect(image)
    assert result['color'] == color
