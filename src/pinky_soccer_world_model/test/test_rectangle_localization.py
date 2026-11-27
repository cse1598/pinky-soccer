"""Rectangular geometry, interior rejection and camera/IMU symmetry decisions."""
import math

import cv2
import numpy as np
import pytest
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_world_model.geometry import wrap
from pinky_soccer_world_model.heading_reference import goal_blobs, mirrored
from pinky_soccer_world_model.rectangle_localization import (
    fit_rectangle, GoalImuReferences, StartupGate)


def wall_points(pose=(.2, -.1, .37), noise=.003, clutter=True):
    rng = np.random.default_rng(7)
    points = np.vstack([np.c_[np.linspace(-.91, .91, 180), np.full(180, y)] for y in (-.61, .61)]+
                       [np.c_[np.full(140, x), np.linspace(-.61, .61, 140)] for x in (-.91, .91)])
    points += rng.normal(0, noise, points.shape)
    if clutter:
        points = np.r_[points, rng.uniform([-.45, -.3], [.45, .3], (150, 2))]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (points-pose[:2]) @ np.array([[c, -s], [s, c]])


@pytest.mark.parametrize('theta', [.37, -1.55, 2.8])
def test_four_walls_reject_internal_points_and_leave_original_unchanged(theta):
    pose = (.2, -.1, theta)
    p = wall_points(pose)
    before = p.copy()
    result = fit_rectangle(p, 1.82, 1.22)
    assert result is not None
    best = min(result.hypotheses, key=lambda q: abs(wrap(q[2]-theta)))
    assert np.linalg.norm(np.asarray(best[:2])-pose[:2]) < .01
    assert abs(wrap(best[2]-theta)) < .01
    assert result.mask[-150:].sum() == 0
    assert result.mask[:640].sum() > 620
    assert np.array_equal(p, before)
    tracked = fit_rectangle(p, 1.82, 1.22, prior_yaw=result.pose[2]+.05)
    assert tracked is not None and tracked.residual < .008


def test_hidden_wall_cannot_fabricate_a_pose():
    p = wall_points(clutter=False, noise=0)
    assert fit_rectangle(p[:500], 1.82, 1.22) is None
    assert fit_rectangle(np.full((100, 2), np.nan), 1.82, 1.22) is None


def test_two_colours_save_raw_angles_but_use_geometry_corrected_axes():
    field = Field.from_file()
    refs = GoalImuReferences(field)
    offset = 2.9
    for color in ('blue', 'yellow'):
        a, b = field.goal_back(color)
        gx, gy = (a[0]+b[0])/2, (a[1]+b[1])/2
        # Off-centre robot and off-axis panel: the two raw yaw observations are NOT pi apart.
        bearing = .18 if color == 'blue' else -.14
        theta = wrap(math.atan2(gy-.3, gx-.4)-bearing)
        pose = (.4, .3, theta)
        for i in range(4):
            refs.observe((pose, mirrored(pose)), color, bearing, wrap(theta+offset),
                         i+(0 if color == 'blue' else 10))
    assert refs.complete
    assert abs(wrap(refs.references['blue']['blue_axis_imu_yaw']-offset)) < 1e-8
    for theta in (-3.1, -1., 0., 1., 3.1):
        pose = (.4, .3, theta)
        selected = refs.select((mirrored(pose), pose), wrap(theta+offset))
        assert selected['branch'] == 1
        assert np.allclose(selected['pose'], pose)
    assert refs.select((pose, mirrored(pose)), wrap(theta+offset+.8)) is None


def test_duplicate_images_and_wrong_goal_bearings_do_not_complete_reference():
    refs = GoalImuReferences(Field.from_file())
    pose = (0., 0., 0.)
    for _ in range(20):
        refs.observe((pose, mirrored(pose)), 'blue', 0., 0., 1.)
    assert len(refs.samples['blue']) == 1
    assert not refs.complete
    for i in range(10):
        refs.observe((pose, mirrored(pose)), 'yellow', .9, 0., 10.+i)
    assert not refs.references


def test_cyan_blue_and_yellow_panels_are_recognized_without_green():
    ranges = {'blue': ([78, 90, 45], [135, 255, 255]),
              'yellow': ([20, 90, 60], [38, 255, 255])}
    for color, bgr in [('blue', (255, 255, 0)), ('yellow', (0, 255, 255))]:
        frame = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(frame, (100, 100), (220, 125), bgr, -1)
        assert [b['color'] for b in goal_blobs(frame, ranges)] == [color]
    frame[:] = (0, 255, 0)
    assert not goal_blobs(frame, ranges)


def test_gate_never_moves_on_stale_inputs_or_after_losing_localization():
    gate = StartupGate()
    assert gate.command(False, False, True, True, True)[:2] == (0., 0.)
    assert gate.command(False, True, True, True, True) == (0., .3, 'SEEKING_GOALS')
    assert gate.command(False, True, True, True, False)[:2] == (0., 0.)
    assert gate.command(True, True, True, True, True)[2] == 'READY'
    assert gate.command(False, True, True, True, True) == (0., 0., 'LOCALIZATION_LOST')
    gate.aborted = True
    assert gate.command(True, True, True, True, True) == (0., 0., 'ABORTED')
