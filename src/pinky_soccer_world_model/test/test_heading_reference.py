"""Independent geometric, magnetic interference and 180-degree ambiguity checks."""

import math

import cv2
import numpy as np
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_world_model.geometry import wrap
from pinky_soccer_world_model.heading_reference import (
    HeadingReference, goal_blobs, goal_error, magnetic_heading, mirrored)
import pytest


@pytest.fixture
def field():
    return Field.from_file()


def bearing(field, pose, color):
    a, _ = field.goal_back(color)
    return wrap(math.atan2(-pose[1], a[0]-pose[0])-pose[2])


@pytest.mark.parametrize('color,pose', [('blue', (.3, .2, -.4)),
                                      ('yellow', (-.2, -.3, 2.7))])
def test_visual_disambiguates_whole_pose_and_records_compass(field, color, pose):
    reference = HeadingReference(field)
    wrong = mirrored(pose)
    observed = bearing(field, pose, color)
    assert goal_error(field, pose, color, observed) < 1e-8
    for _ in range(4):
        assert reference.visual(wrong, color, observed, pose[2]-.7, 48e-6) is None
    result = reference.visual(wrong, color, observed, pose[2]-.7, 48e-6)
    assert result['flip']
    assert result['pose'] == pytest.approx(pose)
    assert reference.offset == pytest.approx(.7)
    # Once the coloured panel is hidden, the recorded compass reference rejects
    # the symmetric candidate at a different position and heading as well.
    moved = (-.1, .1, .9)
    for _ in range(4):
        assert reference.magnetic(mirrored(moved), moved[2]-.7, 48e-6) is None
    assert reference.magnetic(mirrored(moved), moved[2]-.7, 48e-6)['pose'] == pytest.approx(moved)
    assert reference.magnetic(moved, moved[2]-.7, 48e-6) is None


def test_color_does_not_assume_goal_straight_ahead(field):
    r = HeadingReference(field)
    pose = (.1, .4, .2)
    # Off-axis viewing differs from saying that a blue goal means yaw=0.
    observed = bearing(field, pose, 'blue')
    assert abs(observed) > .2
    for _ in range(5):
        result = r.visual(pose, 'blue', observed, -.6, 45e-6)
    assert result is not None and not result['flip']
    assert r.offset == pytest.approx(.8)
    for _ in range(10):
        assert r.magnetic(mirrored(pose), -.6, 85e-6) is None


def test_reject_inconsistent_visual_and_offset_samples(field):
    r = HeadingReference(field)
    for i in range(10):
        assert r.visual((0, 0, 0), 'blue', 1.5, i*.5, 45e-6) is None
    for i in range(10):
        assert r.visual((0, 0, 0), 'blue', 0., i*.5, 45e-6) is None
    assert r.offset is None


def test_magnetic_right_handed_yaw_and_bad_inputs():
    for angle in (0, .5, -2., math.pi):
        b = [22e-6*math.cos(angle), -22e-6*math.sin(angle), -42e-6]
        h, _ = magnetic_heading(b, [0, 0, 9.81])
        assert wrap(h-angle) == pytest.approx(0., abs=1e-8)
    for b, a in [([0, 0, 0], [0, 0, 9.81]), ([1, 0, 0], [0, 0, 9.81]),
                 ([22e-6, 0, -42e-6], [0, 0, 14]), ([float('nan'), 0, 0], [0, 0, 9.81])]:
        with pytest.raises(ValueError):
            magnetic_heading(b, a)


def test_compass_tilt_compensation_and_vertical_pose_rejection():
    r, p, yaw = .35, -.25, 1.2
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0],
                   [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    rotation = rz @ ry @ rx
    heading, _ = magnetic_heading(rotation.T @ [22e-6, 0., -42e-6],
                                   rotation.T @ [0., 0., 9.81])
    assert heading == pytest.approx(yaw)
    with pytest.raises(ValueError):
        magnetic_heading([22e-6, 0., -42e-6], [9.81, 0., 0.])


def test_goal_detection_rejects_ball_crop_and_duplicate_panels():
    ranges = {'blue': ([95, 90, 45], [135, 255, 255])}
    image = np.zeros((240, 320, 3), np.uint8)
    cv2.rectangle(image, (70, 90), (200, 115), (255, 0, 0), -1)
    assert len(goal_blobs(image, ranges)) == 1
    cv2.rectangle(image, (70, 140), (200, 165), (255, 0, 0), -1)
    assert not goal_blobs(image, ranges)
    image[:] = 0
    cv2.circle(image, (150, 100), 20, (255, 0, 0), -1)
    assert not goal_blobs(image, ranges)
    cv2.rectangle(image, (0, 180), (180, 210), (255, 0, 0), -1)
    assert not goal_blobs(image, ranges)
