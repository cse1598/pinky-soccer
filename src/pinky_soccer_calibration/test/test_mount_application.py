"""Measured camera mounting must affect review and exported launch geometry."""

from types import SimpleNamespace
import xml.etree.ElementTree as ET

import cv2

import numpy as np

from pinky_soccer_bringup.calibration_profile import configure_urdf
from pinky_soccer_bringup.camera_mount import apply_mount, fixed_transforms, OPTICAL, rotation_rpy
from pinky_soccer_calibration.ball_review import gameplay_settings, GameplayReview
from pinky_soccer_calibration.suite_model import Session

import pytest


def summary():
    """Return synthetic, unverified measurements without pretending full XYZ is known."""
    return {'height_m': {'mean': .06}, 'roll_rad': {'mean': .01},
            'pitch_rad': {'mean': .02}, 'repeats': 4, 'repeatability_passed': False}


def test_review_uses_project_mount_and_keeps_output_frame_height():
    """Project a known floor ball despite an intentionally wrong runtime mounting TF."""
    cfg, params = gameplay_settings()
    engine = GameplayReview(cfg, params)
    old = (rotation_rpy(0, .14, 0) @ OPTICAL, np.array([.03, .004, .064]))
    corrected = apply_mount(old, summary())
    target = np.array([.35, .004, cfg['ball_radius']])
    optical = corrected[0].T @ (target-corrected[1])
    k = np.array([[550., 0, 320], [0, 550., 240], [0, 0, 1]])
    uv = (k @ optical)[:2]/optical[2]
    image = np.full((480, 640, 3), 45, np.uint8)
    cv2.circle(image, tuple(np.rint(uv).astype(int)), 18, (15, 100, 245), -1)
    project = SimpleNamespace(results={'camera_mount': summary()}, stale=set(), camera=None)

    def lookup(target_frame, source, stamp):
        if target_frame == cfg['floor_frame']:
            return old
        if target_frame == cfg['output_frame']:
            return np.eye(3), np.array([0., 0., -.028])
        return None

    _, report = engine.process(image, 1, 10, 10, SimpleNamespace(frame_id='camera', stamp=None),
                               lookup, project, external=None, approximate=True, hfov=60.4)
    # Use exact K next, independently of the fallback HFOV approximation.
    project.camera = (k, np.zeros(5), (640, 480), 30.)
    project.results['camera'] = {'passed': True}
    _, report = engine.process(image, 2, 11, 11, SimpleNamespace(frame_id='camera', stamp=None),
                               lookup, project)
    assert report['point'] == pytest.approx([.35, .004, -.00665], abs=.003)
    assert '보정 높이' in report['mount_source']
    assert '미완료' in report['mount_source']
    assert old[1][2] == .064
    project.stale.add('floor')
    _, stale = engine.process(image, 3, 12, 12, SimpleNamespace(frame_id='camera', stamp=None),
                              lookup, project)
    assert stale['point'][0] < .25
    _, external = engine.process(
        image, 4, 13, 13, SimpleNamespace(frame_id='camera', stamp=None), lookup, project,
        external=(k, np.zeros(5), (640, 480)), external_mount=summary())
    assert external['point'] == pytest.approx([.35, .004, -.00665], abs=.003)


def test_exported_partial_mount_updates_nested_urdf_without_guessing_xy_yaw(tmp_path):
    """Real/sim launch use the measured height/tilt and the original model XY/yaw."""
    session = Session(tmp_path, 'real', 'test-camera')
    session.results['camera_mount'] = summary()
    path = session.export(True)
    urdf = """<robot><link name="base_footprint"/><link name="base_link"/>
    <link name="mount"/><link name="front_camera_link"/>
    <joint name="base" type="fixed"><parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 .028"/></joint>
    <joint name="holder" type="fixed"><parent link="base_link"/><child link="mount"/>
    <origin xyz=".02 .003 .05" rpy="0 .14 .2"/></joint>
    <joint name="front_camera_fixed_joint" type="fixed"><parent link="mount"/>
    <child link="front_camera_link"/><origin xyz=".01 0 -.01"/></joint></robot>"""
    before = fixed_transforms(ET.fromstring(urdf), 'base_footprint')['front_camera_link']
    for simulation in (False, True):
        after = fixed_transforms(ET.fromstring(configure_urdf(urdf, path, simulation)),
                                 'base_footprint')['front_camera_link']
        assert after[:2, 3] == pytest.approx(before[:2, 3])
        assert after[2, 3] == pytest.approx(.06)
        assert after[:3, :3] == pytest.approx(rotation_rpy(.01, .02, .2))
    session.release()
