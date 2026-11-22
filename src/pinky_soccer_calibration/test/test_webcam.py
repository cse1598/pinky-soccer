"""Local-camera rehearsal without DDS, fabricated hardware, or robot pose defaults."""

import math
from queue import Empty
import threading
import time
from types import SimpleNamespace

import numpy as np
import cv2
import pytest
import rclpy
import yaml

from pinky_soccer_calibration.ball_review import GameplayReview, select_intrinsics
from pinky_soccer_calibration.board_geometry import MECHANICAL_FROM_OPTICAL, rpy_matrix
from pinky_soccer_calibration.suite_model import Session
from pinky_soccer_calibration.suite_ui import App
from pinky_soccer_calibration.ui_options import parse_ui_options
from pinky_soccer_calibration.webcam_acquisition import WebcamAcquisition
from test_compressed_capture import board_image


class SyntheticSource:
    """Explicit test injection, never a runtime fallback when a camera is absent."""

    def __init__(self, frame=None):
        self.frame = board_image() if frame is None else frame
        self.sequence = 0
        self.closed = threading.Event()
        self.paused = False

    def get(self, timeout):
        if self.closed.wait(.025) or self.paused:
            time.sleep(.02)
            raise Empty
        self.sequence += 1
        return 'frame', self.sequence, time.monotonic_ns(), time.time_ns(), self.frame.copy()

    def close(self):
        self.closed.set()


def test_cli_webcam_is_opt_in_and_preserves_ros_arguments():
    options, args = parse_ui_options(['-webcam', '--ros-args', '-p', 'use_sim_time:=false'])
    assert options.webcam == 0 and options.webcam_hfov_deg is None
    assert args == ['--ros-args', '-p', 'use_sim_time:=false']
    assert parse_ui_options(['-webcam', '/dev/video2'])[0].webcam == '/dev/video2'
    assert parse_ui_options([])[0].webcam is None
    for args in (['-webcam', 'http://camera'], ['--webcam-width', '800'],
                 ['-webcam', '--webcam-fps', 'nan'], ['-webcam', '--webcam-hfov-deg', '180']):
        with pytest.raises(SystemExit):
            parse_ui_options(args)


def test_launch_passes_webcam_only_as_startup_arguments(monkeypatch):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    path = Path(__file__).parents[1]/'launch/calibration_ui.launch.py'
    spec = importlib.util.spec_from_file_location('webcam_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(webcam='true', webcam_device='2', webcam_width='800',
                                         webcam_height='600', webcam_fps='10', webcam_hfov_deg='',
                                         use_sim_time='false')
    action = module.start(context)[0]
    assert action['arguments'][:2] == ['-webcam', '2']
    context.launch_configurations['use_sim_time'] = 'true'
    with pytest.raises(ValueError, match='시스템 시각'):
        module.start(context)


def test_local_capture_and_recording_require_no_ros_context(tmp_path, monkeypatch):
    monkeypatch.setattr(rclpy, 'init', lambda *args, **kwargs: pytest.fail('Webcam must not initialize DDS'))
    source = SyntheticSource()
    camera = WebcamAcquisition(source=source)
    session = Session(tmp_path, 'rehearsal', 'webcam:0')
    try:
        camera.bind_project(session, creating=True)
        camera.begin(session)
        before = time.monotonic_ns()
        sample = camera.capture_raw()
        assert sample.metadata['read_complete_monotonic_ns'] > before
        assert sample.metadata['source'] == 'local_webcam'
        assert np.array_equal(sample.frame, source.frame)
        assert sample.image.header == sample.camera_info.header
        assert not hasattr(camera, 'publisher') and not hasattr(camera, 'tf_listener')
        session.save_raw_capture(sample.frame, sample.metadata)
        camera.record_capture(sample, session)
        session.capture(sample.frame, 'training', sample.metadata)
        camera.end()
        assert (session.root/'raw_bag/metadata.yaml').exists()
        assert 'local_webcam' in (session.root/'observations.csv').read_text()
        assert session.meta['image_acquisition']['network_transport'] is False
        root = session.root
        session.release()
        opened = Session.open(root)
        camera.bind_project(opened)
        assert opened.meta['image_source']['kind'] == 'webcam'
        assert len(opened.frames['training']) == 1
        camera.begin(opened)
        camera.end()
        assert len(opened.meta['recordings']) == 2
        opened.release()
        with pytest.raises(ValueError, match='주행'):
            camera.start_motion('forward')
    finally:
        camera.close()
        session.release()
    assert source.closed.is_set()


def test_source_identity_and_rehearsal_profiles_are_not_robot_calibration(tmp_path):
    camera = WebcamAcquisition(source=SyntheticSource())
    session = Session(tmp_path, 'rehearsal', 'webcam:0')
    try:
        camera.bind_project(session, creating=True)
        session.results['camera_mount'] = {'test_only': True}
        profile = yaml.safe_load(session.export(True).read_text())
        assert profile['environment'] == 'rehearsal' and profile['image_source']['kind'] == 'webcam'
        normal = App.__new__(App)
        normal.node = SimpleNamespace()
        with pytest.raises(ValueError, match='-webcam'):
            normal.bind_source(session)
        from pinky_soccer_bringup.calibration_profile import read_profile
        with pytest.raises(ValueError, match='Rehearsal'):
            read_profile(session.root/'profile.yaml', False)
        session.meta['environment'] = 'real'
        with pytest.raises(ValueError, match='rehearsal'):
            camera.bind_project(session)
        with pytest.raises(ValueError, match='rehearsal'):
            session.export(True)
        session.meta['environment'] = 'rehearsal'
        session.meta['image_source']['device'] = 'another_camera'
        with pytest.raises(ValueError, match='장치/해상도'):
            camera.bind_project(session)
    finally:
        camera.close()
        session.release()


def test_webcam_range_uses_measured_floor_height_tilt_only(tmp_path):
    camera = WebcamAcquisition(source=SyntheticSource())
    project = Session(tmp_path, 'rehearsal', 'webcam:0')
    try:
        camera.bind_project(project, creating=True)
        assert camera.review_transform(camera.floor_frame, camera.optical_frame) is None
        k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
        project.camera = k, np.zeros(5), (640, 480), 30.
        project.results['camera'] = {'passed': True}
        project.results['camera_mount'] = {'height_m': {'mean': .18}, 'roll_rad': {'mean': .03},
                                           'pitch_rad': {'mean': .44}}
        rotation, translation = camera.review_transform(camera.floor_frame, camera.optical_frame)
        assert rotation == pytest.approx(rpy_matrix(.03, .44, 0.) @ MECHANICAL_FROM_OPTICAL)
        assert translation == pytest.approx([0., 0., .18])
        assert camera.review_transform('base_link', camera.optical_frame) is None
        assert camera.review_transform('odom', camera.optical_frame) is None
        assert select_intrinsics((480, 640, 3), None, None, None, False, '')[0] is None
        project.stale.add('floor')
        assert camera.review_transform(camera.floor_frame, camera.optical_frame) is None
    finally:
        camera.close()
        project.release()


def test_no_cached_shutter_or_fake_frame_on_source_failure():
    source = SyntheticSource()
    camera = WebcamAcquisition(source=source)
    try:
        source.paused = True
        time.sleep(.08)
        camera.capture_timeout = .1
        with pytest.raises(ValueError, match='시간 초과'):
            camera.capture_raw()
    finally:
        camera.close()
    class Failed:
        def get(self, timeout):
            return 'error', 'explicit missing camera'
        def close(self):
            pass
    with pytest.raises(ValueError, match='explicit missing camera'):
        WebcamAcquisition(source=Failed())


def test_webcam_intrinsics_floor_pose_and_ball_range_end_to_end(tmp_path):
    """Synthetic camera with known geometry exercises the complete local rehearsal path."""
    from test_intrinsics import draw_board, model
    k = np.array(model()['camera_matrix']['data']).reshape(3, 3)
    source = SyntheticSource(cv2.cvtColor(draw_board(0), cv2.COLOR_GRAY2BGR))
    camera = WebcamAcquisition(source=source)
    project = Session(tmp_path, 'rehearsal', 'webcam:0')
    try:
        camera.bind_project(project, creating=True)
        for category, count in [('training', 20), ('validation', 14)]:
            for index in range(60):
                source.frame = cv2.cvtColor(draw_board(index, category == 'validation'), cv2.COLOR_GRAY2BGR)
                sample = camera.capture_raw()
                try:
                    project.capture(sample.frame, category, sample.metadata)
                except ValueError:
                    continue
                if len(project.frames[category]) == count:
                    break
            assert len(project.frames[category]) == count
        assert project.recalculate('camera')['passed']
        height, pitch = .25, .5
        rotation = (rpy_matrix(0., pitch, .1) @ MECHANICAL_FROM_OPTICAL).T
        rv = cv2.Rodrigues(rotation)[0]
        translation = -rotation @ np.array([-.35, .03, height])
        floor = np.full((600, 800), 170, np.uint8)
        for row in range(7):
            for col in range(10):
                square = np.array([[col-1, row-1, 0], [col, row-1, 0],
                                   [col, row, 0], [col-1, row, 0]], float)*.03
                pixels = cv2.projectPoints(square, rv, translation, k, np.zeros(5))[0]
                cv2.fillConvexPoly(floor, np.round(pixels).astype(np.int32), 255 if (row+col)%2 else 0)
        source.frame = cv2.cvtColor(cv2.GaussianBlur(floor, (3, 3), .6), cv2.COLOR_GRAY2BGR)
        for _ in range(5):
            sample = camera.capture_raw()
            project.floor_capture(sample.frame, provenance=sample.metadata)
        mount = project.results['camera_mount']
        assert mount['height_m']['mean'] == pytest.approx(height, abs=.008)
        assert mount['pitch_rad']['mean'] == pytest.approx(pitch, abs=.025)
        engine = GameplayReview()
        engine.settings.update(floor_frame=camera.floor_frame, output_frame=camera.floor_frame)
        engine.stable_frames = (camera.floor_frame,)
        ball = np.array([[.45, .02, engine.settings['ball_radius']]])
        rotation = (rpy_matrix(0., pitch, 0.) @ MECHANICAL_FROM_OPTICAL).T
        pixels = cv2.projectPoints(ball, cv2.Rodrigues(rotation)[0],
                                   -rotation @ np.array([0., 0., height]), k, np.zeros(5))[0]
        source.frame = np.zeros((600, 800, 3), np.uint8)
        cv2.circle(source.frame, tuple(np.round(pixels[0, 0]).astype(int)), 24, (0, 140, 255), -1)
        camera.capture_raw()
        frame, header, info, key = camera.review_frame()
        now = camera.get_clock().now().nanoseconds*1e-9
        _, report = engine.process(frame, key, now, now, header, camera.review_transform,
                                    session=project, info=info, approximate=False)
        assert report['frame'] == 'webcam_floor' and report['point'] is not None
        assert report['range_m'] == pytest.approx(math.hypot(.45, .02), abs=.02)
        assert not report['approximate'] and report['tracking']['stable_frame'] == 'webcam_floor'
        root = project.root
        project.export(True)
        project.release()
        reopened = Session.open(root)
        camera.bind_project(reopened)
        assert reopened.results['camera_mount']['height_m']['mean'] == mount['height_m']['mean']
        reopened.release()
    finally:
        camera.close()
        project.release()
