"""JPEG preview stays separate from original service photographs and recorded provenance."""

import hashlib
import queue
import threading
import time

import cv2
import numpy as np
from pinky_soccer_bringup.camera_publisher import CameraPublisher
from pinky_soccer_bringup.camera_support import image_messages
from pinky_soccer_calibration.camera_capture import decode_capture
from pinky_soccer_calibration.suite_model import Session
from pinky_soccer_calibration.suite_ros import (
    Acquisition, CAPTURE_IMAGE_TOPIC, CAPTURE_INFO_TOPIC, PREVIEW_TOPIC)
from pinky_soccer_calibration.suite_ui import App
from pinky_soccer_msgs.srv import CaptureFrame
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image
import yaml


def board_image():
    frame = np.full((480, 640, 3), 180, np.uint8)
    for row in range(7):
        for col in range(10):
            x, y = 70+40*col, 65+40*row
            color = (255, 255, 255) if (row+col) % 2 else (0, 0, 0)
            cv2.rectangle(frame, (x, y), (x+40, y+40), color, -1)
    frame[440:450, :256] = np.arange(256, dtype=np.uint8)[None, :, None]
    return frame


@pytest.fixture
def connected(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=175)
    source = board_image()

    class Camera:
        def start(self, width, height):
            assert (width, height) == (640, 480)

        def get_frame(self):
            return source.copy()

        def close(self):
            pass

    server = CameraPublisher(Camera, parameter_overrides=[Parameter('fps', value=10.0)])
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    client = Acquisition()
    try:
        deadline = time.monotonic()+5
        while not client.fresh(PREVIEW_TOPIC) and time.monotonic() < deadline:
            time.sleep(.02)
        assert client.fresh(PREVIEW_TOPIC)
        assert server.raw_pub.get_subscription_count() == 0
        yield client, server, source
    finally:
        client.close()
        executor.shutdown()
        worker.join(timeout=2)
        server.destroy_node()
        rclpy.shutdown()


def app_for(node, session):
    app = App.__new__(App)
    app.node, app.session, app.events = node, session, queue.Queue()
    app.inputs = {'columns': '9', 'rows': '6', 'square': '30'}
    app.inputs.update(floor_columns='9', floor_rows='6', floor_square='30')
    app.options = {'floor_confirm': True}
    return app


def test_jpeg_preview_and_raw_shutter_png_bag_csv(connected, tmp_path):
    node, camera, original = connected
    preview = node.frame()
    assert preview.shape == original.shape
    assert not np.array_equal(preview, original)  # Preview is actually JPEG-decoded.
    session = Session(tmp_path, 'rehearsal', 'jpeg-preview')
    node.begin(session)
    app = app_for(node, session)
    assert '원본 1장' in app.capture('training')
    assert camera.raw_pub.get_subscription_count() == 0
    assert '/camera/image_raw' not in node.latest
    photo = session.frames['training'][0]
    saved = cv2.imread(str(photo))
    assert np.array_equal(saved, original)
    provenance = yaml.safe_load(photo.with_suffix('.capture.yaml').read_text())
    preserved = cv2.imread(str(session.root/provenance['raw_image_file']))
    assert np.array_equal(preserved, original)
    assert provenance['format'] == 'raw'
    assert provenance['pixel_sha256'] == hashlib.sha256(original.tobytes()).hexdigest()
    assert provenance['camera_info']['width'] == 640
    assert provenance['camera_info']['header']['frame_id'] == 'front_camera_optical_frame'
    assert session.meta['image_acquisition']['raw_recording'] == 'service_captures_only'
    node.end()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session.root/'raw_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    captured, info, jpeg_count = [], [], 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == CAPTURE_IMAGE_TOPIC:
            captured.append(deserialize_message(data, Image))
        if topic == CAPTURE_INFO_TOPIC:
            info.append(deserialize_message(data, CameraInfo))
        if topic == PREVIEW_TOPIC:
            jpeg_count += 1
    assert len(captured) == len(info) == 1 and jpeg_count > 0
    assert captured[0].header == info[0].header
    assert bytes(captured[0].data) == original.tobytes()
    assert CAPTURE_IMAGE_TOPIC in (session.root/'observations.csv').read_text()


def test_failed_service_does_not_use_available_preview(connected, tmp_path):
    node, camera, _ = connected
    session = Session(tmp_path, 'rehearsal', 'timeout')
    node.begin(session)
    app = app_for(node, session)
    node.capture_timeout = .15
    complete = camera.capture_service.complete
    camera.capture_service.complete = lambda *args: None  # Stream alive, no fresh service shot.
    try:
        assert node.frame().shape == (480, 640, 3)
        with pytest.raises(ValueError, match='Timed out'):
            app.capture('training')
        assert not session.frames['training']
        assert not (session.root/'captures').exists()
        assert camera.raw_pub.get_subscription_count() == 0
    finally:
        camera.capture_service.complete = complete


def test_rejected_checker_shot_still_preserves_service_original(connected, tmp_path):
    node, camera, original = connected
    session = Session(tmp_path, 'rehearsal', 'rejected')
    node.begin(session)
    app = app_for(node, session)
    app.inputs['columns'] = '11'  # Actual board has nine inner corners.
    with pytest.raises(ValueError, match='내부 코너'):
        app.capture('training')
    assert not session.frames['training']
    files = list((session.root/'captures').glob('*.png'))
    assert len(files) == 1 and np.array_equal(cv2.imread(str(files[0])), original)
    assert node.latest[CAPTURE_IMAGE_TOPIC].encoding == 'bgr8'


def test_floor_shutter_uses_service_original_and_provenance(connected, tmp_path):
    node, _, original = connected
    session = Session(tmp_path, 'rehearsal', 'floor-route')
    node.begin(session)
    app = app_for(node, session)
    app.inputs.update(floor_columns='7', floor_rows='5', floor_square='24.5')
    observed = {}

    def floor_capture(frame, registration, provenance):
        observed.update(frame=frame, registration=registration, provenance=provenance)
        return {'repeats': 1}

    session.floor_capture = floor_capture  # Pose fitting itself has separate geometry tests.
    assert app.pose()['repeats'] == 1
    assert np.array_equal(observed['frame'], original)
    assert observed['registration'] is None
    assert observed['provenance']['format'] == 'raw'
    assert session.board.pattern == (9, 6)
    assert session.floor_board.pattern == (7, 5)
    assert session.floor_board.square_mm == 24.5


@pytest.mark.parametrize('failure', ['jpeg', 'header', 'size'])
def test_wrong_service_payload_is_rejected(failure):
    from builtin_interfaces.msg import Time
    frame = board_image()
    raw, jpeg, info = image_messages(frame, Time(sec=5), 'camera',
                                     CameraInfo(width=640, height=480))
    response = CaptureFrame.Response(success=True, image=raw, camera_info=info)
    if failure == 'jpeg':
        response.image = Image()
        response.compressed_image = jpeg
    elif failure == 'header':
        response.camera_info.header.stamp.sec = 6
    else:
        response.camera_info.width = 320
    with pytest.raises(ValueError, match='응답'):
        decode_capture(response, '/camera/capture_frame', 0, time.monotonic(), False)
