"""ROS graph tests using an injected camera, never motors or physical devices."""

import os
import signal
import subprocess
import time

import numpy as np
from pinky_soccer_bringup.camera_publisher import CameraPublisher
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class TestCamera:
    """Synthetic SDK replacement used exclusively by these runtime tests."""

    __test__ = False

    def __init__(self, fail_start=False):
        """Track camera lifecycle calls without opening hardware."""
        self.closed = False
        self.fail_start = fail_start

    def start(self, width, height):
        """Use a small synthetic image instead of a real device."""
        self.shape = (height, width, 3)
        if self.fail_start:
            raise RuntimeError('synthetic start failure')

    def get_frame(self):
        """Return a recognizable nonzero BGR frame."""
        return np.full(self.shape, 42, dtype=np.uint8)

    def close(self):
        """Remember that the device was released."""
        self.closed = True


@pytest.fixture
def ros_context(tmp_path, monkeypatch):
    """Isolate ROS logging and discovery from the user's robot graph."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    context = Context()
    rclpy.init(args=[], context=context, domain_id=89)
    yield context
    context.shutdown()


def overrides():
    """Return deliberately tiny test image settings."""
    return [Parameter('width', value=8), Parameter('height', value=6),
            Parameter('fps', value=20.0)]


def test_camera_topics_and_device_release(ros_context):
    """Exercise actual ROS publishers and raw/JPEG/CameraInfo synchronization."""
    device = TestCamera()
    camera = CameraPublisher(lambda: device, context=ros_context, parameter_overrides=overrides())
    receiver = Node('sensor_test_receiver', context=ros_context)
    received = {'raw': {}, 'jpeg': {}, 'info': {}}

    def store(kind):
        def callback(message):
            stamp = message.header.stamp
            received[kind][(stamp.sec, stamp.nanosec)] = message
        return callback

    for message_type, topic, kind in [
        (Image, '/camera/image_raw', 'raw'),
        (CompressedImage, '/camera/image_raw/compressed', 'jpeg'),
        (CameraInfo, '/camera/camera_info', 'info'),
    ]:
        receiver.create_subscription(message_type, topic, store(kind), qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(camera)
    executor.add_node(receiver)
    try:
        deadline = time.monotonic() + 6
        common = set()
        while not common and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            common = set(received['raw']) & set(received['jpeg']) & set(received['info'])
        assert common, 'No synchronized camera messages received over DDS'
        stamp = next(iter(common))
        raw, info = received['raw'][stamp], received['info'][stamp]
        assert bytes(raw.data) == bytes([42] * (8 * 6 * 3))
        assert raw.header.frame_id == 'front_camera_optical_frame'
        assert raw.header == info.header and info.k[0] == 0
    finally:
        executor.shutdown()
        camera.destroy_node()
        receiver.destroy_node()
    assert device.closed


def test_camera_start_failure_releases_partial_device(ros_context):
    """Startup failure must not leave the sensor busy for later launches."""
    device = TestCamera(fail_start=True)
    with pytest.raises(RuntimeError, match='start failure'):
        CameraPublisher(lambda: device, context=ros_context, parameter_overrides=overrides())
    assert device.closed


def test_bad_calibration_rejected_before_camera_open(ros_context):
    """A bad file must fail before hardware is touched."""
    opened = []
    with pytest.raises(ValueError, match='file:'):
        CameraPublisher(lambda: opened.append(True), context=ros_context,
                        parameter_overrides=overrides() + [
                            Parameter('camera_info_url', value='http://invalid/calibration.yaml')])
    assert not opened


def test_repeated_camera_failure_never_republishes_stale_images(ros_context):
    """Abort repeated read failures and release the owned device on teardown."""
    class FailedCamera(TestCamera):
        def get_frame(self):
            raise RuntimeError('synthetic read failure')

    device = FailedCamera()
    node = CameraPublisher(lambda: device, context=ros_context, parameter_overrides=overrides())
    try:
        node.publish_frame()
        node.publish_frame()
        assert node.failures == 2
        with pytest.raises(RuntimeError, match='three times'):
            node.publish_frame()
    finally:
        node.destroy_node()
    assert device.closed


def test_installed_robot_launch_description_only(ros_context):
    """Run the real entry point with every sensor flag true but hardware false."""
    receiver = Node('launch_gate_observer', context=ros_context)
    process = subprocess.Popen([
        'ros2', 'launch', 'pinky_soccer_bringup', 'robot.launch.py',
        'hardware:=false', 'camera_enabled:=true', 'imu_enabled:=true', 'adc_enabled:=true',
    ], env=dict(os.environ, ROS_DOMAIN_ID='89'), start_new_session=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10
        names = set()
        expected = {'robot_state_publisher', 'joint_state_publisher', 'wheel_joint_state_relay'}
        while time.monotonic() < deadline and process.poll() is None:
            names = set(receiver.get_node_names())
            if expected <= names:
                break
            time.sleep(.1)
        assert expected <= names, f'Description nodes not visible; process exit={process.poll()}'
        assert not {'soccer_camera', 'soccer_imu_driver', 'soccer_adc_driver'} & names
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=3)
        receiver.destroy_node()
    assert 'executable not found' not in output
    assert 'camera_publisher-' not in output
