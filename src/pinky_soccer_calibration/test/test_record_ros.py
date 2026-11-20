"""Exercise real DDS image capture and inspect the resulting bag without hardware."""

import csv
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
rosbag2_py = pytest.importorskip('rosbag2_py')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image

from pinky_soccer_calibration.core import load


def test_raw_bag_csv_and_png(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    domain = 180 + os.getpid() % 40
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('synthetic_camera_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    images = node.create_publisher(Image, '/test/image', qos_profile_sensor_data)
    infos = node.create_publisher(CameraInfo, '/test/info', qos_profile_sensor_data)
    root = tmp_path / 'record'
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    source = str(Path(__file__).resolve().parents[1])
    env['PYTHONPATH'] = source + os.pathsep + env.get('PYTHONPATH', '')
    command = [sys.executable, '-m', 'pinky_soccer_calibration.cli', 'record',
               '--session', str(root), '--purpose', 'validation', '--square-mm', '30',
               '--camera-id', 'synthetic', '--camera-settings', 'test-only',
               '--image-topic', '/test/image', '--info-topic', '/test/info',
               '--duration', '3', '--interval', '0.3']
    process = subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True)
    deadline = time.monotonic() + 12
    try:
        while process.poll() is None and time.monotonic() < deadline:
            msg = Image()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'synthetic_optical'
            msg.width, msg.height, msg.encoding, msg.step = 20, 16, 'bgr8', 60
            msg.data = bytes([30, 120, 230] * 320)
            images.publish(msg)
            info = CameraInfo()
            info.header = msg.header
            info.width, info.height = 20, 16
            infos.publish(info)
            executor.spin_once(timeout_sec=.05)
        stdout, stderr = process.communicate(timeout=2)
        assert process.returncode == 0, stdout + stderr
        acquisition = load(root / 'acquisition.yaml')
        assert acquisition['completed']
        assert acquisition['received_images'] > 0
        assert acquisition['received_camera_info'] > 0
        assert acquisition['encoding'] == 'bgr8'
        with (root / 'frames.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == acquisition['received_images']
        assert len(list((root / 'frames').glob('*.png'))) == acquisition['sampled_images']
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(root / 'raw_bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        raw_images = []
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == '/test/image':
                raw_images.append(deserialize_message(raw, Image))
        assert len(raw_images) == len(rows)
        assert bytes(raw_images[0].data) == bytes(msg.data)
        assert raw_images[0].encoding == 'bgr8'
        assert (root / 'raw_bag/metadata.yaml').is_file()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_no_camera_is_not_a_completed_experiment(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    root = tmp_path / 'empty'
    result = subprocess.run([
        sys.executable, '-m', 'pinky_soccer_calibration.cli', 'record',
        '--session', str(root), '--purpose', 'calibration', '--square-mm', '30',
        '--camera-id', 'synthetic', '--camera-settings', 'test-only',
        '--image-topic', '/nonexistent_camera_for_test', '--duration', '0.2'],
        capture_output=True, text=True, timeout=8)
    assert result.returncode == 2, result.stdout + result.stderr
    assert not load(root / 'acquisition.yaml')['completed']
