"""Sensor conversion and launch gates, without accessing robot hardware."""

from copy import deepcopy
import importlib.util
import math
from pathlib import Path

from builtin_interfaces.msg import Time
import cv2
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
import numpy as np
from pinky_soccer_bringup.camera_support import camera_info, image_messages
from pinky_soccer_bringup.imu_units_relay import to_si
from pinky_soccer_bringup.sensor_launch import generate_sensor_description, SENSOR_ARGUMENTS
import pytest
from sensor_msgs.msg import Imu
import yaml


def flatten(entities):
    """Launch entities including those nested in GroupActions (robot_name namespace group)."""
    out = []
    for entity in entities:
        out.append(entity)
        if isinstance(entity, GroupAction):
            out += flatten(entity.get_sub_entities())
    return out


@pytest.fixture(autouse=True)
def writable_logs(tmp_path, monkeypatch):
    """Keep launch logs in the test directory, including restricted environments."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'launch_logs'))


def calibration_yaml(path):
    """Write explicitly synthetic intrinsics used only by this test suite."""
    data = {
        'image_width': 8, 'image_height': 6, 'distortion_model': 'plumb_bob',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [10., 0., 4., 0., 11., 3., 0., 0., 1.]},
        'rectification_matrix': {'rows': 3, 'cols': 3,
                                 'data': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
        'projection_matrix': {'rows': 3, 'cols': 4,
                              'data': [10., 0., 4., 0., 0., 11., 3., 0., 0., 0., 1., 0.]},
        'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': [0.] * 5},
    }
    path.write_text(yaml.safe_dump(data))
    return data


def test_camera_calibration_absent_or_measured(tmp_path):
    """Never invent or silently resize intrinsics."""
    info = camera_info('', 8, 6)
    assert not any(info.k) and not any(info.r) and not any(info.p) and not info.d
    path = tmp_path / 'camera calibration.yaml'
    calibration_yaml(path)
    calibrated = camera_info(path.as_uri(), 8, 6)
    assert calibrated.k[0] == 10.
    with pytest.raises(ValueError, match='resolution'):
        camera_info(path.as_uri(), 16, 12)
    for url in ('http://robot/file.yaml', 'file://otherhost/a.yaml', str(path)):
        with pytest.raises(ValueError):
            camera_info(url, 8, 6)


@pytest.mark.parametrize('key', ['camera_matrix', 'distortion_coefficients'])
def test_camera_calibration_rejects_nonfinite_values(tmp_path, key):
    """Refuse malformed files before opening the camera."""
    path = tmp_path / 'invalid.yaml'
    data = calibration_yaml(path)
    data[key]['data'][0] = float('nan')
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        camera_info(path.as_uri(), 8, 6)


def test_raw_jpeg_and_calibration_share_stamp_and_pixels():
    """Preserve BGR samples and use the same sample header for every output."""
    frame = np.zeros((6, 8, 3), np.uint8)
    frame[:, :] = [5, 25, 200]
    info = camera_info('', 8, 6)
    raw, jpeg, output_info = image_messages(frame, Time(sec=12, nanosec=34), 'optical', info)
    assert raw.header == jpeg.header == output_info.header
    assert raw.header.frame_id == 'optical' and raw.header.stamp.sec == 12
    assert raw.encoding == 'bgr8' and raw.step == 24
    assert bytes(raw.data) == frame.tobytes()
    decoded = cv2.imdecode(np.frombuffer(bytes(jpeg.data), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == frame.shape
    assert np.abs(decoded.astype(float) - frame).max() < 5
    assert info.header.frame_id == ''  # no mutation of the reusable calibration template
    raw, jpeg, _ = image_messages(frame, Time(), 'optical', info,
                                  include_raw=False, include_compressed=False)
    assert raw is None and jpeg is None
    with pytest.raises(ValueError, match='resolution'):
        image_messages(frame[:3], Time(), 'optical', info)
    with pytest.raises(ValueError):
        image_messages(frame.astype(float), Time(), 'optical', info)


def test_imu_conversion_preserves_time_and_scales_covariance():
    """The official driver emits degrees/s despite the ROS message type."""
    native = Imu()
    native.header.stamp.sec = 9
    native.header.frame_id = 'imu_link'
    native.angular_velocity.x, native.angular_velocity.z = 180., -90.
    native.angular_velocity_covariance = [4., 1., 0., 1., 3., 0., 0., 0., 2.]
    native.linear_acceleration.z = 9.81
    native.orientation.w = 1.
    snapshot = deepcopy(native)
    si = to_si(native)
    assert si.angular_velocity.x == pytest.approx(math.pi)
    assert si.angular_velocity.z == pytest.approx(-math.pi / 2)
    assert si.angular_velocity_covariance[1] == pytest.approx((math.pi / 180)**2)
    assert si.header == native.header and si.linear_acceleration == native.linear_acceleration
    assert native == snapshot
    native.angular_velocity_covariance[0] = -1.
    assert to_si(native).angular_velocity_covariance[0] == -1.
    native.angular_velocity.x = float('nan')
    with pytest.raises(ValueError):
        to_si(native)


@pytest.mark.parametrize('hardware,camera,imu,adc,expected', [
    ('false', 'true', 'true', 'true', 0),
    ('true', 'true', 'false', 'false', 2),
    ('true', 'true', 'true', 'true', 5),
    ('true', 'false', 'true', 'true', 3),
])
def test_sensor_launch_hardware_gates(hardware, camera, imu, adc, expected):
    """Description-only mode must never start hardware, even with sensor flags true."""
    context = LaunchContext()
    context.launch_configurations.update(
        hardware=hardware, camera_enabled=camera, imu_enabled=imu, adc_enabled=adc,
        imu_magnetic_enabled='true')
    nodes = [entity for entity in generate_sensor_description().entities
             if isinstance(entity, Node)]
    assert sum(node.condition.evaluate(context) for node in nodes) == expected


def test_robot_launch_forwards_all_sensor_arguments():
    """The main entry point must expose and forward the complete sensor configuration."""
    path = Path(__file__).parents[1] / 'launch/robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    includes = [entity for entity in flatten(module.generate_launch_description().entities)
                if isinstance(entity, IncludeLaunchDescription)]
    sensors = [include for include in includes
               if 'camera_enabled' in dict(include.launch_arguments)]
    assert len(sensors) == 1
    assert set(SENSOR_ARGUMENTS) <= dict(sensors[0].launch_arguments).keys()


@pytest.mark.parametrize('magnetic', ['true', 'false'])
def test_magnetic_and_official_driver_never_run_together(magnetic):
    context = LaunchContext()
    context.launch_configurations.update(hardware='true', imu_enabled='true',
                                         imu_magnetic_enabled=magnetic)
    nodes = {n.node_executable: n for n in generate_sensor_description().entities
             if isinstance(n, Node)}
    assert nodes['bno055_magnetic'].condition.evaluate(context) == (magnetic == 'true')
    official = next(n for n in generate_sensor_description().entities
                    if isinstance(n, Node) and n.node_executable == 'main_node'
                    and n.node_package == 'pinky_imu_bno055')
    assert official.condition.evaluate(context) == (magnetic == 'false')
