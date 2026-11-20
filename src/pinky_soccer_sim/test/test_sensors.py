"""Sensor model and ROS interface regression tests against the physical contract."""

from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from pinky_soccer_sim.sensor_adapter import bgr_frame, SensorAdapter
from pinky_soccer_sim.sensor_model import (
    adc_from_distance, add_mujoco_sensors, configure_gazebo, load_config, ray_ranges)
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, LaserScan, Range
from std_msgs.msg import UInt16MultiArray
import yaml

CONFIG = Path(__file__).parents[1] / 'config/sensors.yaml'


def test_rgb_stride_is_converted_to_bgr():
    msg = Image(height=1, width=1, step=4, encoding='rgb8', data=[255, 0, 2, 99])
    assert bgr_frame(msg).tolist() == [[[2, 0, 255]]]
    msg.encoding = 'mono8'
    with pytest.raises(ValueError):
        bgr_frame(msg)


def test_missing_ir_table_never_invents_adc():
    cfg = load_config(CONFIG)
    with pytest.raises(ValueError, match='missing'):
        adc_from_distance(.2, cfg['ir'])
    cfg['ir'].update(distances=[.1, .3], adc_values=[3000, 1000])
    assert adc_from_distance(.2, cfg['ir']) == 2000
    assert adc_from_distance(float('inf'), cfg['ir']) == 1000


def test_invalid_ir_table_rejected(tmp_path):
    cfg = load_config(CONFIG)
    cfg['ir'].update(distances=[.3, .1], adc_values=[10, 20])
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):
        load_config(path)


def test_gazebo_wrapper_preserves_core_and_adds_sensors():
    from ament_index_python.packages import get_package_share_directory
    import xacro
    path = Path(get_package_share_directory('pinky_soccer_description'))
    urdf = xacro.process_file(str(path / 'urdf/soccer_robot.urdf.xacro'),
                              mappings={'is_sim': 'true'}).toxml()
    root = ET.fromstring(configure_gazebo(urdf, load_config(CONFIG), True, True, True))
    assert root.find("joint[@name='l_wheel_joint']") is not None
    assert root.find(".//sensor[@type='camera']/camera/image/width").text == '640'
    assert root.find(".//sensor[@type='camera']/camera/image/height").text == '480'
    assert root.find(".//sensor[@type='imu']/topic").text == 'sim/imu'
    assert len(root.findall(".//sensor[@type='gpu_lidar']")) == 5  # LiDAR + US + 3 IR
    disabled = ET.fromstring(configure_gazebo(urdf, load_config(CONFIG), False, False, False))
    assert disabled.find(".//sensor[@type='camera']") is None
    assert disabled.find(".//sensor[@type='imu']") is None


def test_mujoco_sensors_measure_gravity_and_obstacle(tmp_path):
    import mujoco
    from ament_index_python.packages import get_package_share_directory
    from pinky_mujoco.description import expand_urdf
    from pinky_mujoco.model import build_model, Physics
    urdf = expand_urdf(package='pinky_soccer_description',
                       filename='urdf/soccer_robot.urdf.xacro')
    scene = build_model(get_package_share_directory('pinky_mujoco'), tmp_path, urdf,
                        world=f"{get_package_share_directory('pinky_gz_sim')}/worlds/empty.world",
                        simple_render=True)
    add_mujoco_sensors(scene)
    tree = ET.parse(scene)
    body = ET.SubElement(tree.getroot().find('worldbody'), 'body',
                         name='test_obstacle', mocap='true', pos='.5 0 .2')
    ET.SubElement(body, 'geom', type='box', size='.01 .5 .2', group='0')
    tree.write(scene)
    physics = Physics(scene)
    physics.step(0, 0, steps=1000)
    mujoco.mj_forward(physics.model, physics.data)
    acc = physics.data.sensor('soccer_acc').data
    gyro = physics.data.sensor('soccer_gyro').data
    assert np.linalg.norm(acc) == pytest.approx(9.81, abs=.3)
    assert np.linalg.norm(gyro) < .1
    measured = ray_ranges(physics, 'ultrasonic_link', load_config(CONFIG))
    assert .35 < min(measured) < .55
    # Moving the obstacle must change the physical ray observation.
    physics.data.mocap_pos[0, 0] += .5
    mujoco.mj_forward(physics.model, physics.data)
    assert min(ray_ranges(physics, 'ultrasonic_link', load_config(CONFIG))) > min(measured) + .4
    for _ in range(150):
        physics.step(0, .6)
    mujoco.mj_forward(physics.model, physics.data)
    assert physics.data.sensor('soccer_gyro').data[2] == pytest.approx(.6, abs=.12)


def test_actual_ros_camera_imu_and_range_contract(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    cfg = load_config(CONFIG)
    # Synthetic table is a test fixture only, never installed as device calibration.
    cfg['ir'].update(distances=[.1, .3], adc_values=[3000, 1000])
    config = tmp_path / 'sensors.yaml'
    config.write_text(yaml.safe_dump(cfg))
    context = Context()
    rclpy.init(args=[], context=context, domain_id=88)
    node = SensorAdapter(context=context, parameter_overrides=[
        Parameter('sensor_config', value=str(config)), Parameter('imu_enabled', value=True),
        Parameter('adc_enabled', value=True)])
    receiver = Node('sensor_contract_receiver', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(receiver)
    got = {}
    subscriptions = []
    for typ, topic, name, qos in [
        (Image, 'camera/image_raw', 'image', qos_profile_sensor_data),
        (CompressedImage, 'camera/image_raw/compressed', 'jpeg', qos_profile_sensor_data),
        (CameraInfo, 'camera/camera_info', 'info', qos_profile_sensor_data),
        (Imu, 'imu_raw', 'imu', qos_profile_sensor_data),
        (Range, 'us_sensor/range', 'us', 10),
        (UInt16MultiArray, 'ir_sensor/range', 'ir', 10),
    ]:
        subscriptions.append(receiver.create_subscription(
            typ, topic, lambda msg, key=name: got.update({key: msg}), qos))
    image_pub = receiver.create_publisher(Image, 'sim/camera/image_raw', 10)
    info_pub = receiver.create_publisher(CameraInfo, 'sim/camera/camera_info', 10)
    imu_pub = receiver.create_publisher(Imu, 'sim/imu', 10)
    range_pubs = {link: receiver.create_publisher(LaserScan, f'sim/{link}/scan', 10)
                  for link in ('ultrasonic_link', 'ir_l_link', 'ir_mid_link', 'ir_r_link')}
    image = Image(height=480, width=640, step=640 * 3, encoding='rgb8',
                  data=np.tile([200, 10, 5], 640 * 480).astype('uint8').tobytes())
    image.header.stamp.sec = 10
    info = CameraInfo(height=480, width=640)
    info.header.stamp.sec = 10
    imu = Imu()
    imu.angular_velocity.z = 1.2
    scan = LaserScan(ranges=[.2])
    scan.header.stamp.sec = 10
    try:
        deadline = time.monotonic() + 8
        while len(got) < 6 and time.monotonic() < deadline:
            image_pub.publish(image)
            info_pub.publish(info)
            imu_pub.publish(imu)
            for pub in range_pubs.values():
                pub.publish(scan)
            executor.spin_once(timeout_sec=.03)
        assert set(got) == {'image', 'jpeg', 'info', 'imu', 'us', 'ir'}
        assert got['image'].encoding == 'bgr8'
        assert list(got['image'].data[:3]) == [5, 10, 200]
        assert got['image'].header == got['jpeg'].header == got['info'].header
        assert got['image'].header.frame_id == 'front_camera_optical_frame'
        assert got['imu'].angular_velocity.z == pytest.approx(1.2)
        assert got['us'].range == pytest.approx(.2)
        assert list(got['ir'].data) == [2000, 2000, 2000]
    finally:
        executor.shutdown()
        node.destroy_node()
        receiver.destroy_node()
        context.shutdown()
