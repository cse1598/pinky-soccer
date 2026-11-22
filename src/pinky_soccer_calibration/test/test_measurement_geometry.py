"""Check front-face measurements, reversed scans, provenance and model failures."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from pinky_soccer_calibration.measurement_geometry import MeasurementGeometry


def robot(visual='<box size="0.02 0.08 0.04"/>'):
    return f'''<robot name="test">
      <link name="base_link"/><link name="bumper_link"><visual>
        <origin xyz="0.005 0 0"/><geometry>{visual}</geometry></visual></link>
      <joint name="bumper" type="fixed"><parent link="base_link"/>
        <child link="bumper_link"/><origin xyz="0.04 0.002 0.01"/></joint>
      <joint name="lidar" type="fixed"><parent link="base_link"/>
        <child link="scan_frame"/><origin xyz="-0.017 0 0.10" rpy="0 0 {math.pi}"/></joint>
      <joint name="us" type="fixed"><parent link="base_link"/>
        <child link="us_frame"/><origin xyz="0.0267 0 0.009"/></joint></robot>'''


def test_front_face_not_link_origin_or_box_centre():
    geometry = MeasurementGeometry(robot())
    assert geometry.front == pytest.approx([.055, .002, .01])
    lidar = geometry.reference(.5, 'scan_frame', 'lidar')
    ultrasound = geometry.reference(.5, 'us_frame', 'ultrasonic')
    assert lidar['sensor_reference_m'] == pytest.approx(.572)
    assert ultrasound['sensor_reference_m'] == pytest.approx(.5283)
    assert geometry.board_registration(.3, -.04, .2, .027) == pytest.approx(
        [.355, -.038, .2, .027])


def test_asymmetric_mesh_bounds_and_scale(tmp_path):
    import struct
    path = tmp_path/'bumper.stl'
    # COM-centred meshes need actual bounds; half-depth would give the wrong nose.
    points = [(0.001, -.02, -.01), (.006, .02, .01), (-.019, 0, 0)]
    path.write_bytes(b'\0'*80 + struct.pack('<I', 1) +
                     struct.pack('<12fH', 0, 0, 1, *np.ravel(points), 0))
    geometry = MeasurementGeometry(robot(f'<mesh filename="{path}" scale="2 1 1"/>'))
    assert geometry.front[0] == pytest.approx(.04+.005+.012)


def test_lidar_uses_robot_front_and_projects_oblique_rays():
    geometry = MeasurementGeometry(robot())
    scan = SimpleNamespace(header=SimpleNamespace(frame_id='scan_frame'),
                           angle_min=-math.pi, angle_increment=math.pi/180,
                           range_min=.05, range_max=10., ranges=[float('inf')]*361)
    scan.ranges[180] = .12  # Rear obstacle must not be compared to the front wall.
    scan.ranges[0], scan.ranges[360] = .572, .572
    scan.ranges[1] = .572/math.cos(math.pi/180)
    assert geometry.scan_distance(scan) == pytest.approx(.572)
    scan.ranges = [float('inf')]*361
    assert math.isnan(geometry.scan_distance(scan))


def test_missing_bumper_frame_invalid_measurement_rejected():
    with pytest.raises(ValueError, match='범퍼'):
        MeasurementGeometry('<robot name="none"><link name="base_link"/></robot>')
    geometry = MeasurementGeometry(robot())
    for distance in (0, -1, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            geometry.reference(distance, 'scan_frame', 'lidar')
    with pytest.raises(ValueError, match='고정 좌표'):
        geometry.reference(.5, 'unknown', 'lidar')
    with pytest.raises(ValueError, match='5°'):
        geometry.reference(.5, 'scan_frame', 'ultrasonic')


def test_ui_range_result_preserves_bumper_input_and_converted_reference(tmp_path):
    from pinky_soccer_calibration.suite_model import Session
    from pinky_soccer_calibration.suite_ui import App
    geometry = MeasurementGeometry(robot())
    import threading
    node = SimpleNamespace(lock=threading.RLock(),
        latest={'/scan': SimpleNamespace(header=SimpleNamespace(frame_id='scan_frame'))},
        fresh=lambda topic: True, measurement_geometry=lambda: geometry)
    app = App.__new__(App)
    app.node, app.inputs = node, {'lidar_reference': '.5'}
    app.session = Session(tmp_path, 'rehearsal', 'bumper')
    sample = {'distance': .574, 'sensor_frame': 'scan_frame',
              'robot_description_sha256': geometry.sha256}
    app.collect = lambda topic: [(0, 0, sample)]*20
    result = app.range_test('lidar')
    assert result['bias'] == pytest.approx(.002)
    assert result['bumper_to_wall_m'] == .5
    assert result['sensor_reference_m'] == pytest.approx(.572)
    assert result['reference'] == result['sensor_reference_m']
    profile = yaml.safe_load(app.session.export(True).read_text())
    assert profile['results']['lidar'][0]['bumper_to_wall_m'] == .5
    assert (app.session.root/result['robot_description_file']).read_text() == robot()
    sample['robot_description_sha256'] = 'changed'
    with pytest.raises(ValueError, match='바뀌었습니다'):
        app.range_test('lidar')


def test_ros_latched_model_scan_selection_and_original_bag(tmp_path, monkeypatch):
    import time
    import rclpy
    import rosbag2_py
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan, Range, BatteryState
    from std_msgs.msg import String, Float32, UInt16MultiArray
    from pinky_soccer_calibration.suite_model import Session
    from pinky_soccer_calibration.suite_ros import Acquisition
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=88)
    publisher = Node('bumper_fixture')
    model = publisher.create_publisher(String, '/robot_description',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    scans = publisher.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    us = publisher.create_publisher(Range, '/us_sensor/range', qos_profile_sensor_data)
    battery = publisher.create_publisher(Float32, '/battery/percent', qos_profile_sensor_data)
    model.publish(String(data=robot()))  # Before subscriber creation, like RSP.
    node = Acquisition()
    try:
        deadline = time.monotonic()+4
        while node.geometry is None and time.monotonic() < deadline:
            time.sleep(.02)
        assert node.measurement_geometry().front[0] == pytest.approx(.055)
        session = Session(tmp_path, 'rehearsal', 'ros-bumper')
        node.begin(session)  # Must retain the pre-existing model in the bag too.
        scan = LaserScan()
        scan.header.frame_id = 'scan_frame'
        scan.angle_min, scan.angle_increment = -math.pi, math.pi/180
        scan.range_min, scan.range_max = .05, 10.
        scan.ranges = [.572]+[float('inf')]*179+[.12]+[float('inf')]*179+[.572]
        ultrasound = Range()
        ultrasound.header.frame_id = 'us_frame'
        ultrasound.min_range, ultrasound.max_range, ultrasound.range = .02, 4., .5283
        deadline = time.monotonic()+4
        while (not all(t in node.latest for t in ('/scan', '/us_sensor/range', '/battery/percent'))
               ) and time.monotonic() < deadline:
            scans.publish(scan)
            us.publish(ultrasound)
            battery.publish(Float32(data=76.))
            time.sleep(.05)
        with node.lock:
            assert node.values['/scan'][-1][2]['distance'] == pytest.approx(.572)
            assert node.values['/us_sensor/range'][-1][2]['distance'] == pytest.approx(.5283)
        node.receive('/batt_state', BatteryState(voltage=7.8, percentage=float('nan')))
        node.receive('/ir_sensor/range', UInt16MultiArray(data=[100, 200, 300]))
        snapshot = node.monitor_snapshot()
        assert snapshot[0]['/battery/percent'].data == 76.
        assert snapshot[0]['/batt_state'].voltage == pytest.approx(7.8)
        assert list(snapshot[0]['/ir_sensor/range'].data) == [100, 200, 300]
        node.end()
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(session.root/'raw_bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        saved_model = saved_scan = saved_battery = False
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == '/robot_description':
                assert deserialize_message(data, String).data == robot()
                saved_model = True
            if topic == '/scan':
                raw = deserialize_message(data, LaserScan)
                assert raw.ranges[180] == pytest.approx(.12)
                assert raw.ranges[0] == pytest.approx(.572)
                saved_scan = True
            if topic == '/battery/percent':
                assert deserialize_message(data, Float32).data == 76.
                saved_battery = True
        assert saved_model and saved_scan and saved_battery
    finally:
        node.close()
        publisher.destroy_node()
        rclpy.shutdown()
