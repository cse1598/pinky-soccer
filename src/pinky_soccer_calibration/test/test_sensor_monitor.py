"""Verify units, unavailable values, slow battery topics and LiDAR coordinates."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan, Range
from std_msgs.msg import Float32, UInt16MultiArray

from pinky_soccer_calibration.sensor_monitor import attitude, readings, scan_points, summary


def test_attitude_and_unknown_orientation():
    msg = Imu()
    msg.orientation.w = 0.
    assert '계산 불가' in summary('/imu_raw', msg)
    msg.orientation.z, msg.orientation.w = math.sin(math.pi/4), math.cos(math.pi/4)
    msg.angular_velocity.z = math.pi
    text = summary('/imu_raw', msg)
    assert '90.0' in text and 'rad/s' in text and '3.142' in text
    assert 'deg/s' in summary('/imu_native', msg)
    msg.orientation_covariance[0] = -1
    assert '자세 미제공' in summary('/imu_raw', msg)
    assert '계산 불가' in attitude(Quaternion(w=float('nan')))


def test_battery_sources_have_different_percentage_units_and_no_fake_charge():
    assert summary('/battery/percent', Float32(data=76.)) == '76.0 %'
    battery = BatteryState(voltage=7.8, percentage=.76, current=float('nan'),
                           temperature=float('nan'))
    text = summary('/batt_state', battery)
    assert '76.0 %' in text and '7.800 V' in text and '미제공' in text
    battery.percentage = float('nan')
    assert '잔량: 미제공' in summary('/batt_state', battery)
    assert '범위 오류' in summary('/battery/percent', Float32(data=120.))


def test_slow_battery_not_marked_stale_and_missing_is_not_zero():
    snapshot = ({'/battery/percent': Float32(data=70.), '/imu_raw': Imu()},
                {'/battery/percent': 100., '/imu_raw': 100.},
                {'/battery/percent': [95., 100.], '/imu_raw': [99.99, 100.]})
    rows = readings(snapshot, 106.)
    assert rows['/battery/percent'][3]
    assert '0.2 Hz' in rows['/battery/percent'][1]
    assert not rows['/imu_raw'][3]
    assert '마지막 값' in rows['/imu_raw'][1]
    assert rows['/us_sensor/range'][2] == '—'
    assert '미수신' in rows['/us_sensor/range'][1]
    assert not readings(snapshot, 116.)['/battery/percent'][3]


def test_range_limits_ir_raw_and_partial_joint_arrays():
    msg = Range(min_range=.02, max_range=3., range=float('inf'))
    assert '미검출' in summary('/us_sensor/range', msg)
    msg.range = float('-inf')
    assert '최소 범위' in summary('/us_sensor/range', msg)
    msg.range = float('nan')
    assert 'NaN' in summary('/us_sensor/range', msg)
    msg.range = .01
    assert '범위 밖' in summary('/us_sensor/range', msg)
    assert '100, 200, 300' in summary('/ir_sensor/range', UInt16MultiArray(data=[100, 200, 300]))
    assert '환산하지 않음' in summary('/ir_sensor/range', UInt16MultiArray())
    assert '미제공' in summary('/joint_states', JointState(name=['left', 'right'], position=[1.]))


def test_lidar_plot_uses_real_model_frame_and_filters_invalid_ranges():
    scan = LaserScan(angle_min=0., angle_increment=math.pi/2, range_min=.1, range_max=5.,
                     ranges=[1., float('nan'), float('inf'), .01, 2.])
    scan.header.frame_id = 'backwards_lidar'
    transform = np.diag([-1., -1., 1., 1.])
    transform[0, 3] = -.017
    geometry = SimpleNamespace(sensor=lambda frame: transform)
    points, frame = scan_points(scan, geometry)
    assert frame == 'base_link'
    assert points[:, 0] == pytest.approx([-1.017, -2.017])
    assert '유효 2/5' in summary('/scan', scan)
    points, frame = scan_points(scan)
    assert frame == 'backwards_lidar'
    assert points[:, 0] == pytest.approx([1., 2.])
    scan.angle_min = float('nan')
    assert not scan_points(scan)[0].size
