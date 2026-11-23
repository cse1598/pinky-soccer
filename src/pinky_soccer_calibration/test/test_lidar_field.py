"""Check map coordinates, unknown-space honesty and lossless raw LiDAR preservation."""

import csv
import math
from types import SimpleNamespace

import cv2
from nav_msgs.msg import Odometry
import numpy as np
from pinky_soccer_calibration.lidar_field import observation, rasterize, save_field
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import LaserScan
import yaml


def scan_box():
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.header.stamp.sec = 123
    scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
    scan.angle_max = scan.angle_min+359*scan.angle_increment
    scan.range_min, scan.range_max = .05, 5.
    angles = scan.angle_min+np.arange(360)*scan.angle_increment
    scan.ranges = np.minimum(1/np.maximum(abs(np.cos(angles)), 1e-12),
                             .6/np.maximum(abs(np.sin(angles)), 1e-12)).astype('float32').tolist()
    scan.intensities = [7.]*360
    return scan


def save(tmp_path, scan, **kwargs):
    return save_field(tmp_path, scan, received_monotonic=10., received_ros_ns=123_000_000_000,
                      now_monotonic=10.1, **kwargs)


def test_map_round_trip_raw_bag_csv_and_repeated_immutable_saves(tmp_path):
    scan = scan_box()
    scan.ranges = (list(scan.ranges[:1])+[float('nan'), float('inf'), float('-inf')]
                   + list(scan.ranges[4:]))
    raw = serialize_message(scan)
    folder, meta = save(tmp_path, scan)
    info = yaml.safe_load((folder/'map.yaml').read_text())
    image = cv2.imread(str(folder/info['image']), cv2.IMREAD_GRAYSCALE)
    assert set(np.unique(image)) == {0, 205, 254}
    assert meta['source_frame'] == 'laser'
    assert meta['capture_base_pose_in_map'] is None
    assert meta['ignored_beams'] == 3
    assert meta['occupied_cells']+meta['free_cells']+meta['unknown_cells'] == image.size
    assert len(list(csv.DictReader((folder/'scan.csv').open()))) == 360
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder/'raw_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic, payload, stamp = reader.read_next()
    assert topic == '/scan' and payload == raw and stamp == 123_000_000_000
    assert serialize_message(scan) == raw
    previous = (folder/'map.pgm').read_bytes()
    second, _ = save(tmp_path, scan)
    assert second != folder and (folder/'map.pgm').read_bytes() == previous
    assert not list(folder.parent.glob('.saving_*'))


def test_sensor_transform_and_pgm_vertical_axis_are_preserved():
    scan = scan_box()
    transform = np.array([[0., -1, 0, .1], [1., 0, 0, .2],
                          [0, 0, 1, .125], [0, 0, 0, 1]])
    geometry = SimpleNamespace(sensor=lambda frame: transform, sha256='model')
    _, _, valid, hits, points, matrix, frame, _ = observation(scan, geometry)
    assert frame == 'base_link' and np.array_equal(matrix, transform)
    # Local +X becomes robot +Y, including the actual sensor translation.
    assert points[180] == pytest.approx([.1, 1.2, .125], abs=1e-6)
    grid, origin = rasterize(points, valid, hits, matrix[:3, 3], .01)
    x, y = np.floor((points[180, :2]-origin)/.01).astype(int)
    assert grid[grid.shape[0]-1-y, x] == 0


def test_only_observed_rays_are_free_not_a_filled_rectangle():
    scan = scan_box()
    scan.ranges = list(scan.ranges[:180])+[float('inf')]*180
    _, _, valid, hits, points, matrix, *_ = observation(scan)
    grid, origin = rasterize(points, valid, hits, matrix[:3, 3], .01)
    x, y = np.floor((np.array([0., .05])-origin)/.01).astype(int)
    assert 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]
    assert grid[grid.shape[0]-1-y, x] == 205


def test_missing_stale_moving_and_unreasonable_input_are_rejected(tmp_path):
    scan = scan_box()
    with pytest.raises(ValueError, match='수신'):
        save_field(tmp_path, scan, received_monotonic=1, received_ros_ns=1, now_monotonic=5)
    odom = Odometry()
    odom.twist.twist.angular.z = .2
    with pytest.raises(ValueError, match='움직'):
        save(tmp_path, scan, odom=odom, odom_age_s=.1)
    scan.ranges = [float('inf')]*360
    with pytest.raises(ValueError, match='12개'):
        observation(scan)
    scan = scan_box()
    scan.angle_increment = 0.
    with pytest.raises(ValueError, match='각도'):
        observation(scan)
    assert not (tmp_path/'lidar_fields').exists()


def test_maximum_range_is_free_space_without_an_invented_wall():
    scan = scan_box()
    scan.ranges[180] = scan.range_max
    _, _, valid, hits, points, matrix, *_ = observation(scan)
    assert valid[180] and not hits[180]
    grid, origin = rasterize(points, valid, hits, matrix[:3, 3], .01)
    x, y = np.floor((points[180, :2]-origin)/.01).astype(int)
    assert grid[grid.shape[0]-1-y, x] == 254
