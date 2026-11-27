"""Same-stamp TF, one opponent and all legacy avoidance candidates."""
import math

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from pinky_soccer_world_model.world_model_node import WorldModelNode


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=191)
    n = WorldModelNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def scan(t):
    m = LaserScan()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t-int(t))*1e9))
    m.header.frame_id = 'laser'
    m.angle_min, m.angle_increment = -math.pi, 2*math.pi/720
    m.range_min, m.range_max = .05, 10.
    m.ranges = [float('inf')]*720
    for centre in (360, 440):
        for i in range(centre-4, centre+5):
            m.ranges[i] = .5
    return m


def test_one_track_many_obstacles_only_new_scan_updates(node):
    node.scan_transform = lambda *args: (np.eye(3), np.zeros(3))
    node.now = lambda: 1.2
    node.pose = (0., 0., 0.)
    for t in (1., 1.1, 1.2):
        node.on_scan(scan(t))
    assert len(node.opponents) == 2
    corrected = node.opponent_tracker.position.copy()
    outputs, obstacles = [], []
    node.pub.publish = outputs.append
    node.obstacle_pub.publish = obstacles.append
    for _ in range(5):
        node.tick()
    assert len(outputs[-1].opponents) == 1
    assert outputs[-1].opponents[0].id == 'opp_0'
    assert len(obstacles[-1].opponents) == 2
    np.testing.assert_array_equal(corrected, node.opponent_tracker.position)
    node.on_scan(scan(1.1))  # late packet
    np.testing.assert_array_equal(corrected, node.opponent_tracker.position)
    node.now = lambda: 2.3
    node.tick()
    assert not outputs[-1].opponents


def test_missing_stamped_tf_does_not_use_latest(node):
    class Unavailable:
        calls = []
        def lookup_transform(self, target, source, stamp, **kwargs):
            from tf2_ros import LookupException
            self.calls.append(stamp.nanoseconds)
            raise LookupException('no measurement-time TF')
    buffer = Unavailable()
    node.tf = buffer
    node.on_scan(scan(1.))
    assert buffer.calls == [1000000000]
    assert node.opponent_tracker.last_scan is None


def test_map_alignment_jump_discards_track(node):
    node.scan_transform = lambda *args: (np.eye(3), np.zeros(3))
    for t in (1., 1.1, 1.2):
        node.on_scan(scan(t))
    assert node.opponent_tracker.position is not None
    node.scan_transform = lambda *args: (np.eye(3), np.array([.4, 0., 0.]))
    node.on_scan(scan(1.3))
    assert node.opponent_tracker.position is None


def test_delayed_tf_queue_retries_same_stamp_without_latest(node):
    available = [False]
    node.now = lambda: 1.1
    node.scan_transform = lambda *args: ((np.eye(3), np.zeros(3)) if available[0] else None)
    node.on_scan(scan(1.))
    assert len(node.opp_pending) == 1
    assert node.opponent_tracker.last_scan is None
    available[0] = True
    node.process_opponent_scans()
    assert not node.opp_pending
    assert node.opponent_tracker.last_scan == 1.
    available[0] = False
    node.on_scan(scan(1.1))
    node.now = lambda: 1.5
    node.process_opponent_scans()
    assert not node.opp_pending
    assert node.opp_expired == 1
    assert node.opponent_tracker.last_scan == 1.
