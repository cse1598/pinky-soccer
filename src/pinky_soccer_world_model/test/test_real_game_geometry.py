"""Regression checks for explicit approximation and kickoff odometry alignment."""

import math
from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from pinky_soccer_world_model.ball_detector import BallDetector
from pinky_soccer_world_model.geometry import (compose2d, intrinsics_from_hfov,
                                               quat_from_yaw)
from pinky_soccer_world_model.world_model_node import WorldModelNode
import pytest
from sensor_msgs.msg import CameraInfo


def test_explicit_approximation_publishes_with_tf_and_calibration_takes_over():
    """Use real ray/plane geometry, then replace assumed K by received calibrated K."""
    bridge = CvBridge()
    image = np.zeros((240, 320, 3), np.uint8)
    cv2.circle(image, (160, 180), 15, (0, 140, 255), -1)
    msg = bridge.cv2_to_imgmsg(image, 'bgr8')
    msg.header.frame_id = 'camera_optical'
    camera = TransformStamped()
    camera.transform.translation.z = 0.10
    q = camera.transform.rotation  # Rz(-pi/2) Rx(-pi/2): optical forward = robot +X
    q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
    output = TransformStamped()
    output.transform.rotation.w = 1.0
    points = []
    tf = SimpleNamespace(lookup_transform=lambda target, *a, **k:
                         camera if target == 'floor' else output)
    node = SimpleNamespace(
        K=None, D=None, fixed_intrinsics=False, approximate=False,
        allow_uncalibrated=True, approximate_hfov=66.0,
        debug_pub=None, bridge=bridge, pub=SimpleNamespace(publish=points.append),
        ranges=[((5, 120, 80), (25, 255, 255))], min_area=12, min_circularity=0.4,
        rotated=False, floor_frame='floor', output_frame='base', radius=0.02135,
        max_range=2.5, tf=tf)
    BallDetector.on_image(node, msg)
    assert node.approximate and len(points) == 1
    point = points[0]
    assert point.header.frame_id == 'base'
    assert 0.1 < point.point.x < 1.0
    assert abs(point.point.y) < 0.01
    info = CameraInfo()
    info.k = [500.0, 0.0, 160.0, 0.0, 500.0, 120.0, 0.0, 0.0, 1.0]
    BallDetector.on_info(node, info)
    assert not node.approximate and node.K[0, 0] == 500.0


@pytest.mark.parametrize('hfov', [0, 180, float('nan')])
def test_invalid_approximation_rejected(hfov):
    with pytest.raises(ValueError):
        intrinsics_from_hfov(640, 480, hfov)


def test_first_nonzero_odom_aligns_once_and_transform_agrees():
    """A previously running driver need not restart at zero for real kickoff."""
    kickoff = (0.1, -0.2, 0.3)
    sent = []
    node = SimpleNamespace(
        initial_pose=kickoff, odom_offset=None, localization='odom_only', self_sigma=0.01,
        static=SimpleNamespace(sendTransform=sent.append),
        map_to_odom=lambda: node.odom_offset,
        get_logger=lambda: SimpleNamespace(info=lambda text: None))
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = 3.0, -2.0
    q = msg.pose.pose.orientation
    q.x, q.y, q.z, q.w = quat_from_yaw(math.pi / 2)
    WorldModelNode.on_odom(node, msg)
    assert np.allclose(node.pose, kickoff)
    offset = node.odom_offset
    msg.pose.pose.position.y += 0.2  # forward 20 cm in the original odom frame
    WorldModelNode.on_odom(node, msg)
    assert node.odom_offset == offset and len(sent) == 1
    assert np.allclose(node.pose, compose2d(kickoff, (0.2, 0.0, 0.0)))
    assert np.allclose(node.pose, compose2d(offset, (3.0, -1.8, math.pi / 2)))
