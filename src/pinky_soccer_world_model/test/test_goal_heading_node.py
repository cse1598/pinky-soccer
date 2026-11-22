"""Exercise camera/compass callbacks, auditing, dropout and hidden-goal recovery."""

import json
import math
import time

import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from pinky_soccer_world_model.goal_heading_node import GoalHeading
import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, MagneticField


def test_camera_anchor_then_hidden_goal_magnetic_recovery(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=['--ros-args', '-p', 'heading_record:='+str(tmp_path/'heading.yaml')],
                domain_id=189)
    node = GoalHeading()
    try:
        current = np.eye(4)
        current[:2, :2] = -np.eye(2)  # wrong AMCL hypothesis
        optical = np.eye(4)
        optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        node.transform = lambda target, source, seconds: (
            current if target == 'map' else optical if source == 'optical' else np.eye(4))
        sent = []
        node.initial.publish = sent.append
        node.info = CameraInfo(width=320, height=240,
                               k=[250., 0., 160., 0., 250., 120., 0., 0., 1.])
        node.pose = PoseWithCovarianceStamped()
        node.pose.header.frame_id = 'map'
        node.pose.pose.pose.orientation.z = 1.
        node.pose.pose.pose.orientation.w = 0.
        for i in (0, 7, 35):
            node.pose.pose.covariance[i] = .001
        frame = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(frame, (95, 108), (225, 132), (255, 0, 0), -1)

        def feed(image):
            now = node.get_clock().now()
            seconds = now.nanoseconds/1e9
            node.pose_received = time.monotonic()
            node.quality = dict(mag=3, mode='NDOF', stamp=seconds)
            imu = Imu()
            imu.header.frame_id, imu.header.stamp = 'imu_link', now.to_msg()
            imu.linear_acceleration.z = 9.81
            node.on_imu(imu)
            mag = MagneticField(header=imu.header)
            mag.magnetic_field.x, mag.magnetic_field.z = 22e-6, -42e-6
            node.on_mag(mag)
            _, jpeg = cv2.imencode('.jpg', image)
            photo = CompressedImage(format='jpeg', data=jpeg.tobytes())
            photo.header.frame_id, photo.header.stamp = 'optical', now.to_msg()
            node.on_image(photo)
            node.tick()
            time.sleep(.11)

        for _ in range(6):
            feed(frame)
        assert node.reference.offset is not None
        assert len(sent) == 1
        assert abs(sent[0].pose.pose.orientation.z) < .05
        audit = [json.loads(line) for line in (tmp_path/'heading.jsonl').read_text().splitlines()]
        assert any(row.get('color') == 'blue' and row['flip'] for row in audit)
        assert (tmp_path/'heading.yaml').exists()
        # Camera still streams, but no goal is visible. The learned heading must
        # work independently of camera activity and reseed the aliased AMCL pose.
        node.last_correction -= 6
        for _ in range(20):
            feed(np.zeros_like(frame))
        assert len(sent) >= 2
        assert 'recorded_goal_magnetic_reference' in (tmp_path/'heading.jsonl').read_text()
        # A new compass heading after a dropout may not reuse the previous anchor.
        node.last_magnetic = (node.last_magnetic[0]-1., *node.last_magnetic[1:])
        feed(np.zeros_like(frame))
        assert node.reference.offset is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
