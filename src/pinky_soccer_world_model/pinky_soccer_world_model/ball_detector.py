"""Detect the orange golf ball by HSV colour and publish its centre on the floor.

Input  /camera/image_raw (BEST_EFFORT), /camera/camera_info
Output /ball_detection (geometry_msgs/PointStamped, frame base_link, image stamp)

The blob centroid is cast as a ray through the calibrated camera model and intersected
with the plane z = ball_radius in base_footprint (floor frame), using the camera TF.
A YOLO detector can replace this node later by publishing the same topic.
"""

import math
from pathlib import Path

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from pinky_soccer_world_model.geometry import (detect_color_blob, intersect_plane_z,
                                               pixel_ray, quat_to_matrix)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def load_camera_yaml(path):
    """Read K and D from a ROS camera_info YAML or a calibration profile.yaml."""
    data = yaml.safe_load(Path(path).read_text())
    if 'launch' in data and 'camera_matrix' not in data:     # calibration profile
        url = (data.get('launch') or {}).get('camera_info_url', '')
        if not url:
            raise ValueError(f'{path} has no launch.camera_info_url')
        return load_camera_yaml(url.replace('file://', '', 1))
    K = np.array(data['camera_matrix']['data'], float).reshape(3, 3)
    D = np.array(data.get('distortion_coefficients', {}).get('data', []), float)
    return K, D, (int(data.get('image_width', 0)), int(data.get('image_height', 0)))


def transform_matrix(tf):
    """Return (R, t) of a geometry_msgs/TransformStamped."""
    q, p = tf.transform.rotation, tf.transform.translation
    return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])


class BallDetector(Node):
    """HSV ball detector (YOLO-replaceable by keeping the output topic)."""

    def __init__(self):
        super().__init__('ball_detector')
        p = self.declare_parameter
        self.radius = p('ball_radius', 0.021335).value     # golf ball rule size
        flat = p('hsv_ranges', [5, 120, 80, 25, 255, 255]).value
        if len(flat) % 6:
            raise ValueError('hsv_ranges must hold groups of 6 numbers')
        self.ranges = [(tuple(flat[i:i + 3]), tuple(flat[i + 3:i + 6]))
                       for i in range(0, len(flat), 6)]
        self.min_area = p('min_area_px', 12.0).value
        self.min_circularity = p('min_circularity', 0.4).value
        self.max_range = p('max_range', 2.5).value
        self.rotated = p('image_rotated_180', False).value
        self.floor_frame = p('floor_frame', 'base_footprint').value
        self.output_frame = p('output_frame', 'base_link').value
        camera_yaml = p('camera_yaml', '').value
        self.K = self.D = None
        self.fixed_intrinsics = bool(camera_yaml)
        if camera_yaml:
            self.K, self.D, _ = load_camera_yaml(camera_yaml)
            self.get_logger().info(f'Using intrinsics from {camera_yaml}')
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PointStamped, 'ball_detection', 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)

    def on_info(self, msg):
        """Take intrinsics from camera_info unless a calibration file was given."""
        if self.fixed_intrinsics:
            return
        K = np.array(msg.k, float).reshape(3, 3)
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            self.get_logger().warn('camera_info is not calibrated (K = 0); waiting',
                                   throttle_duration_sec=10.0)
            return
        self.K, self.D = K, np.array(msg.d, float)

    def on_image(self, msg):
        """Detect, cast the ray and publish the ball centre."""
        if self.K is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        blob = detect_color_blob(frame, self.ranges, self.min_area, self.min_circularity)
        if blob is None:
            return
        u, v = blob[0], blob[1]
        if self.rotated:   # image delivered upside down relative to the optical frame
            h, w = frame.shape[:2]
            u, v = w - 1 - u, h - 1 - v
        try:
            cam = self.tf.lookup_transform(self.floor_frame, msg.header.frame_id,
                                           Time.from_msg(msg.header.stamp),
                                           timeout=Duration(seconds=0.05))
            out = self.tf.lookup_transform(self.output_frame, self.floor_frame, Time())
        except TransformException:
            try:
                cam = self.tf.lookup_transform(self.floor_frame, msg.header.frame_id, Time())
                out = self.tf.lookup_transform(self.output_frame, self.floor_frame, Time())
            except TransformException as exc:
                self.get_logger().warn(f'No camera TF: {exc}', throttle_duration_sec=5.0)
                return
        R, t = transform_matrix(cam)
        hit = intersect_plane_z(t, R @ pixel_ray(u, v, self.K, self.D), self.radius)
        if hit is None or math.hypot(hit[0], hit[1]) > self.max_range:
            return
        R2, t2 = transform_matrix(out)
        point = R2 @ hit + t2
        detection = PointStamped()
        detection.header.stamp = msg.header.stamp
        detection.header.frame_id = self.output_frame
        detection.point.x, detection.point.y, detection.point.z = map(float, point)
        self.pub.publish(detection)


def main(args=None):
    """Run the detector."""
    rclpy.init(args=args)
    node = BallDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
