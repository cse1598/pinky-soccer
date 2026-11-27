"""Detect the orange golf ball by HLS arcs and reuse the original floor projection.

Input  /camera/image_raw (BEST_EFFORT), /camera/camera_info
Output /ball_detection (geometry_msgs/PointStamped, frame base_link, image stamp)

The fitted outline centre is cast through the calibrated camera model and intersected
with the plane z = ball_radius in base_footprint (floor frame), using the camera TF.
A YOLO detector can replace this node later by publishing the same topic.
"""

from pathlib import Path
import json
from urllib.parse import unquote, urlparse

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from pinky_soccer_world_model.ball_vision import annotate_ball, project_ball
from pinky_soccer_world_model.geometry import (
    BLOB_SHAPE_DEFAULTS, intrinsics_from_hfov, quat_to_matrix)
from pinky_soccer_world_model.robust_ball import BallImageDetector, ROBUST_DEFAULTS
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .ball_offset import apply_ball_offset, load_ball_offset


def observation_status(node, observation, header, point=None):
    """Keep bearing evidence separate from a valid metric observation."""
    publisher = getattr(node, 'status_pub', None)
    if publisher is not None:
        horizontal = None
        if observation.bearing_uv is not None:
            u = observation.bearing_uv[0]
            width = observation.mask.shape[1]
            u = width-1-u if getattr(node, 'rotated', False) else u
            horizontal = u-(width-1)/2
        publisher.publish(String(data=json.dumps(dict(
            stamp=header.stamp.sec+header.stamp.nanosec/1e9,
            image_frame=header.frame_id, bearing_valid=observation.bearing_valid,
            bearing_uv=observation.bearing_uv, circle_valid=observation.distance_valid,
            bearing_horizontal_px=horizontal, bearing_positive='image_right_after_rotation',
            bearing_source='fitted_center' if observation.blob else 'visible_arc_centroid',
            range_valid=point is not None, blob=observation.blob,
            confidence=observation.confidence, arc_fraction=observation.arc_fraction,
            radius_uncertainty_px=observation.radius_uncertainty_px,
            fit_error_px=observation.fit_error_px, roi_top=observation.roi_top,
            offset_xy_m=list(getattr(node, 'offset_xy', (0., 0.))),
            offset_frame='base_link', offset_source=getattr(node, 'offset_source', 'none')))))


def load_camera_yaml(path, _seen=None):
    """Read K and D from a ROS camera_info YAML or a calibration profile.yaml."""
    if str(path).startswith('file:'):
        uri = urlparse(str(path))
        if uri.netloc not in ('', 'localhost'):
            raise ValueError('Camera YAML requires a local file URI')
        path = unquote(uri.path)
    path = Path(path).expanduser().resolve()
    seen = set() if _seen is None else _seen
    if path in seen:
        raise ValueError('Circular camera profile reference')
    seen.add(path)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('Camera YAML must be a mapping')
    if 'launch' in data and 'camera_matrix' not in data:     # calibration profile
        url = (data.get('launch') or {}).get('camera_info_url', '')
        if not url:
            raise ValueError(f'{path} has no launch.camera_info_url')
        target = url if url.startswith('file:') or Path(url).is_absolute() else path.parent / url
        return load_camera_yaml(target, seen)
    K = np.array(data['camera_matrix']['data'], float).reshape(3, 3)
    D = np.array(data.get('distortion_coefficients', {}).get('data', []), float)
    return K, D, (int(data.get('image_width', 0)), int(data.get('image_height', 0)))


def transform_matrix(tf):
    """Return (R, t) of a geometry_msgs/TransformStamped."""
    q, p = tf.transform.rotation, tf.transform.translation
    return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])


class BallDetector(Node):
    """Shared HLS detector with the existing metric PointStamped contract."""

    def __init__(self):
        super().__init__('ball_detector')
        p = self.declare_parameter
        self.radius = p('ball_radius', 0.02135).value     # user-measured diameter 42.7 mm
        flat = p('hsv_ranges', [3, 160, 70, 19, 255, 255]).value
        if len(flat) % 6:
            raise ValueError('hsv_ranges must hold groups of 6 numbers')
        self.ranges = [(tuple(flat[i:i + 3]), tuple(flat[i + 3:i + 6]))
                       for i in range(0, len(flat), 6)]
        hls = list(p('hls_ranges', [3, 15, 100, 19, 220, 255, 3, 65, 60, 19, 160, 99]).value)
        if not hls or len(hls) % 6:
            raise ValueError('hls_ranges must hold groups of six H,L,S bounds')
        self.hls_ranges = [(tuple(hls[i:i+3]), tuple(hls[i+3:i+6])) for i in range(0, len(hls), 6)]
        self.min_area = p('min_area_px', 12.0).value
        self.min_circularity = p('min_circularity', .72).value
        self.shape = {key: float(p(key, value).value)
                      for key, value in BLOB_SHAPE_DEFAULTS.items()}
        if any(not 0 <= value <= 1 for value in self.shape.values()):
            raise ValueError('Ball shape thresholds must be between 0 and 1')
        self.robust_options = {key: float(p(key, value).value)
                               for key, value in ROBUST_DEFAULTS.items()}
        self.visual = BallImageDetector(
            self.ranges, self.min_area, self.min_circularity, self.shape, self.robust_options,
            hls_ranges=self.hls_ranges)
        self.max_range = p('max_range', 2.5).value
        self.rotated = p('image_rotated_180', False).value
        self.floor_frame = p('floor_frame', 'base_footprint').value
        self.output_frame = p('output_frame', 'base_link').value
        camera_yaml = p('camera_yaml', '').value
        self.offset_xy, self.offset_source = load_ball_offset(
            p('ball_offset_source', '').value or camera_yaml,
            simulation=bool(self.get_parameter('use_sim_time').value))
        self.get_logger().info(f'Ball XY correction {self.offset_xy.tolist()} m in base_link; '
                               f'source: {self.offset_source}')
        self.K = self.D = None
        self.allow_uncalibrated = p('allow_uncalibrated', False).value
        self.approximate_hfov = float(p('approximate_hfov_deg', 66.0).value)
        self.approximate = False
        if self.allow_uncalibrated:
            intrinsics_from_hfov(640, 480, self.approximate_hfov)  # validate before startup
            self.get_logger().warn(
                f'TEST MODE: uncalibrated fallback HFOV={self.approximate_hfov} deg, '
                'zero distortion, URDF mounting pose. Metric positions are APPROXIMATE.')
        self.fixed_intrinsics = bool(camera_yaml)
        if camera_yaml:
            self.K, self.D, _ = load_camera_yaml(camera_yaml)
            self.get_logger().info(f'Using intrinsics from {camera_yaml}')
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PointStamped, 'ball_detection', 10)
        self.status_pub = self.create_publisher(String, 'ball_detection/status', qos_profile_sensor_data)
        self.debug_pub = (self.create_publisher(Image, 'ball_detection/debug_image',
                                                qos_profile_sensor_data)
                          if p('publish_debug_image', False).value else None)
        self.mask_pub = None
        self.source_pub = None
        if self.debug_pub is not None:
            self.mask_pub = self.create_publisher(
                Image, 'ball_detection/debug_mask', qos_profile_sensor_data)
            self.source_pub = self.create_publisher(
                Image, 'ball_detection/source_image', qos_profile_sensor_data)
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
            status = ('TEST fallback enabled; metric positions are APPROXIMATE'
                      if self.allow_uncalibrated else
                      'metric detection disabled; pixel preview is available')
            self.get_logger().warn(f'camera_info is not calibrated (K = 0); {status}',
                                   throttle_duration_sec=10.0)
            return
        self.K, self.D = K, np.array(msg.d, float)
        self.approximate = False

    def on_image(self, msg):
        """Detect, cast the ray and publish the ball centre."""
        if self.K is None and self.debug_pub is None and not self.allow_uncalibrated:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.allow_uncalibrated and (self.K is None or self.approximate):
            h, w = frame.shape[:2]
            self.K = intrinsics_from_hfov(w, h, self.approximate_hfov)
            self.D = np.zeros(5)
            self.approximate = True
        visual = getattr(self, 'visual', None)
        if visual is None:
            self.visual = visual = BallImageDetector(
                self.ranges, self.min_area, self.min_circularity,
                getattr(self, 'shape', BLOB_SHAPE_DEFAULTS), getattr(self, 'robust_options', None))
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        observation = visual.detect(frame, stamp, (stamp, msg.header.frame_id))
        blob, mask = observation.blob, observation.mask
        if self.debug_pub is not None:
            # Reuse the detector's existing robot input. The PC monitor must not
            # add another remote raw-image reader and duplicate Wi-Fi traffic.
            source = getattr(self, 'source_pub', None)
            if source is not None and source.get_subscription_count() > 0:
                source.publish(msg)
            preview = annotate_ball(
                frame, blob, 'none' if self.K is None else
                ('approximate' if self.approximate else 'calibrated'), observation)
            debug = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
            debug.header = msg.header
            self.debug_pub.publish(debug)
            if getattr(self, 'mask_pub', None) is not None:
                debug_mask = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                debug_mask.header = msg.header
                self.mask_pub.publish(debug_mask)
        if blob is None or self.K is None:
            observation_status(self, observation, msg.header)
            return
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
                observation_status(self, observation, msg.header)
                return
        point = project_ball(blob, frame.shape, self.K, self.D, transform_matrix(cam),
                             transform_matrix(out), self.radius, self.max_range, self.rotated)
        if point is None:
            observation_status(self, observation, msg.header)
            return
        offset = getattr(self, 'offset_xy', (0., 0.))
        rotation = None
        if np.any(np.asarray(offset) != 0) and self.output_frame != 'base_link':
            try:
                rotation, _ = transform_matrix(self.tf.lookup_transform(self.output_frame, 'base_link', Time()))
            except TransformException:
                observation_status(self, observation, msg.header)
                return
        point = apply_ball_offset(point, offset, rotation)
        observation_status(self, observation, msg.header, point)
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
