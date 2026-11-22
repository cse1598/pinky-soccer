"""Camera/compass supervisor for AMCL's symmetric field hypotheses."""

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid

import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from pinky_soccer_field.field_geometry import Field
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, MagneticField
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .ball_detector import load_camera_yaml, transform_matrix
from .geometry import intrinsics_from_hfov, wrap, yaw_from_quat
from .heading_reference import goal_blobs, HeadingReference, magnetic_heading, observed_bearing


def stamp(header):
    """Message time in seconds, in the node's ROS time domain."""
    return header.stamp.sec+header.stamp.nanosec/1e9


class GoalHeading(Node):
    """Publish AMCL reseeds only after qualified, independent confirmations."""

    def __init__(self):
        super().__init__('goal_heading')
        p = self.declare_parameter
        self.field = Field.from_file(p('field_spec', '').value or None)
        self.reference = HeadingReference(self.field)
        self.record_session = uuid.uuid4().hex
        self.ranges = {color: (p(f'{color}_low', lo).value, p(f'{color}_high', hi).value)
                       for color, lo, hi in [('blue', [95, 90, 45], [135, 255, 255]),
                                             ('yellow', [20, 90, 60], [38, 255, 255])]}
        self.allow_approximate = p('allow_uncalibrated', False).value
        self.rotated = p('image_rotated_180', False).value
        self.hfov = float(p('approximate_hfov_deg', 66.).value)
        self.camera = None
        camera_yaml = p('camera_yaml', '').value
        if camera_yaml:
            self.camera = load_camera_yaml(camera_yaml)
        default_record = Path.home().joinpath(
            '.ros', 'pinky_soccer', 'localization', self.record_session, 'heading.yaml')
        record = p('heading_record', str(default_record)).value
        self.record = Path(record).expanduser()
        self.record.parent.mkdir(parents=True, exist_ok=True)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.info = self.imu = self.pose = self.quality = None
        self.magnetic = deque(maxlen=300)
        self.last_magnetic = None
        self.last_correction = -math.inf
        self.last_camera_time = -math.inf
        self.last_visual_time = -math.inf
        self.last_magnetic_vote = -math.inf
        self.pose_received = -math.inf
        self.last_image_wall = -math.inf
        self.last_ros_time = None
        self.last_update_request = -math.inf
        self.nomotion = self.create_client(Empty, '/request_nomotion_update')
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.debug = self.create_publisher(CompressedImage, '/localization/goal_image/compressed',
                                           qos_profile_sensor_data)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/localization/status', state_qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(MagneticField, '/imu/mag', self.on_mag, qos_profile_sensor_data)
        self.create_subscription(String, '/imu/status', self.on_quality, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, state_qos)
        self.create_timer(.1, self.tick)

    def on_info(self, msg):
        self.info = msg

    def on_imu(self, msg):
        self.imu = msg

    def on_pose(self, msg):
        if msg.header.frame_id == 'map':
            self.pose, self.pose_received = msg, time.monotonic()

    def on_quality(self, msg):
        try:
            quality = json.loads(msg.data)
            self.quality = quality if isinstance(quality, dict) else None
        except (ValueError, TypeError):
            self.quality = None

    def transform(self, target, source, seconds):
        when = Time(seconds=seconds, clock_type=self.get_clock().clock_type)
        tf = self.tf.lookup_transform(target, source, when)
        r, t = transform_matrix(tf)
        result = np.eye(4)
        result[:3, :3], result[:3, 3] = r, t
        return result

    def on_mag(self, msg):
        seconds = stamp(msg.header)
        if (self.imu is None or self.quality is None or self.quality.get('mag', 0) < 2
                or self.quality.get('mode') != 'NDOF'
                or abs(seconds-float(self.quality.get('stamp', -math.inf))) > .5
                or abs(seconds-stamp(self.imu.header)) > .1
                or self.imu.header.frame_id != msg.header.frame_id):
            return
        try:
            r = self.transform('base_footprint', msg.header.frame_id, seconds)[:3, :3]
            b = msg.magnetic_field
            a = self.imu.linear_acceleration
            heading, strength = magnetic_heading(r @ [b.x, b.y, b.z], r @ [a.x, a.y, a.z])
            gyro = self.imu.angular_velocity
            speed = float((r @ [gyro.x, gyro.y, gyro.z])[2])
            if self.last_magnetic is not None:
                dt = seconds-self.last_magnetic[0]
                if dt <= 0:
                    return
                if dt >= .5:
                    # A dropout or sustained magnetic jump invalidates the anchor;
                    # a fresh visual consensus must establish it again.
                    self.reference = HeadingReference(self.field)
                    self.magnetic.clear()
                if dt < .5 and abs(wrap(heading-self.last_magnetic[1]-speed*dt)) > .25:
                    return  # compass jump inconsistent with gyro; do not overwrite baseline
            self.last_magnetic = (seconds, heading, strength)
            self.magnetic.append(self.last_magnetic)
        except (ValueError, TransformException):
            return

    def usable_pose(self):
        if self.pose is None or time.monotonic()-self.pose_received > 2.:
            return False
        c = self.pose.pose.covariance
        return (0 <= c[0] <= .04 and 0 <= c[7] <= .04 and 0 <= c[35] <= .12
                and all(math.isfinite(v) for v in c)
                and all(math.isfinite(v) for v in self.pose_tuple()))

    def pose_tuple(self):
        p = self.pose.pose.pose
        q = p.orientation
        return (p.position.x, p.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def record_decision(self, decision, seconds, source):
        """Audit data and latest anchor are session-local; never auto-load old anchors."""
        item = dict(decision, stamp=seconds, source=source,
                    field_spec=self.field.spec, imu_quality=self.quality,
                    record_session=self.record_session,
                    recorded_utc=datetime.now(timezone.utc).isoformat())
        with self.record.with_suffix('.jsonl').open('a') as stream:
            stream.write(json.dumps(item)+'\n')
        temporary = self.record.with_suffix('.yaml.tmp')
        temporary.write_text(yaml.safe_dump(item, allow_unicode=True))
        temporary.replace(self.record)

    def apply(self, decision, seconds, source):
        self.record_decision(decision, seconds, source)
        if not decision['flip'] or time.monotonic()-self.last_correction < 5.:
            return
        msg = deepcopy(self.pose)
        # Use the newest available pose, not the earlier camera exposure pose.
        # A time-zero stamp lets AMCL apply it at its current odometry time.
        msg.header.stamp = Time().to_msg()
        current = self.transform('map', 'base_footprint', 0.)
        x, y = -float(current[0, 3]), -float(current[1, 3])
        theta = wrap(math.atan2(current[1, 0], current[0, 0])+math.pi)
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.x = msg.pose.pose.orientation.y = 0.
        msg.pose.pose.orientation.z = math.sin(theta/2)
        msg.pose.pose.orientation.w = math.cos(theta/2)
        # Under a pi rotation X/Y covariance stays, X/yaw and Y/yaw change sign.
        for i in (5, 11, 30, 31):
            msg.pose.covariance[i] *= -1
        self.initial.publish(msg)
        self.last_correction = time.monotonic()
        self.reference.samples.clear()
        self.get_logger().warn('Reseeded AMCL symmetric pose using '+source)

    def on_image(self, msg):
        seconds = stamp(msg.header)
        if seconds <= self.last_camera_time or time.monotonic()-self.last_image_wall < .1:
            return
        self.last_camera_time, self.last_image_wall = seconds, time.monotonic()
        if abs(self.get_clock().now().nanoseconds/1e9-seconds) > .5:
            return
        frame = None
        try:
            frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return
            blobs = goal_blobs(frame, self.ranges)
            for blob in blobs:
                x, y, bw, bh = blob['box']
                cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                cv2.putText(frame, blob['color'], (x, max(15, y-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
            if not self.usable_pose() or not self.magnetic:
                return
            magnetic = min(self.magnetic, key=lambda item: abs(item[0]-seconds))
            if abs(magnetic[0]-seconds) > .1:
                return
            h, w = frame.shape[:2]
            source = 'calibrated_camera'
            if self.camera:
                k, d, size = self.camera
                if size != (w, h):
                    return
            elif (self.info and self.info.k[0] > 0
                  and (self.info.width, self.info.height) == (w, h)):
                k, d = np.array(self.info.k).reshape(3, 3), np.array(self.info.d)
            elif self.allow_approximate:
                k, d = intrinsics_from_hfov(w, h, self.hfov), np.zeros(5)
                source = 'explicit_approximate_hfov'
            else:
                return
            camera = self.transform('base_footprint', msg.header.frame_id, seconds)
            world = self.transform('map', 'base_footprint', seconds)
            pose = (float(world[0, 3]), float(world[1, 3]),
                    math.atan2(world[1, 0], world[0, 0]))
            # Multiple different visible goals are not used for an automatic anchor.
            if len(blobs) == 1:
                blob = blobs[0]
                uv = blob['uv']
                if self.rotated:
                    uv = (w-1-uv[0], h-1-uv[1])
                bearing = observed_bearing(uv, k, d, camera)
                result = self.reference.visual(pose, blob['color'], bearing,
                                               magnetic[1], magnetic[2], camera[:2, 3])
                x, y, bw, bh = blob['box']
                cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                text = f"{blob['color']} {math.degrees(bearing):.1f} deg"
                cv2.putText(frame, text, (x, max(15, y-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
                if result is not None:
                    self.apply(result, seconds, source+'+goal_color+magnetic')
                    self.last_visual_time = seconds
        except (ValueError, TransformException, cv2.error, OSError) as error:
            if isinstance(error, OSError):
                self.reference = HeadingReference(self.field)
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
        finally:
            # Colour inspection remains usable before AMCL or magnetic calibration is ready.
            if frame is not None and self.debug.get_subscription_count():
                ok, data = cv2.imencode('.jpg', frame)
                if ok:
                    self.debug.publish(CompressedImage(header=deepcopy(msg.header),
                                                       format='jpeg', data=data.tobytes()))

    def tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        if time.monotonic()-self.last_update_request > .5 and self.nomotion.service_is_ready():
            self.nomotion.call_async(Empty.Request())
            self.last_update_request = time.monotonic()
        if self.last_ros_time is not None and now < self.last_ros_time:
            self.reference = HeadingReference(self.field)
            self.magnetic.clear()
            self.last_magnetic = None
            self.last_camera_time = self.last_magnetic_vote = -math.inf
            self.last_visual_time = -math.inf
        self.last_ros_time = now
        if self.last_magnetic is not None and now-self.last_magnetic[0] >= .5:
            self.reference = HeadingReference(self.field)
        ready = bool(self.magnetic and 0 <= now-self.magnetic[-1][0] < .3)
        if ready and self.usable_pose() and now-self.last_visual_time > 1.:
            seconds, heading, strength = self.magnetic[-1]
            if seconds > self.last_magnetic_vote:
                self.last_magnetic_vote = seconds
                result = self.reference.magnetic(self.pose_tuple(), heading, strength)
                if result is not None:
                    try:
                        self.apply(result, now, 'recorded_goal_magnetic_reference')
                    except (OSError, TransformException) as error:
                        self.get_logger().error(str(error), throttle_duration_sec=5.)
        self.status.publish(String(data=json.dumps({
            'stamp': now, 'lidar_pose_ready': bool(self.usable_pose()),
            'magnetic_ready': ready, 'heading_anchor_ready': self.reference.offset is not None,
            'field_minus_magnetic_yaw': self.reference.offset,
            'record': str(self.record), 'method': 'AMCL + confirmed symmetry reseed'})))


def main(args=None):
    rclpy.init(args=args)
    node = GoalHeading()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
