"""PC startup localization and command ownership; no robot driver modifications."""
from collections import deque
from copy import deepcopy
import csv
import json
import math
from pathlib import Path
import time
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from pinky_soccer_field.field_geometry import Field
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
import yaml

from .ball_detector import load_camera_yaml, transform_matrix
from .geometry import compose2d, inverse2d, quat_to_matrix, wrap
from .goal_hls import HlsGoalDetector
from .rectangle_localization import fit_rectangle, StartupGate
from .rectangle_tracking import GoalInitializer, RectangleTracker


def seconds(header):
    return header.stamp.sec+header.stamp.nanosec*1e-9


class RectangleLocalization(Node):
    """One map->odom authority and one application-command output owner."""
    def __init__(self):
        super().__init__('rectangle_localization')
        p = self.declare_parameter
        self.field = Field.from_file(p('field_spec', '').value or None)
        wall = self.field.inner_walls
        self.length, self.width = wall.width, wall.height
        self.team = p('our_goal_color', 'yellow').value
        if self.team not in ('yellow', 'blue'):
            raise ValueError('our_goal_color must be yellow or blue')
        self.turn_rate = min(float(p('calibration_turn_rate', .3).value),
                             float(p('calibration_turn_cap', .5).value))
        if not 0 < self.turn_rate <= .5:
            raise ValueError('calibration turn rate must be in (0, .5] rad/s')
        self.timeout = float(p('calibration_timeout', 60.).value)
        self.allow_approximate = p('allow_uncalibrated', False).value
        self.rotated = p('image_rotated_180', False).value
        self.hfov = float(p('approximate_hfov_deg', 66.).value)
        camera_file = p('camera_yaml', '').value
        self.camera = load_camera_yaml(camera_file) if camera_file else None
        self.camera_source = camera_file or 'pixel_counts_no_intrinsics_required'
        self.imu_frame = ''
        self.imu_jump_count = self.imu_outliers = 0
        self.imu_jump_limit = float(p('imu_jump_limit_rad', .25).value)
        self.imu_jump_confirmations = int(p('imu_jump_confirmations', 3).value)
        self.debug = bool(p('debug_enabled', False).value)
        self.fit_interval = 1./float(p('fit_rate_hz', 10.).value)
        self.fit_tolerance = float(p('fit_tolerance_m', .025).value)
        self.fit_minimum = int(p('fit_minimum_per_wall', 12).value)
        self.max_residual = float(p('fit_max_residual_m', .018).value)
        self.coast = float(p('coast_timeout_s', .8).value)
        self.reinitialize_after = float(p('reinitialize_timeout_s', 3.).value)
        self.settle = float(p('goal_settle_s', .5).value)
        self.view_timeout = float(p('goal_view_timeout_s', 2.5).value)
        self.angle_tolerance = float(p('goal_angle_tolerance_rad', .035).value)
        self.tracking_parameters = dict(
            max_position_innovation=float(p('tracking.max_position_innovation_m', .12).value),
            max_yaw_innovation=float(p('tracking.max_yaw_innovation_rad', .25).value),
            imu_limit=float(p('tracking.imu_match_limit_rad', .5).value),
            drift_gain=float(p('tracking.imu_drift_gain', .02).value))
        self.detector = HlsGoalDetector(
            **{key: p('hls.'+key, list(default)).value for key, default in (
                ('yellow_low', (24, 85, 180)), ('yellow_high', (35, 180, 255)),
                ('blue_low', (96, 30, 135)), ('blue_high', (115, 130, 255)))},
            minimum=p('hls.minimum_pixels', 500).value,
            ratio=p('hls.area_ratio', 3.).value,
            confirmations=p('hls.confirmations', 3).value,
            crop_top=p('hls.crop_top', 130).value)
        if (not all(math.isfinite(v) and v > 0 for v in (
                self.fit_interval, self.fit_tolerance, self.max_residual, self.coast,
                self.reinitialize_after, self.settle, self.view_timeout, self.angle_tolerance))
                or self.reinitialize_after <= self.coast or self.fit_minimum < 3):
            raise ValueError('Invalid rectangle localization timing/support settings')
        record = p('heading_record', '').value
        self.root = (Path(record).expanduser().parent if record else
                     Path.home()/'.ros/pinky_soccer/localization'/uuid.uuid4().hex)
        self.root.mkdir(parents=True, exist_ok=True)
        # A caller-supplied parent never causes old records to be overwritten.
        self.root = self.root/('rectangle_'+uuid.uuid4().hex)
        self.root.mkdir()
        self.initializer = self.new_initializer()
        self.tracker = self.new_tracker()
        self.gate = StartupGate()
        self.tf = Buffer()
        self.static_transforms = {}
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.imus = deque(maxlen=300)
        self.odoms = deque(maxlen=120)
        self.fits = deque(maxlen=10)
        self.info = None
        self.imu_received = self.scan_received = self.image_received = -math.inf
        self.odom_received = -math.inf
        self.fit_received = -math.inf
        self.seek_started = None
        self.rotation = 0.
        self.last_imu = None
        self.last_image_stamp = -math.inf
        self.last_fit_attempt = -math.inf
        self.last_fit_stamp = -math.inf
        self.last_goal_sample = -math.inf
        self.reading_started_stamp = -math.inf
        self.processing_ms = deque(maxlen=500)
        self.fit_attempts = self.fit_successes = 0
        self.first_scan_stamp = None
        self.fit_accepted = False
        self.camera_xy, self.camera_yaw, self.angular_speed = (0., 0.), 0., 0.
        self.camera_geometry_ready = False
        self.provisional_offset = None
        self.current_pose = None
        self.confidence = 0.
        self.goal_counts = {'yellow': 0, 'blue': 0}
        self.prior_yaw = None
        self.last_map_publish = -math.inf
        self.last_status = -math.inf
        self.state = 'WAITING_FOR_CALIBRATION'
        self.calibration_requested = False
        self.ready = False
        self.map_odom = None
        self.saved_raw = self.saved_complete = False
        self.led_future = None
        self.led_sent_at = -math.inf
        self.led_key = None
        self.led_state = 'service_unavailable'
        self.last_controller = -math.inf
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, 'localization/status', qos)
        self.ready_pub = self.create_publisher(Bool, 'localization/ready', qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'localization/pose', 10)
        self.filtered = self.create_publisher(LaserScan, 'localization/boundary_scan', qos_profile_sensor_data)
        self.map_pub = self.create_publisher(OccupancyGrid, 'localization/observed_map', qos)
        self.map_image = self.create_publisher(Image, 'localization/map_image', qos_profile_sensor_data)
        self.goal_image = self.create_publisher(Image, 'localization/goal_image', qos_profile_sensor_data)
        self.command = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, 'localization/controller_cmd_vel', self.on_command, 10)
        self.create_service(Trigger, 'localization/abort', self.abort)
        self.create_service(Trigger, 'localization/calibrate', self.recalibrate)
        self.create_service(Trigger, 'localization/recalibrate', self.recalibrate)
        self.led_clients = []
        try:
            from pinky_interfaces.srv import SetLed, SetLamp
            if p('led_enabled', True).value:
                self.led_clients = [(self.create_client(SetLed, 'set_led'), SetLed, 'led'),
                                    (self.create_client(SetLamp, 'set_lamp'), SetLamp, 'lamp')]
        except ImportError:
            self.get_logger().warn('pinky_interfaces unavailable: localization works, LED service types missing')
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.steady)
        self.get_logger().info('Passive sensor reception. Start calibration: localization/calibrate '
                               '(Trigger) or R in the localization window. No motion/LED requests '
                               'before that signal. Records: '+str(self.root))

    def new_initializer(self):
        return GoalInitializer(self.length, self.turn_rate, self.angle_tolerance,
                               self.settle, self.view_timeout)

    def new_tracker(self):
        return RectangleTracker(self.coast, self.reinitialize_after, **self.tracking_parameters)

    def transform(self, source):
        if source in self.static_transforms:
            return self.static_transforms[source]
        tr = self.tf.lookup_transform('base_footprint', source, Time())
        r, t = transform_matrix(tr)
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = r, t
        if seconds(tr.header) == 0.:
            self.static_transforms[source] = matrix
        return matrix

    def source_fresh(self, stamp):
        return stamp > 0 and abs(self.get_clock().now().nanoseconds*1e-9-stamp) < .5

    def on_info(self, msg):
        self.info = msg

    def on_odom(self, msg):
        if not self.source_fresh(seconds(msg.header)):
            return
        q, t = msg.pose.pose.orientation, msg.pose.pose.position
        r = quat_to_matrix(q.x, q.y, q.z, q.w)
        pose = (t.x, t.y, math.atan2(r[1, 0], r[0, 0]))
        if all(math.isfinite(v) for v in pose):
            self.odoms.append((seconds(msg.header), pose))
            self.odom_received = time.monotonic()

    def on_imu(self, msg):
        stamp = seconds(msg.header)
        q = msg.orientation
        if (not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, msg.angular_velocity.x,
                                               msg.angular_velocity.y, msg.angular_velocity.z)) or
                not self.source_fresh(stamp) or msg.orientation_covariance[0] < 0 or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .05):
            return
        try:
            base_sensor = self.transform(msg.header.frame_id)
            rotation = quat_to_matrix(q.x, q.y, q.z, q.w) @ base_sensor[:3, :3].T
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            speed = (base_sensor[:3, :3] @ [msg.angular_velocity.x,
                                          msg.angular_velocity.y, msg.angular_velocity.z])[2]
            if self.last_imu:
                dt = stamp-self.last_imu[0]
                if dt <= 0:
                    return
                innovation = abs(wrap(yaw-self.last_imu[1]-speed*dt))
                if dt <= .5 and innovation > self.imu_jump_limit:
                    self.imu_jump_count += 1
                    self.imu_outliers += 1
                    if self.imu_jump_count < self.imu_jump_confirmations:
                        return  # isolated orientation outlier; do not change the reference
                else:
                    self.imu_jump_count = 0
                discontinuity = dt > .5 or self.imu_jump_count >= self.imu_jump_confirmations
                if discontinuity and (not self.calibration_requested or
                        (dt > .5 and not self.gate.ever_ready and not self.gate.aborted)):
                    # The launch may precede robot bringup, or see only a few
                    # packets during initial discovery. Discard partial anchors
                    # and accept the returning IMU without requiring an R click.
                    # Keep the original motion/time budget; a reconnect cannot
                    # grant unlimited calibration turns.
                    self.initializer = self.new_initializer()
                    self.detector.reset()
                    self.imus.clear()
                    self.fits.clear()
                    self.prior_yaw = self.map_odom = None
                    self.fit_received = self.image_received = -math.inf
                    self.ready = False
                elif discontinuity:
                    self.ready = False
                    self.gate.aborted = True
                    self.state = 'IMU_REFERENCE_INVALID_RECALIBRATE'
                    if self.calibration_requested:
                        self.send()
                    return
                if dt <= .5 and self.seek_started is not None and not self.gate.ever_ready:
                    self.rotation += abs(wrap(yaw-self.last_imu[1]))
            self.last_imu = (stamp, yaw)
            self.angular_speed = float(speed)
            self.imu_frame = msg.header.frame_id
            self.imus.append((stamp, yaw))
            self.imu_received = time.monotonic()
        except (TransformException, ValueError):
            return

    def nearest(self, buffer, stamp, tolerance=.12):
        value = min(buffer, key=lambda v: abs(v[0]-stamp)) if buffer else None
        return value if value and abs(value[0]-stamp) <= tolerance else None

    def on_scan(self, msg):
        wall, stamp = time.monotonic(), seconds(msg.header)
        if not self.source_fresh(stamp) or stamp-self.last_fit_stamp < self.fit_interval-1e-6:
            return
        self.last_fit_attempt = wall
        self.last_fit_stamp = stamp
        try:
            transform = self.transform(msg.header.frame_id)
        except TransformException:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= max(.035, msg.range_min)) & (ranges <= msg.range_max)
        finite_ranges = np.where(valid, ranges, 0.)
        xyz = np.c_[finite_ranges*np.cos(angles), finite_ranges*np.sin(angles), np.zeros(len(ranges))]
        xyz[~valid] = np.nan
        points = (xyz @ transform[:3, :3].T+transform[:3, 3])[:, :2]
        imu = self.nearest(self.imus, stamp)
        if imu is None:
            return
        prior = self.prior_yaw
        if prior is not None and self.fits:
            prior = wrap(prior+wrap(imu[1]-self.fits[-1]['imu_yaw']))
        started = time.perf_counter()
        fit = fit_rectangle(points, self.length, self.width, tolerance=self.fit_tolerance,
                            minimum=self.fit_minimum, prior_yaw=prior)
        self.processing_ms.append(1000*(time.perf_counter()-started))
        self.fit_attempts += 1
        if self.first_scan_stamp is None:
            self.first_scan_stamp = stamp
        self.scan_received = wall
        self.fit_accepted = False
        if fit is None or fit.residual > self.max_residual:
            return
        # Keep a fixed provisional branch while aiming, independent of an
        # optimizer's equivalent pi branch ordering.
        if self.provisional_offset is None:
            self.provisional_offset = wrap(imu[1]-fit.pose[2])
        expected = wrap(imu[1]-self.provisional_offset)
        fit.pose = min(fit.hypotheses, key=lambda p: abs(wrap(p[2]-expected)))
        self.prior_yaw = fit.pose[2]
        odom = self.nearest(self.odoms, stamp)
        if self.tracker.pose is not None:
            if odom is None or self.tracker.predict(odom[1], imu[1], stamp) is None:
                return
            if not self.tracker.correct(fit.hypotheses, imu[1], stamp):
                return
        self.fit_accepted = True
        self.fit_successes += 1
        self.fit_received = time.monotonic()
        self.fits.append(dict(stamp=stamp, fit=fit, imu_yaw=imu[1], points=points,
                              angles=angles, ranges=ranges))
        if self.debug:
            filtered = deepcopy(msg)
            filtered.ranges = [float(r) if keep else float('nan') for r, keep in zip(ranges, fit.mask)]
            self.filtered.publish(filtered)
        if not self.saved_raw:
            with (self.root/'initial_scan.csv').open('w') as f:
                writer = csv.writer(f)
                writer.writerow(['angle_rad', 'range_m', 'base_x_m', 'base_y_m', 'outer_wall'])
                writer.writerows((a, r, p[0], p[1], int(k)) for a, r, p, k in
                                  zip(angles, ranges, points, fit.mask))
            (self.root/'initial_scan.yaml').write_text(yaml.safe_dump(dict(
                stamp=stamp, frame=msg.header.frame_id, base_from_scan=transform.tolist(),
                field_spec=self.field.spec, hypotheses=[list(p) for p in fit.hypotheses],
                measured_size=list(fit.measured_size), residual=fit.residual)))
            self.saved_raw = True
        if self.debug:
            self.draw_map(self.fits[-1], None)

    def on_image(self, msg):
        # No decode, conversion, masks or debug image generation during play.
        if self.gate.ever_ready or self.initializer.offset is not None:
            return
        stamp = seconds(msg.header)
        if not self.source_fresh(stamp) or stamp <= self.last_image_stamp:
            return
        self.last_image_stamp = stamp
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        try:
            # Geometry aims the optical axis; counts need no pixel-to-ray
            # projection. Ball perception keeps its existing K/D checks.
            camera = self.transform(msg.header.frame_id)
            forward = camera[:3, 2]
            self.camera_yaw = math.atan2(forward[1], forward[0])
            self.camera_xy = tuple(camera[:2, 3])
            self.camera_geometry_ready = True
            imu = self.nearest(self.imus, stamp)
            item = min(self.fits, key=lambda f: abs(f['stamp']-stamp)) if self.fits else None
            if not imu or item is None or abs(item['stamp']-stamp) > .15:
                return
            self.image_received = time.monotonic()
            if (not self.calibration_requested or self.initializer.state != 'READING_GOAL'
                    or stamp < self.reading_started_stamp or abs(self.angular_speed) > .05):
                self.detector.reset()
                return
            if time.monotonic()-self.last_goal_sample > .5:
                self.detector.reset()
            self.last_goal_sample = time.monotonic()
            if self.rotated:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            result = self.detector.detect(frame, self.debug)
            self.goal_counts = result['counts']
            if self.debug and 'masks' in result:
                debug = np.zeros_like(frame)
                roi = debug[self.detector.crop_top:]
                roi[result['masks']['yellow'] > 0] = (0, 255, 255)
                roi[result['masks']['blue'] > 0] = (255, 100, 0)
                annotated = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
                annotated.header = msg.header
                self.goal_image.publish(annotated)
            if result['color']:
                pose = (*item['fit'].pose[:2], wrap(item['fit'].pose[2]+imu[1]-item['imu_yaw']))
                chosen = self.initializer.identify(pose, result['color'], imu[1], stamp)
                odom = self.nearest(self.odoms, stamp)
                if chosen is not None and odom is not None:
                    self.tracker.seed(chosen, odom[1], imu[1], stamp)
                elif chosen is not None:
                    self.initializer.offset = None
                    self.initializer.state = 'READING_GOAL'
        except (TransformException, ValueError, cv2.error):
            return

    def send(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.command.publish(msg)

    def on_command(self, msg):
        self.last_controller = time.monotonic()
        if (self.ready and time.monotonic()-self.fit_received < self.coast and
                time.monotonic()-self.odom_received < .5 and
                time.monotonic()-self.imu_received < .5 and not self.gate.aborted):
            if all(math.isfinite(v) for vector in (msg.linear, msg.angular)
                   for v in (vector.x, vector.y, vector.z)):
                self.command.publish(msg)  # preserve each 50 Hz controller sample; no resampling

    def abort(self, request, response):
        self.gate.aborted = True
        self.ready = False
        if self.calibration_requested:
            self.send()
        response.success, response.message = True, 'Stopped. Use localization/recalibrate to restart.'
        return response

    def recalibrate(self, request, response):
        self.calibration_requested = True
        self.send()
        self.initializer = self.new_initializer()
        self.tracker = self.new_tracker()
        self.detector.reset()
        self.provisional_offset = None
        self.current_pose = None
        self.last_goal_sample = -math.inf
        self.reading_started_stamp = -math.inf
        self.gate = StartupGate()
        self.ready = self.saved_complete = False
        self.seek_started = None
        self.rotation = 0.
        self.last_imu = None
        self.imu_jump_count = 0
        self.imus.clear()
        self.fits.clear()
        self.prior_yaw = None
        self.last_fit_stamp = -math.inf
        self.map_odom = None
        self.first_scan_stamp = None
        self.fit_attempts = self.fit_successes = 0
        self.processing_ms.clear()
        self.confidence = 0.
        self.fit_received = -math.inf
        self.root = self.root.parent/('rectangle_'+uuid.uuid4().hex)
        self.root.mkdir()
        self.saved_raw = False
        response.success, response.message = True, 'Started a new calibration session.'
        return response

    def tick(self):
        # Display-only output can be toggled live without restarting localization.
        self.debug = bool(self.get_parameter('debug_enabled').value)
        now = time.monotonic()
        links = dict(imu=now-self.imu_received < .5,
                     lidar_outer_walls=now-self.fit_received < .5,
                     odom=now-self.odom_received < .5,
                     camera_intrinsics_tf=now-self.image_received < .5)
        sensors = links['imu'] and links['lidar_outer_walls'] and links['odom']
        connected = sensors and links['camera_intrinsics_tf']
        if not self.calibration_requested:
            self.state = 'WAITING_FOR_CALIBRATION'
            self.ready = False
            self.publish_status(now, links, connected)
            return  # receive/render only: no cmd_vel, TF or LED hardware requests
        if connected and self.seek_started is None and not self.gate.aborted:
            self.seek_started = now
        budget = ((self.seek_started is None or now-self.seek_started < self.timeout)
                  and abs(self.rotation) < 2.5*math.pi)
        self.ready = False
        w = 0.
        if self.gate.aborted:
            self.state = 'ABORTED'
        elif self.tracker.pose is not None:
            odom = self.odoms[-1] if self.odoms else None
            imu = self.nearest(self.imus, odom[0]) if odom else None
            stamp = self.get_clock().now().nanoseconds*1e-9
            valid, state, confidence = self.tracker.validity(stamp)
            self.state, self.confidence = state, confidence
            if valid and links['imu'] and links['odom'] and odom and imu:
                delta = compose2d(inverse2d(self.tracker.odom), odom[1])
                pose = compose2d(self.tracker.pose, delta)
                pose = (*pose[:2], wrap(self.tracker.pose[2]+wrap(imu[1]-self.tracker.imu)))
                self.current_pose = pose
                self.map_odom = compose2d(pose, inverse2d(odom[1]))
                self.ready = self.gate.ever_ready = True
                if now-self.last_map_publish >= .05:
                    item = dict(self.fits[-1], stamp=odom[0])
                    self.publish_pose(pose, item)
                    if self.debug:
                        self.draw_map(self.fits[-1], pose)
                    self.last_map_publish = now
                if not self.saved_complete:
                    self.save_reference(pose, odom[0])
            elif valid:
                self.state, self.confidence = 'WAITING_FOR_ROBOT', 0.
        elif not budget:
            self.state = 'CALIBRATION_TIMEOUT'
        elif not connected or not self.camera_geometry_ready:
            self.state = 'WAITING_FOR_ROBOT'
            self.detector.reset()
        elif self.fits:
            item = self.fits[-1]
            pose = (*item['fit'].pose[:2], wrap(item['fit'].pose[2]+
                                              self.last_imu[1]-item['imu_yaw']))
            previous = self.initializer.state
            w = self.initializer.step(pose, now, self.camera_xy,
                                      self.camera_yaw, self.angular_speed)
            if previous != self.initializer.state or self.initializer.state != 'READING_GOAL':
                self.detector.reset()
            if previous != 'READING_GOAL' and self.initializer.state == 'READING_GOAL':
                self.reading_started_stamp = self.get_clock().now().nanoseconds*1e-9
            self.state = self.initializer.state
        if self.ready:
            tr = TransformStamped()
            tr.header.stamp = self.get_clock().now().to_msg()
            tr.header.frame_id, tr.child_frame_id = 'map', 'odom'
            tr.transform.translation.x, tr.transform.translation.y = self.map_odom[:2]
            tr.transform.rotation.z, tr.transform.rotation.w = math.sin(self.map_odom[2]/2), math.cos(self.map_odom[2]/2)
            self.broadcaster.sendTransform(tr)
            if now-self.last_controller > .5:
                self.send()
        else:
            self.send(0., w)
        self.update_led(now)
        self.publish_status(now, links, connected)

    def save_reference(self, pose, stamp):
        (self.root/'heading_reference.yaml').write_text(yaml.safe_dump(dict(
            field_spec=self.field.spec, team=self.team, references=self.initializer.anchors,
            imu_map_offset_rad=self.tracker.offset, imu_session_only=True,
            angle_unit='rad', imu_frame=self.imu_frame,
            camera_intrinsics_source=self.camera_source, camera_yaw_rad=self.camera_yaw,
            image_rotated_180=self.rotated, namespace=self.get_namespace(),
            initial_pose=list(self.initializer.initial_pose), final_pose=list(pose),
            localization_valid=True, scan_stamp=stamp), allow_unicode=True))
        self.saved_complete = True

    def publish_status(self, now, links, connected):
        if now-self.last_status > .2:
            if getattr(self, '_reported_state', None) != self.state:
                self.get_logger().info('Localization state: '+self.state)
                self._reported_state = self.state
            self.ready_pub.publish(Bool(data=self.ready))
            self.status.publish(String(data=json.dumps(dict(
                source='rectangle_lidar_goal_imu', state=self.state, ready=self.ready,
                stamp=self.get_clock().now().nanoseconds*1e-9,
                calibration_requested=self.calibration_requested,
                team=self.team, learned_colors=[c for c, ref in self.initializer.anchors.items()
                                                if ref['observed']],
                initialization_complete=self.gate.ever_ready,
                valid=self.ready, confidence=self.confidence,
                hypotheses=[list(p) for p in self.fits[-1]['fit'].hypotheses] if self.fits else [],
                pose=list(self.current_pose) if self.current_pose else None,
                selected_goal_sign=(next((1 if c == 'blue' else -1 for c, ref in
                    self.initializer.anchors.items() if ref['observed']), self.initializer.target)),
                provisional_goal_sign=self.initializer.target,
                selected_goal_color=next((c for c, ref in self.initializer.anchors.items()
                                         if ref['observed']), None),
                debug_enabled=self.debug,
                goal_centers=[[-self.length/2, 0.], [self.length/2, 0.]],
                target_error_rad=getattr(self.initializer, 'error', None),
                goal_pixel_counts=self.goal_counts, goal_streak=self.detector.streak,
                imu_map_offset_rad=self.tracker.offset,
                fit_accepted=self.fit_accepted, rejected_fits=self.tracker.rejected,
                fit_attempts=self.fit_attempts, fit_successes=self.fit_successes,
                fit_source_hz=((self.fit_attempts-1)/(self.last_fit_stamp-self.first_scan_stamp)
                    if self.first_scan_stamp is not None and self.last_fit_stamp > self.first_scan_stamp
                    else None),
                imu_outliers=self.imu_outliers,
                processing_ms=dict(mean=float(np.mean(self.processing_ms)),
                    p95=float(np.percentile(self.processing_ms, 95)),
                    maximum=float(max(self.processing_ms))) if self.processing_ms else {},
                startup_inputs_ready=connected,
                missing_inputs=[k for k, ok in links.items() if not ok
                                and (k != 'camera_intrinsics_tf' or not self.gate.ever_ready)],
                led=self.led_state, records=str(self.root), rotation_rad=self.rotation,
                imu_yaw_deg=math.degrees(self.last_imu[1]) if self.last_imu else None,
                goal_axis_imu_deg={color: math.degrees(ref['goal_axis_imu_yaw'])
                                   for color, ref in self.initializer.anchors.items()},
                sensors=links,
                fit_residual_m=self.fits[-1]['fit'].residual if self.fits else None))))
            self.last_status = now

    def publish_pose(self, pose, item):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = Time(seconds=item['stamp']).to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, pose[:2])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        # Starting uncertainty assumptions, not a claim of measured global accuracy.
        msg.pose.covariance[0] = msg.pose.covariance[7] = max(.02, item['fit'].residual)**2
        msg.pose.covariance[35] = math.radians(5)**2
        self.pose_pub.publish(msg)

    def draw_map(self, item, pose):
        fit, points = item['fit'], item['points']
        oriented = pose is not None
        pose = fit.pose if pose is None else pose
        c, s = math.cos(pose[2]), math.sin(pose[2])
        mapped = points @ np.array([[c, s], [-s, c]])+pose[:2]
        image = np.full((440, 640, 3), 28, np.uint8)
        scale = min(580/self.length, 360/self.width)
        def pixel(point):
            return (round(320+scale*point[0]), round(220-scale*point[1]))
        cv2.rectangle(image, pixel((-self.length/2, self.width/2)),
                      pixel((self.length/2, -self.width/2)), (180, 180, 180), 1)
        for point, keep in zip(mapped, fit.mask):
            if np.isfinite(point).all():
                cv2.circle(image, pixel(point), 2, (80, 240, 80) if keep else (80, 80, 220), -1)
        for sign in (-1, 1):
            color = (0, 255, 255) if oriented and sign < 0 else (
                     (255, 100, 0) if oriented else (170, 170, 170))
            cv2.circle(image, pixel((sign*self.length/2, 0.)), 7, color, 2)
        if not oriented:
            alternate = fit.hypotheses[1]
            center2 = pixel(alternate[:2])
            end2 = (round(center2[0]+25*math.cos(alternate[2])),
                    round(center2[1]-25*math.sin(alternate[2])))
            cv2.arrowedLine(image, center2, end2, (180, 100, 180), 2)
        center = (round(320+scale*pose[0]), round(220-scale*pose[1]))
        forward = (round(center[0]+30*math.cos(pose[2])), round(center[1]-30*math.sin(pose[2])))
        cv2.arrowedLine(image, center, forward, (230, 230, 230), 2)
        cv2.putText(image, 'green: walls | red: excluded | '+self.state,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .43, (240, 240, 240), 1)
        self.map_image.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        if not self.gate.ever_ready or not oriented:
            return
        res = .01
        nx, ny = math.ceil((self.length+.1)/res), math.ceil((self.width+.1)/res)
        origin = (-self.length/2-.05, -self.width/2-.05)
        cells = np.full((ny, nx), -1, np.int8)
        for x, y in mapped[fit.mask]:
            ix, iy = int((x-origin[0])/res), int((y-origin[1])/res)
            if 0 <= ix < nx and 0 <= iy < ny:
                cells[iy, ix] = 100
        msg = OccupancyGrid()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        msg.info.resolution, msg.info.width, msg.info.height = res, nx, ny
        msg.info.origin.position.x, msg.info.origin.position.y = origin
        msg.info.origin.orientation.w = 1.
        msg.data = cells.ravel().tolist()
        self.map_pub.publish(msg)
        if not self.saved_complete:
            with (self.root/'final_scan.csv').open('w') as f:
                writer = csv.writer(f)
                writer.writerow(['angle_rad', 'range_m', 'map_x_m', 'map_y_m', 'outer_wall'])
                writer.writerows((a, r, point[0], point[1], int(keep)) for a, r, point, keep in
                    zip(item['angles'], item['ranges'], mapped, fit.mask))
            pgm = np.where(cells < 0, 205, 0).astype(np.uint8)
            cv2.imwrite(str(self.root/'outer_walls.pgm'), np.flipud(pgm))
            (self.root/'outer_walls.yaml').write_text(yaml.safe_dump(dict(
                image='outer_walls.pgm', resolution=res, origin=[*origin, 0.],
                negate=0, occupied_thresh=.65, free_thresh=.196)))

    def update_led(self, now):
        if not self.calibration_requested:
            return
        if self.led_future and not self.led_future.done():
            if now-self.led_sent_at < 2:
                return
            self.led_future.cancel()
        if self.led_future and self.led_future.done():
            try:
                response = self.led_future.result()
                self.led_state = 'ok' if response and getattr(response, 'success', getattr(response, 'result', False)) else 'service_failed'
            except Exception:
                self.led_state = 'service_failed'
            self.led_future = None
        color = ((96, 96, 0) if self.team == 'yellow' else (0, 0, 96)) if self.ready else (
                 (0, 96, 0) if int(now*2) % 2 else (0, 0, 0))
        if color == self.led_key and now-self.led_sent_at < 2:
            return
        for client, srv, kind in self.led_clients:
            if not client.service_is_ready():
                continue
            request = srv.Request()
            if kind == 'led':
                request.command = 'fill'
                request.r, request.g, request.b = color
            else:
                request.mode, request.time = 1, 500
                request.color.r, request.color.g, request.color.b = [v/255 for v in color]
            self.led_future = client.call_async(request)
            self.led_key, self.led_sent_at = color, now
            self.led_state = 'request_pending'
            return
        self.led_state = 'service_unavailable'


def main(args=None):
    rclpy.init(args=args)
    node = RectangleLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and node.calibration_requested:
            node.send()
        node.destroy_node()
        rclpy.try_shutdown()
