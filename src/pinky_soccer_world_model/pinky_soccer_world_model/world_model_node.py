"""Fuse self pose, ball detections and LiDAR into /world_model (map frame, ~30 Hz).

Self pose
  localization=tf        : TF map -> base_footprint (AMCL provides map -> odom).
  localization=odom_only : /odom composed with the kickoff pose `initial_pose`; this node
                           then also broadcasts map -> odom (static) so TF users agree.
Ball      : /ball_detection (PointStamped) -> map -> BallFilter (Kalman + friction).
Search    : SearchGrid lowered inside the camera field of view when nothing is seen.
Opponents : /scan points inside the walls, clustered, confirmed and alpha-beta tracked (one opponent).
Game      : /game/playing (std_msgs/Bool).
"""

from collections import deque
import json
import math
import time

from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import TrackedObject, WorldModel
from pinky_soccer_world_model.ball_detector import transform_matrix
from pinky_soccer_world_model.ball_filter import BallFilter, BallFilterParams, LOST, OBSERVED
from pinky_soccer_world_model.geometry import compose2d, inverse2d, quat_from_yaw, yaw_from_quat
from pinky_soccer_world_model.geometry import wrap
from pinky_soccer_world_model.msgs import tracked, world_model
from pinky_soccer_world_model.opponents import (
    SingleOpponentTracker, cluster_opponents, indexed_candidates, scan_points)
from pinky_soccer_world_model.search_grid import SearchGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


class WorldModelNode(Node):
    """World model; all tunables are parameters (see config/*.yaml)."""

    def __init__(self):
        super().__init__('world_model_node')
        p = self.declare_parameter
        self.field = Field.from_file(p('field_spec', '').value or None,
                                     p('our_goal_color', 'yellow').value)
        self.localization = p('localization', 'odom_only').value
        if self.localization not in ('odom_only', 'tf'):
            raise ValueError('localization must be odom_only or tf')
        self.initial_pose = tuple(p('initial_pose', [0.0, 0.0, 0.0]).value)
        self.align_first_odom = p('align_first_odom', False).value
        self.odom_offset = None if self.align_first_odom else self.initial_pose
        self.odom_timeout = float(p('odom_timeout', 0.0).value)
        self.last_odom_received = None
        self.localization_timeout = float(p('localization_pose_timeout', 0.0).value)
        self.last_localization_received = None
        self.tf_pose_ready = False
        self.map_frame = p('map_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.self_sigma = p('self_sigma_floor', 0.01).value
        self.playing = p('playing_default', False).value
        rate = p('rate_hz', 30.0).value

        names = BallFilterParams.__dataclass_fields__
        bp = BallFilterParams(**{k: p(f'ball.{k}', float(getattr(BallFilterParams(), k))).value
                                 for k in names})
        self.ball_radius = p('ball_radius', 0.02135).value
        self.ball = BallFilter(bp, self.field.inner_walls, self.ball_radius)

        self.grid = SearchGrid(self.field.inner_walls, p('grid.resolution', 0.05).value,
                               relax_tau=p('grid.relax_tau', 8.0).value,
                               miss_factor=p('grid.miss_factor', 0.3).value)
        self.hfov_fallback = math.radians(p('grid.camera_hfov_deg', 60.0).value)
        self.fov_min = p('grid.min_range', 0.15).value
        self.fov_max = p('grid.max_range', 1.5).value
        self.hfov = None
        self.camera_seen = None

        self.opp_enabled = p('opponents.enabled', True).value
        self.opp_cfg = {'wall_margin': p('opponents.wall_margin', 0.06).value,
                        'gap': p('opponents.cluster_gap', 0.05).value,
                        'min_points': p('opponents.min_points', 3).value,
                        'max_size': p('opponents.max_size', 0.30).value}
        self.opp_hold = p('opponents.hold_sec', 1.0).value
        self.opponents, self.opp_time = [], None
        self.opponent_tracker = SingleOpponentTracker(
            hold_sec=self.opp_hold, **{k: p('opponents.'+k, v).value for k, v in dict(
                alpha=.6, beta=.06, observed_sec=.3, confirm_hits=3, confirm_window=5,
                max_speed=.6, association_slack=.08, velocity_deadband=.015).items()})
        self.opp_extra = {k: p('opponents.'+k, v).value for k, v in dict(
            min_size=.015, self_radius=.08, max_beam_gap=3, max_adaptive_gap=.09).items()}
        self.opp_map_alignment = None
        self.opp_pending = deque(maxlen=5)
        self.opp_tf_wait = p('opponents.tf_wait_sec', .3).value
        self.opp_processed = self.opp_expired = 0

        self.pose = None            # (x, y, theta) map
        self.pose_source_stamp = None
        self.last_lookup_stamp = None
        self.velocity = (0.0, 0.0)
        self.pose_cov = [0.0] * 9
        self.last_tick = None

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        if self.localization == 'odom_only' and p('publish_map_to_odom', True).value:
            self.static = StaticTransformBroadcaster(self)
            if self.odom_offset is not None:
                self.static.sendTransform(self.map_to_odom())
        self.pub = self.create_publisher(WorldModel, 'world_model', 10)
        # Existing unconfirmed clusters remain available for collision avoidance.
        self.obstacle_pub = self.create_publisher(WorldModel, 'world_model/obstacles', 10)
        self.opponent_status_pub = self.create_publisher(String, 'opponents/status', 5)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(String, 'preparation/team', self.on_team,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointStamped, 'ball_detection', self.on_ball, 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, 'game/playing', self.on_playing, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self.on_localization_pose, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'World model: {self.localization}, our goal {self.field.our_goal_color}, '
            f'attack {"+X" if self.field.attack_sign > 0 else "-X"}')

    # ------------------------------------------------------------------ inputs
    def now(self):
        """Return node clock seconds (sim time when use_sim_time)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def map_to_odom(self):
        """Return the static kickoff transform map -> odom."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = self.map_frame, self.odom_frame
        t.transform.translation.x, t.transform.translation.y = self.odom_offset[:2]
        q = quat_from_yaw(self.odom_offset[2])
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
         t.transform.rotation.w) = q
        return t

    def on_team(self, msg):
        if not self.playing and msg.data in ('yellow', 'blue'):
            self.field = Field(self.field.spec, msg.data)

    def on_playing(self, msg):
        """Latest game start/stop."""
        self.playing = msg.data

    def on_info(self, msg):
        """Camera alive marker and horizontal FOV from the intrinsics."""
        self.camera_seen = self.now()
        if msg.k[0] > 0 and msg.width > 0:
            self.hfov = 2 * math.atan(msg.width / (2 * msg.k[0]))

    def on_odom(self, msg):
        """odom_only: compose with the kickoff pose; both modes: velocity and covariance."""
        q = msg.pose.pose.orientation
        odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quat(q.x, q.y, q.z, q.w))
        self.last_odom_received = time.monotonic()
        c = msg.pose.covariance
        if self.localization == 'odom_only':
            self.pose_cov = [max(c[0], self.self_sigma ** 2), c[1], c[5], c[6],
                             max(c[7], self.self_sigma ** 2), c[11], c[30], c[31],
                             max(c[35], 1e-4)]
        if self.localization == 'odom_only':
            if self.odom_offset is None:
                self.odom_offset = compose2d(self.initial_pose, inverse2d(odom_pose))
                if hasattr(self, 'static'):
                    self.static.sendTransform(self.map_to_odom())
                self.get_logger().info('Aligned first odom sample to kickoff initial_pose')
            self.pose = compose2d(self.odom_offset, odom_pose)
            self.pose_source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.pose is not None:
            v = msg.twist.twist.linear.x
            self.velocity = (v * math.cos(self.pose[2]), v * math.sin(self.pose[2]))

    def lookup(self, target, source, stamp=None):
        """Return (R, t) target <- source at stamp, falling back to the latest transform."""
        for when in ([Time.from_msg(stamp)] if stamp is not None else []) + [Time()]:
            try:
                tf = self.tf.lookup_transform(target, source, when,
                                              timeout=Duration(seconds=0.0))
                self.last_lookup_stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
                return transform_matrix(tf)
            except TransformException:
                continue
        return None

    def on_ball(self, msg):
        """Fuse a detection (any frame) after moving it into map."""
        rt = self.lookup(self.map_frame, msg.header.frame_id, msg.header.stamp)
        if rt is None:
            return
        R, t = rt
        local = np.array([msg.point.x, msg.point.y, msg.point.z])
        world = R @ local + t
        t_obs = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.ball.update(world[:2], t_obs if t_obs > 0 else self.now(),
                         float(np.hypot(local[0], local[1])))
        self.grid.observe(np.zeros_like(self.grid.p, bool), 0.0, world[:2])

    def scan_transform(self, target, source, stamp):
        """Require measurement-time TF: latest fallback would invent map velocity."""
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        try:
            tf = self.tf.lookup_transform(target, source, Time.from_msg(stamp),
                                           timeout=Duration(seconds=0.0))
            return transform_matrix(tf)
        except TransformException:
            return None

    def on_scan(self, msg):
        """Same scan feeds legacy avoidance candidates and one confirmed opponent."""
        if not self.opp_enabled:
            return
        self.opp_pending.append(msg)
        self.process_opponent_scans()

    def process_opponent_scans(self):
        """Retry delayed TF without blocking callbacks or replacing measurement time."""
        while self.opp_pending:
            msg = self.opp_pending[0]
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            last = self.opponent_tracker.last_scan
            if stamp <= 0 or (last is not None and stamp <= last):
                self.opp_pending.popleft()
                continue
            rt = self.scan_transform(self.map_frame, msg.header.frame_id, msg.header.stamp)
            if rt is None:
                if self.now()-stamp <= self.opp_tf_wait:
                    break
                self.opp_expired += 1
                self.opp_pending.popleft()
                continue
            self.opp_pending.popleft()
            self.process_opponent_scan(msg, rt)

    def process_opponent_scan(self, msg, rt):
        start = time.perf_counter()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        alignment = self.scan_transform(self.map_frame, self.odom_frame, msg.header.stamp)
        if alignment is not None:
            ar, at = alignment
            pose = (float(at[0]), float(at[1]), math.atan2(ar[1, 0], ar[0, 0]))
            previous = self.opp_map_alignment
            if previous is not None and (math.hypot(pose[0]-previous[0], pose[1]-previous[1])
                                         > .15 or abs(wrap(pose[2]-previous[2])) > .35):
                self.opponent_tracker.reset()
                self.opponents, self.opp_time = [], None
            self.opp_map_alignment = pose
        R, t = rt
        pts = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                          msg.range_min, msg.range_max)
        world = (R[:2, :2] @ pts.T).T + t[:2]
        # Preserve the pre-existing candidate geometry/radii for MPC, without confirmation.
        legacy = cluster_opponents(world, self.field.inner_walls, **self.opp_cfg)
        if self.opp_time is None or stamp > self.opp_time:
            self.opponents, self.opp_time = legacy, stamp
        structures = [o.rect for o in self.field.obstacles_seen_at(float(t[2]))
                      if 'roof' not in o.name]
        candidates = indexed_candidates(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            R, t, self.field.inner_walls, structures, **self.opp_cfg, **self.opp_extra)
        self.opponent_tracker.update(candidates, stamp)
        self.opp_processed += 1
        if self.opponent_status_pub.get_subscription_count():
            self.opponent_status_pub.publish(String(data=json.dumps(dict(
                stamp=stamp, processed=self.opp_processed, expired_tf=self.opp_expired,
                pickup_age_s=self.now()-stamp,
                legacy_count=len(legacy), candidate_count=len(candidates),
                candidates=[dict(x=c.x, y=c.y, size=c.size, points=c.count)
                            for c in candidates],
                confirmed=self.opponent_tracker.sample(stamp) is not None,
                velocity_valid=self.opponent_tracker.velocity_valid,
                processing_ms=1000*(time.perf_counter()-start)))))

    # ------------------------------------------------------------------ output
    def on_localization_pose(self, msg):
        """Keep the LiDAR covariance and freshness separate from wheel odometry."""
        if msg.header.frame_id != self.map_frame:
            return
        self.last_localization_received = time.monotonic()
        c = msg.pose.covariance
        if self.localization == 'tf':
            self.pose_cov = [c[i] for i in (0, 1, 5, 6, 7, 11, 30, 31, 35)]

    def update_pose_from_tf(self):
        self.last_lookup_stamp=None
        rt = self.lookup(self.map_frame, self.base_frame)
        self.tf_pose_ready = rt is not None
        if rt is None:
            return
        R, t = rt
        pose = (float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0]))
        if self.pose is not None and (math.hypot(pose[0]-self.pose[0], pose[1]-self.pose[1]) > .25
                                     or abs(wrap(pose[2]-self.pose[2])) > .7):
            # A localization reseed changes the interpretation of earlier map fixes.
            self.ball = BallFilter(self.ball.p, self.field.inner_walls, self.ball_radius)
            self.grid.p.fill(self.grid.prior)
            self.opponents, self.opp_time = [], None
            self.opponent_tracker.reset()
        self.pose = pose
        self.pose_source_stamp=self.last_lookup_stamp

    def tick(self):
        """Predict, update the search grid and publish."""
        now = self.now()
        if self.last_tick is not None and now < self.last_tick:
            self.opponent_tracker.reset()
            self.opponents, self.opp_time = [], None
            self.opp_map_alignment = None
            self.opp_pending.clear()
        dt = 0.0 if self.last_tick is None else max(0.0, now - self.last_tick)
        self.last_tick = now
        if self.localization == 'tf':
            self.update_pose_from_tf()
        self.process_opponent_scans()
        if self.pose is None:
            self.get_logger().warn('Waiting for self pose', throttle_duration_sec=5.0)
            return
        self.ball.predict(now)
        state = self.ball.state(now)

        self.grid.relax(dt)
        camera_alive = self.camera_seen is not None and now - self.camera_seen < 0.5
        if camera_alive and state != OBSERVED:
            mask = self.grid.fov_mask(self.pose[0], self.pose[1], self.pose[2],
                                      self.hfov or self.hfov_fallback,
                                      self.fov_min, self.fov_max)
            self.grid.observe(mask, dt)

        if self.ball.initialized:
            x, y, vx, vy = self.ball.x
            sigma = self.ball.max_sigma()
            confidence = {OBSERVED: 1.0, LOST: 0.0}.get(
                state, max(0.0, 1.0 - sigma / self.ball.p.lost_sigma))
            ball = tracked('ball', (x, y, 0.0), (vx, vy), self.ball.covariance3(), state,
                           self.ball.last_seen, confidence)
        else:
            bx, by = self.grid.best_cell()
            ball = tracked('ball', (bx, by, 0.0), cov=self.ball.covariance3(),
                           state=TrackedObject.LOST, confidence=0.0)

        opponents = []
        sample = self.opponent_tracker.sample(now) if self.opp_enabled else None
        if sample is not None:
            x, y = sample['position']
            v = sample['variance']
            opponents.append(tracked(
                'opp_0', (x, y, 0.), sample['velocity'],
                [v, 0, 0, 0, v, 0, 0, 0, 10.], sample['state'],
                sample['last_seen'], sample['confidence']))
        obstacles = []
        if self.opp_time is not None and 0 <= now-self.opp_time <= self.opp_hold:
            for i, (ox, oy, _) in enumerate(self.opponents):
                obstacles.append(tracked(
                    f'candidate_{i}', (ox, oy, 0.),
                    cov=[.01, 0, 0, 0, .01, 0, 0, 0, 10.],
                    state=TrackedObject.OBSERVED if now-self.opp_time <= .3
                    else TrackedObject.ESTIMATED, last_seen=self.opp_time))

        me = tracked('self', self.pose, self.velocity, self.pose_cov, TrackedObject.OBSERVED,
                     self.pose_source_stamp or 0., 1.0)
        odom_fresh = (self.odom_timeout <= 0 or self.localization != 'odom_only'
                      or (self.last_odom_received is not None
                          and time.monotonic() - self.last_odom_received <= self.odom_timeout))
        if self.localization == 'tf' and self.localization_timeout > 0:
            odom_fresh = (self.tf_pose_ready and self.last_localization_received is not None
                          and time.monotonic()-self.last_localization_received <= self.localization_timeout
                          and self.last_odom_received is not None
                          and time.monotonic()-self.last_odom_received <= .5)
            if not odom_fresh:
                me.state, me.confidence = TrackedObject.ESTIMATED, 0.0
        self.pub.publish(world_model(self.get_clock().now().to_msg(), self.field, me, ball,
                                     opponents, self.playing and odom_fresh, self.grid))
        self.obstacle_pub.publish(world_model(self.get_clock().now().to_msg(), self.field,
                                              me, ball, obstacles,
                                              self.playing and odom_fresh))


def main(args=None):
    """Run the world model."""
    rclpy.init(args=args)
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
