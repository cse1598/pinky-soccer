"""ROS acquisition and independent motion watchdog for the calibration desktop."""

import csv
from collections import defaultdict, deque
from itertools import islice
import json
import math
import threading
import time
import uuid

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pinky_soccer_msgs.srv import CaptureFrame
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import (BatteryState, CameraInfo, CompressedImage, Image, Imu,
                             JointState, LaserScan, Range)
from std_msgs.msg import Float32, String, UInt16MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from pinky_soccer_world_model.ball_detector import transform_matrix
from pinky_soccer_bringup.time_sync import chrony_status, pc_reference_check
from .measurement_geometry import MeasurementGeometry
from .camera_capture import decode_capture

PREVIEW_TOPIC = '/camera/image_raw/compressed'
MOTION_LINEAR_SPEED = 0.08  # m/s, forward/backward calibration experiments
MOTION_ANGULAR_SPEED = 0.30  # rad/s, left/right calibration experiments
CAPTURE_IMAGE_TOPIC = '/calibration/capture/image_raw'
CAPTURE_INFO_TOPIC = '/calibration/capture/camera_info'
TOPICS = {'/robot_description': String, PREVIEW_TOPIC: CompressedImage, '/camera/camera_info': CameraInfo,
          '/cmd_vel': Twist,
          '/odom': Odometry, '/joint_states': JointState, '/scan': LaserScan,
          '/imu_raw': Imu, '/imu_native': Imu,
          '/us_sensor/range': Range, '/ir_sensor/range': UInt16MultiArray,
          '/batt_state': BatteryState, '/battery/voltage': Float32, '/battery/percent': Float32,
          # Robot-side diagnostics (motor_diagnostics:=true on the robot); JSON payloads.
          '/motor/diagnostics': String, '/motor/info': String,
          **{f'/sim/{link}/distance': Range for link in ('ir_l_link', 'ir_mid_link', 'ir_r_link')}}
BAG_TOPICS = {**TOPICS, CAPTURE_IMAGE_TOPIC: Image, CAPTURE_INFO_TOPIC: CameraInfo}


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class Acquisition(Node):
    def __init__(self):
        super().__init__('pinky_calibration_ui')
        # Robot namespace: fixed when the node starts (subscriptions); '' = no namespace.
        # Logical topic keys below stay '/scan', '/odom', ...; only the subscribed name changes.
        # 'auto' (default): the saved connection settings (connection_settings.py).
        from .connection_settings import load as load_connection
        saved = load_connection()
        ns = str(self.declare_parameter('robot_namespace', 'auto').value or '')
        ns = (saved.robot.namespace if ns == 'auto' else ns).strip('/')
        self.namespace = '/'+ns if ns else ''
        # PC IPv4 the robot must use as its chrony server (connection settings, editable).
        self.pc_ip = str(self.declare_parameter('pc_ip', '').value or '') or saved.pc_ip
        self.lock = threading.RLock()
        self.latest, self.seen = {}, {}
        self.geometry = None
        self.geometry_error = 'robot_description 수신 대기'
        self.values = defaultdict(lambda: deque(maxlen=20000))
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.writer = self.csv_file = None
        self.recording_session = None
        self.capture_service_name = self.declare_parameter(
            'capture_service', '/camera/capture_frame').value
        self.capture_timeout = float(self.declare_parameter('capture_timeout', 3.0).value)
        if not math.isfinite(self.capture_timeout) or not 0 < self.capture_timeout <= 10:
            raise ValueError('capture_timeout은 0초 초과~10초 이하로 설정하세요.')
        self.capture_client = self.create_client(CaptureFrame, self.capture_service_name)
        self.motion = None
        self.sync_enabled = False
        self.sync_state = {'ready': False, 'reason': '실물 모드에서 자동 확인'}
        self.robot_clock = None
        self.robot_clock_seen = -math.inf
        self.create_subscription(String, self.robot_topic('/calibration/time_sync_status'),
                                 self.clock_received, 10)
        self.last_motion = None
        self.publisher = self.create_publisher(Twist, self.robot_topic('/cmd_vel'), 10)
        for topic, typ in TOPICS.items():
            self.create_subscription(typ, self.robot_topic(topic),
                                     lambda msg, t=topic: self.receive(t, msg),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                if topic in ('/robot_description', '/motor/info') else
                QoSProfile(depth=50) if topic == '/motor/diagnostics' else qos_profile_sensor_data)
        ping_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.ping_pub = self.create_publisher(String, self.robot_topic('/calibration/ping'), ping_qos)
        self.pongs = None
        self.create_subscription(String, self.robot_topic('/calibration/pong'), self.on_pong,
                                 ping_qos)
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.sync_thread = threading.Thread(target=self.clock_loop, daemon=True)
        self.sync_thread.start()

    def robot_topic(self, topic):
        """Absolute robot topic for a logical key, inside the configured namespace."""
        return self.namespace + topic

    def clock_received(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            value['_received_unix'] = time.time()
            self.robot_clock, self.robot_clock_seen = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    @staticmethod
    def robot_chrony(message):
        """Chrony data of a robot status message; legacy robots only send tracking()."""
        if not message:
            return None
        if isinstance(message.get('chrony'), dict):
            return message['chrony']
        legacy = dict(message)       # old bringup: no sources/sourcestats, absolute offsets
        legacy['collected_unix'] = message.get('_received_unix')
        legacy['legacy_status'] = True
        return legacy

    def clock_loop(self):
        while self.running:
            if self.sync_enabled:
                pc = chrony_status()
                robot = self.robot_chrony(self.robot_clock)
                try:
                    check = pc_reference_check(pc, robot, self.pc_ip)
                except (ValueError, KeyError, TypeError) as error:
                    check = {'relative_verified': False, 'reason': f'판정 오류: {error}'}
                ready = bool(check.get('relative_verified'))
                self.sync_state = {'ready': ready, 'pc': pc, 'robot': robot, 'check': check,
                    'architecture': 'pc_centred', 'reason': check.get('reason', ''),
                    'observed_monotonic': time.monotonic(), 'observed_unix': time.time()}
            for _ in range(20):
                if not self.running:
                    return
                time.sleep(.1)

    def synchronized(self):
        return (self.sync_enabled and self.sync_state.get('ready', False)
                and time.monotonic()-self.sync_state.get('observed_monotonic', -math.inf) < 10)

    def loop(self):
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=.02)
                self.watchdog()
        finally:
            self.stop('ROS 스레드 종료')

    def receive(self, topic, msg, provenance=None):
        now = time.monotonic()
        stamp = None
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        observation = {'clock_sync_verified': self.synchronized()}
        if provenance is not None:
            observation['capture'] = provenance
        if stamp is not None:
            observation['header_age_s'] = self.get_clock().now().nanoseconds / 1e9 - stamp
        if topic == '/robot_description':
            try:
                geometry = MeasurementGeometry(msg.data)
                with self.lock:
                    self.geometry, self.geometry_error = geometry, ''
            except Exception as error:
                with self.lock:
                    self.geometry, self.geometry_error = None, str(error)
        if topic in ('/scan', '/us_sensor/range'):
            observation['sensor_frame'] = msg.header.frame_id
            with self.lock:
                geometry = self.geometry
            observation['robot_description_sha256'] = geometry.sha256 if geometry else None
        if topic == '/scan':
            try:
                observation['distance'] = geometry.scan_distance(msg) if geometry else float('nan')
            except ValueError as error:
                observation.update(distance=float('nan'), geometry_error=str(error))
        elif topic == '/us_sensor/range':
            observation['distance'] = (float(msg.range) if math.isfinite(msg.range)
                and msg.min_range <= msg.range <= msg.max_range else float('nan'))
        elif topic in ('/imu_raw', '/imu_native'):
            observation['orientation_xyzw'] = [getattr(msg.orientation, a) for a in 'xyzw']
            for kind in ('angular_velocity', 'linear_acceleration'):
                for axis in 'xyz':
                    observation[f'{kind}_{axis}'] = float(getattr(getattr(msg, kind), axis))
        elif topic == '/ir_sensor/range':
            observation['adc'] = list(msg.data)
        elif topic in ('/battery/voltage', '/battery/percent'):
            observation['value'] = float(msg.data)
        elif topic == '/batt_state':
            observation.update({key: float(getattr(msg, key)) for key in
                                ('voltage', 'percentage', 'current', 'temperature')})
        elif topic.startswith('/sim/ir_'):
            observation['sim_distance_m'] = float(msg.range)
        elif topic == '/cmd_vel':
            observation.update(linear_x=msg.linear.x, angular_z=msg.angular.z)
        elif topic in ('/motor/diagnostics', '/motor/info'):
            try:
                observation['payload'] = json.loads(msg.data)
            except ValueError:
                observation['payload'] = None
        elif topic == '/joint_states':
            observation.update(name=list(msg.name), position=[float(v) for v in msg.position],
                               velocity=[float(v) for v in msg.velocity])
        elif topic == '/odom':
            p, t = msg.pose.pose, msg.twist.twist
            observation.update(x=p.position.x, y=p.position.y, yaw=yaw(p.orientation),
                               v=t.linear.x, w=t.angular.z)
        with self.lock:
            self.latest[topic], self.seen[topic] = msg, now
            self.values[topic].append((now, stamp, observation))
            if self.writer:
                # Bag uses reception ROS time; CSV separately preserves header and wall time.
                ros_ns = self.get_clock().now().nanoseconds
                self.writer.write(topic, serialize_message(msg), ros_ns)
                self.csv.writerow([topic, ros_ns, time.time_ns(), now, stamp,
                                   json.dumps(observation)])

    def begin(self, session):
        with self.lock:
            if self.writer:
                raise ValueError('먼저 현재 기록을 종료하세요.')
            suffix = uuid.uuid4().hex
            bag = 'raw_bag' if not (session.root/'raw_bag').exists() else 'raw_bag_'+suffix
            csv_name = ('observations.csv' if not (session.root/'observations.csv').exists()
                        else 'observations_'+suffix+'.csv')
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(session.root/bag),
                                                      storage_id='sqlite3'),
                             rosbag2_py.ConverterOptions('', ''))
            for i, (topic, typ) in enumerate(BAG_TOPICS.items()):
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=i, name=topic, type=typ.__module__.split('.')[0]+'/msg/'+typ.__name__,
                    serialization_format='cdr'))
            self.csv_file = (session.root/csv_name).open('x', newline='')
            self.writer = writer
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['topic', 'receive_ros_ns', 'receive_unix_ns', 'receive_monotonic',
                               'header_seconds', 'observation_json'])
            self.values.clear()
            self.recording_session = session
            session.meta.setdefault('recordings', []).append(
                {'bag': bag, 'csv': csv_name, 'started_unix_s': time.time(), 'completed': False})
            session.meta['image_acquisition'] = {
                'preview_topic': PREVIEW_TOPIC, 'capture_service': self.capture_service_name,
                'capture_format': 'raw', 'raw_recording': 'service_captures_only',
                'capture_image_bag_topic': CAPTURE_IMAGE_TOPIC,
                'capture_info_bag_topic': CAPTURE_INFO_TOPIC}
            session.save()
            # RSP may publish only once before recording starts; preserve that exact model.
            if '/robot_description' in self.latest:
                self.receive('/robot_description', self.latest['/robot_description'])

    def measurement_geometry(self):
        with self.lock:
            if self.geometry is None:
                raise ValueError('범퍼 기준 자동 변환 불가: '+self.geometry_error)
            return self.geometry

    def monitor_snapshot(self):
        """Copy references to immutable received messages and a bounded timing window."""
        with self.lock:
            arrivals = {topic: [v[0] for v in reversed(list(islice(reversed(series), 30)))]
                        for topic, series in self.values.items()}
            return dict(self.latest), dict(self.seen), arrivals

    def end(self):
        self.stop('기록 종료')
        with self.lock:
            session = self.recording_session
            self.writer = None
            self.recording_session = None
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            if session:
                session.meta['recordings'][-1]['completed'] = True
                session.meta['recordings'][-1]['ended_unix_s'] = time.time()
                session.save()

    def fresh(self, topic, age=.6):
        return topic in self.seen and time.monotonic()-self.seen[topic] < age

    def frame(self):
        """Decode only the JPEG preview; callers must use capture_raw for calibration photos."""
        with self.lock:
            if not self.fresh(PREVIEW_TOPIC):
                raise ValueError('압축 미리보기가 없거나 지연됐습니다. launch와 ROS domain을 확인하세요.')
            message = self.latest[PREVIEW_TOPIC]
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(message, 'bgr8')
            if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
                raise ValueError('JPEG 영상 형식 오류')
            return frame.copy()
        except Exception as error:
            raise ValueError('압축 미리보기 디코딩 실패: '+str(error)) from error

    def review_frame(self):
        """Take a consistent JPEG/header/info snapshot without adding a raw subscription."""
        with self.lock:
            frame = self.frame()
            msg = self.latest[PREVIEW_TOPIC]
            info = self.latest.get('/camera/camera_info') if self.fresh('/camera/camera_info', 2.) else None
            return frame, msg.header, info, self.seen[PREVIEW_TOPIC]

    def review_transform(self, target, source, stamp=None):
        """Use image-time TF when available, then the same latest fallback as gameplay."""
        for when in ([Time.from_msg(stamp)] if stamp is not None else []) + [Time()]:
            try:
                tf = self.tf.lookup_transform(target, source, when, timeout=Duration(seconds=0.))
                return transform_matrix(tf)
            except TransformException:
                pass
        return None

    def capture_raw(self):
        """Wait on a UI worker while the existing ROS thread handles the service response."""
        if not self.capture_client.wait_for_service(timeout_sec=2.0):
            raise ValueError('원본 촬영 서비스가 없습니다. 로봇의 msgs/bringup 갱신과 '
                             '카메라 launch·ROS 설정을 확인하세요.')
        sent_ros_ns = self.get_clock().now().nanoseconds
        sent = time.monotonic()
        future = self.capture_client.call_async(CaptureFrame.Request(
            format='raw', timeout_sec=self.capture_timeout))
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(self.capture_timeout+5.0):
            future.cancel()
            raise ValueError('원본 촬영 응답 시간 초과. 미리보기 사진으로 대체하지 않습니다.')
        try:
            result = future.result()
        except Exception as error:
            raise ValueError('원본 촬영 통신 실패: '+str(error)) from error
        return decode_capture(result, self.capture_service_name, sent_ros_ns, sent,
                              self.synchronized())

    def record_capture(self, sample, session):
        """Preserve service-returned ROS payloads without adding any video subscription."""
        with self.lock:
            if not self.writer or self.recording_session is not session:
                raise ValueError('원본 촬영 중 기록 세션이 종료되거나 바뀌었습니다.')
            self.receive(CAPTURE_IMAGE_TOPIC, sample.image, sample.metadata)
            self.receive(CAPTURE_INFO_TOPIC, sample.camera_info, sample.metadata)

    def joints(self):
        msg = self.latest['/joint_states']
        mapping = dict(zip(msg.name, msg.position))
        return [float(mapping[n]) for n in ('l_wheel_joint', 'r_wheel_joint')]

    def start_motion(self, kind):
        with self.lock:
            if not self.writer:
                raise ValueError('새 실험을 만들어 원본 기록부터 시작하세요.')
            if self.motion:
                raise ValueError('진행 중입니다. 먼저 정지하세요.')
            if not self.fresh('/odom') or not self.fresh('/joint_states'):
                raise ValueError('최신 odom·joint_states가 모두 필요합니다.')
            msg = self.latest['/odom']
            start = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw(msg.pose.pose.orientation)]
            self.motion = dict(kind=kind, start=start, last_yaw=start[2], angle=0.,
                               joints=self.joints(), started=time.monotonic())

    def stop(self, reason='사용자 정지'):
        with self.lock:
            for _ in range(3):
                self.publisher.publish(Twist())
            if self.motion:
                trial = self.motion
                trial['reason'] = reason
                trial['complete'] = reason in ('목표 도달', '프로파일 완료')
                trial['stopped_monotonic'] = time.monotonic()
                if '/joint_states' in self.latest:
                    end = self.joints()
                    trial['left_rad'] = end[0]-trial['joints'][0]
                    trial['right_rad'] = end[1]-trial['joints'][1]
                self.last_motion = trial
                self.motion = None
                if trial['kind'] == 'profile':
                    return          # saved by the experiment runner, not a wheel trial
                if self.recording_session:
                    self.recording_session.meta['pending_motion'] = dict(trial)
                    self.recording_session.save()

    def start_profile(self, plan):
        """Run a timed command profile [(duration_s, v, w), ...] (motor/braking experiments).

        Shares the single motion slot, the sensor-loss stop and the stop() path with the
        wheel experiments; adds a total time limit, an odom distance limit and a 50 Hz
        command schedule. Every published command is logged with its own timestamps.
        """
        with self.lock:
            if not self.writer:
                raise ValueError('기록 시작/재개 후 실행하세요 (원본 Bag/CSV 필요).')
            if self.motion:
                raise ValueError('다른 주행이 진행 중입니다. 먼저 정지하세요.')
            if not self.fresh('/odom') or not self.fresh('/joint_states'):
                raise ValueError('최신 odom·joint_states가 모두 필요합니다.')
            topic = self.robot_topic('/cmd_vel')
            others = self.count_publishers(topic) - 1
            if others > 0:
                raise ValueError(f'{topic}에 다른 발행자 {others}개 (BT/MPC/teleop). 모두 끄고 실행하세요.')
            msg = self.latest['/odom']
            start = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw(msg.pose.pose.orientation)]
            now = time.monotonic()
            self.motion = dict(kind='profile', plan=plan, start=start, started=now,
                               started_unix=time.time(), joints=self.joints(),
                               commands=[], next_send=now, last_yaw=start[2], angle=0.,
                               tag=0, publisher_check=now)

    def watchdog(self):
        with self.lock:
            m = self.motion
            if not m:
                return
            if m['kind'] == 'profile':
                return self.profile_step(m)
            if not self.fresh('/odom') or not self.fresh('/joint_states'):
                self.stop('센서 수신 중단')
                return
            if time.monotonic()-m['started'] > 60:
                self.stop('60초 제한')
                return
            pose = self.latest['/odom'].pose.pose
            current = yaw(pose.orientation)
            m['angle'] += math.atan2(math.sin(current-m['last_yaw']),
                                     math.cos(current-m['last_yaw']))
            m['last_yaw'] = current
            distance = math.hypot(pose.position.x-m['start'][0], pose.position.y-m['start'][1])
            rotating = m['kind'] in ('left', 'right')
            if (abs(m['angle']) >= 2*math.pi if rotating else distance >= 1.):
                self.stop('목표 도달')
                return
            msg = Twist()
            if rotating:
                msg.angular.z = MOTION_ANGULAR_SPEED if m['kind'] == 'left' else -MOTION_ANGULAR_SPEED
            else:
                msg.linear.x = MOTION_LINEAR_SPEED if m['kind'] == 'forward' else -MOTION_LINEAR_SPEED
            self.publisher.publish(msg)

    def profile_step(self, m):
        """One watchdog pass of a profile experiment (lock held by the caller)."""
        if not self.fresh('/odom') or not self.fresh('/joint_states'):
            return self.stop('센서 수신 중단')
        now = time.monotonic()
        elapsed = now - m['started']
        plan = m['plan']
        pose = self.latest['/odom'].pose.pose
        current = yaw(pose.orientation)
        m['angle'] += math.atan2(math.sin(current-m['last_yaw']), math.cos(current-m['last_yaw']))
        m['last_yaw'] = current
        distance = math.hypot(pose.position.x-m['start'][0], pose.position.y-m['start'][1])
        if distance > plan['max_distance_m']:
            return self.stop('최대 이동 거리 초과 (안전 정지)')
        if abs(m['angle']) > plan.get('max_angle_rad', 4*math.pi):
            return self.stop('최대 회전각 초과 (안전 정지)')
        total = sum(d for d, _, _ in plan['segments'])
        braking = plan.get('braking')
        if braking and m.get('cut_at') is None:
            self.braking_check(m, braking, elapsed)
        if m.get('cut_at') is not None:
            total = m['cut_at'] + plan['segments'][2][0]
        if elapsed >= total:
            return self.stop('프로파일 완료')
        if now - m['publisher_check'] > .25:          # also during the run, not only at start
            m['publisher_check'] = now
            others = self.count_publishers(self.robot_topic('/cmd_vel')) - 1
            if others > 0:
                return self.stop(f'다른 cmd_vel 발행자 {others}개 감지 (안전 정지)')
        if now < m['next_send']:
            return
        m['next_send'] = max(m['next_send'] + 1/plan['rate_hz'], now)
        t, v, w = 0., 0., 0.
        for duration, sv, sw in plan['segments']:
            if elapsed < t + duration:
                v, w = sv, sw
                break
            t += duration
        if m.get('cut_at') is not None and elapsed >= m['cut_at']:
            v, w = 0., 0.                             # braking: stop after steady speed
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        # Tag in a field the drivers ignore, so applied commands can be matched exactly.
        m['tag'] += 1
        cmd.linear.z = float(m['tag'])
        self.publisher.publish(cmd)
        m['commands'].append((now, self.get_clock().now().nanoseconds, time.time_ns(), v, w,
                              m['tag']))

    def on_pong(self, msg):
        recv_mono, recv_unix = time.monotonic_ns(), time.time_ns()
        try:
            reply = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            if self.pongs is not None and isinstance(reply, dict):
                self.pongs.append((recv_mono, recv_unix, reply))

    def pingpong(self, rate_hz, duration_s, payload_bytes=0):
        """Send diagnostic pings at rate_hz for duration_s (worker thread); returns raw data."""
        with self.lock:
            if self.pongs is not None:
                raise ValueError('이미 Ping-Pong 측정 중입니다.')
            self.pongs = []
        sent, period = {}, 1.0 / rate_hz
        pad = 'x' * int(payload_bytes)
        start = time.monotonic()
        try:
            for seq in range(1, int(duration_s * rate_hz) + 1):
                target = start + (seq - 1) * period
                delay = target - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                mono, unix = time.monotonic_ns(), time.time_ns()
                sent[seq] = {'pc_send_mono_ns': mono, 'pc_send_unix_ns': unix}
                self.ping_pub.publish(String(data=json.dumps(
                    {'seq': seq, 'pc_send_unix_ns': unix, 'pad': pad})))
            time.sleep(1.0)                           # late replies
        finally:
            with self.lock:
                replies, self.pongs = self.pongs, None
        return sent, replies

    def braking_check(self, m, braking, elapsed):
        """Mark the stop time once both wheels hold the commanded speed (braking runs)."""
        pre, hold = m['plan']['segments'][0][0], m['plan']['segments'][1][0]
        if elapsed < pre + braking['min_hold_s']:
            return
        if elapsed >= pre + hold:                     # upper limit: stop anyway, not steady
            m['cut_at'], m['braking_steady'] = elapsed, False
            return
        msg = self.latest.get('/joint_states')
        speeds = []
        if msg is not None:
            names = list(msg.name)
            for aliases in (('l_wheel_joint', 'left_wheel_joint'), ('r_wheel_joint', 'right_wheel_joint')):
                idx = next((names.index(a) for a in aliases if a in names), None)
                if idx is not None and idx < len(msg.velocity):
                    speeds.append(abs(msg.velocity[idx]))
        target = braking['expected_wheel_radps']
        steady = len(speeds) == 2 and all(abs(x - target) <= braking['tolerance'] * target
                                          for x in speeds)
        now = time.monotonic()
        if not steady:
            m['steady_since'] = None
        elif m.get('steady_since') is None:
            m['steady_since'] = now
        elif now - m['steady_since'] >= braking['steady_s']:
            m['cut_at'], m['braking_steady'] = elapsed, True

    def close(self):
        self.stop('UI 종료')
        self.running = False
        self.thread.join(timeout=2)
        self.sync_thread.join(timeout=3)
        self.end()
        self.destroy_node()
