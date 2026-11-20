"""ROS acquisition and independent motion watchdog for the calibration desktop."""

import csv
from collections import defaultdict, deque
import json
import math
import threading
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, Range
from std_msgs.msg import String, UInt16MultiArray
from pinky_soccer_bringup.time_sync import relative_ready, tracking

TOPICS = {'/camera/image_raw': Image, '/camera/camera_info': CameraInfo,
          '/cmd_vel': Twist,
          '/odom': Odometry, '/joint_states': JointState, '/scan': LaserScan,
          '/imu_raw': Imu, '/us_sensor/range': Range, '/ir_sensor/range': UInt16MultiArray}


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class Acquisition(Node):
    def __init__(self):
        super().__init__('pinky_calibration_ui')
        self.lock = threading.RLock()
        self.latest, self.seen = {}, {}
        self.values = defaultdict(lambda: deque(maxlen=20000))
        self.bridge = CvBridge()
        self.writer = self.csv_file = None
        self.motion = None
        self.sync_enabled = False
        self.sync_state = {'ready': False, 'reason': '실물 모드에서 자동 확인'}
        self.robot_clock = None
        self.robot_clock_seen = -math.inf
        self.create_subscription(String, '/calibration/time_sync_status', self.clock_received, 10)
        self.last_motion = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        for topic, typ in TOPICS.items():
            self.create_subscription(typ, topic, lambda msg, t=topic: self.receive(t, msg),
                                     qos_profile_sensor_data)
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.sync_thread = threading.Thread(target=self.clock_loop, daemon=True)
        self.sync_thread.start()

    def clock_received(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            self.robot_clock, self.robot_clock_seen = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def clock_loop(self):
        while self.running:
            if self.sync_enabled:
                pc = tracking()
                robot = self.robot_clock if time.monotonic()-self.robot_clock_seen < 12 else None
                try:
                    ready = relative_ready(pc, robot)
                except (ValueError, KeyError, TypeError):
                    ready = False
                self.sync_state = {'ready': ready, 'pc': pc, 'robot': robot,
                    'reason': '자동 동기화 확인 (관측 오차 기준 5ms)' if ready else
                    '동기화 대기: PC/로봇 chrony 설치·UDP123·상태 토픽 확인',
                    'observed_monotonic': time.monotonic()}
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

    def receive(self, topic, msg):
        now = time.monotonic()
        stamp = None
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        observation = {'clock_sync_verified': self.synchronized()}
        if stamp is not None:
            observation['header_age_s'] = self.get_clock().now().nanoseconds / 1e9 - stamp
        if topic == '/scan':
            angles = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]
            vals = [float(r) for a, r in zip(angles, msg.ranges)
                    if abs(a) < .035 and msg.range_min <= r <= msg.range_max]
            observation['distance'] = sum(vals)/len(vals) if vals else float('nan')
        elif topic == '/us_sensor/range':
            observation['distance'] = float(msg.range)
        elif topic == '/imu_raw':
            for kind in ('angular_velocity', 'linear_acceleration'):
                for axis in 'xyz':
                    observation[f'{kind}_{axis}'] = float(getattr(getattr(msg, kind), axis))
        elif topic == '/ir_sensor/range':
            observation['adc'] = list(msg.data)
        elif topic == '/cmd_vel':
            observation.update(linear_x=msg.linear.x, angular_z=msg.angular.z)
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
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=str(session.root/'raw_bag'),
                                                      storage_id='sqlite3'),
                             rosbag2_py.ConverterOptions('', ''))
            for i, (topic, typ) in enumerate(TOPICS.items()):
                self.writer.create_topic(rosbag2_py.TopicMetadata(
                    id=i, name=topic, type=typ.__module__.split('.')[0]+'/msg/'+typ.__name__,
                    serialization_format='cdr'))
            self.csv_file = (session.root/'observations.csv').open('x', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['topic', 'receive_ros_ns', 'receive_unix_ns', 'receive_monotonic',
                               'header_seconds', 'observation_json'])
            self.values.clear()

    def end(self):
        self.stop('기록 종료')
        with self.lock:
            self.writer = None
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None

    def fresh(self, topic, age=.6):
        return topic in self.seen and time.monotonic()-self.seen[topic] < age

    def frame(self):
        with self.lock:
            if not self.fresh('/camera/image_raw'):
                raise ValueError('카메라 영상이 없거나 지연됐습니다. launch와 ROS domain을 확인하세요.')
            return self.bridge.imgmsg_to_cv2(self.latest['/camera/image_raw'], 'bgr8').copy()

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
                trial['complete'] = reason == '목표 도달'
                if '/joint_states' in self.latest:
                    end = self.joints()
                    trial['left_rad'] = end[0]-trial['joints'][0]
                    trial['right_rad'] = end[1]-trial['joints'][1]
                self.last_motion = trial
                self.motion = None

    def watchdog(self):
        with self.lock:
            m = self.motion
            if not m:
                return
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
                msg.angular.z = .15 if m['kind'] == 'left' else -.15
            else:
                msg.linear.x = .04 if m['kind'] == 'forward' else -.04
            self.publisher.publish(msg)

    def close(self):
        self.stop('UI 종료')
        self.running = False
        self.thread.join(timeout=2)
        self.sync_thread.join(timeout=3)
        self.end()
        self.destroy_node()
