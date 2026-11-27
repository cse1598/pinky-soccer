"""Read-only startup map/goal monitor, with explicit abort/recalibrate keys."""
import json
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .game_camera_view import compose_camera_view


class LocalizationView(Node):
    def __init__(self):
        super().__init__('localization_view')
        self.bridge = CvBridge()
        self.images = {}
        self.state = 'WAITING'
        self.details = ''
        self.create_subscription(Image, 'localization/map_image',
            lambda msg: self.receive('map', msg), qos_profile_sensor_data)
        self.create_subscription(Image, 'localization/goal_image',
            lambda msg: self.receive('goal', msg), qos_profile_sensor_data)
        self.create_subscription(String, 'localization/status', self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        preparing = self.declare_parameter('preparation_enabled', True).value
        self.abort = self.create_client(Trigger, 'preparation/stop' if preparing else 'localization/abort')
        self.recalibrate = self.create_client(Trigger, 'preparation/prepare' if preparing else 'localization/calibrate')

    def receive(self, key, msg):
        self.images[key] = (self.bridge.imgmsg_to_cv2(msg, 'bgr8'), time.monotonic())

    def on_status(self, msg):
        try:
            state = json.loads(msg.data)
            self.state = (f"{state['state']} | goals={','.join(state['learned_colors'])} | "
                          f"LED={state['led']}")
            if not state.get('debug_enabled', False):
                self.state += ' | DEBUG OFF: launch localization_debug:=true'
            if state.get('missing_inputs'):
                self.state += ' | waiting='+','.join(state['missing_inputs'])
            counts = state.get('goal_pixel_counts', {})
            self.details = (f"HLS Y={counts.get('yellow', 0)} B={counts.get('blue', 0)} "
                            f"streak={state.get('goal_streak', 0)} | "
                            f"confidence={state.get('confidence', 0.):.2f} | "
                            f"target={state.get('selected_goal_color') or state.get('selected_goal_sign')} "
                            f"offset={state.get('imu_map_offset_rad')}")
        except (ValueError, KeyError, TypeError):
            pass

    def render(self):
        now = time.monotonic()
        panels = []
        for key, label in (('map', 'LIDAR: green walls / red excluded'), ('goal', 'STARTUP HLS MASKS')):
            frame, seen = self.images.get(key, (None, now))
            panels.append((label, frame, now-seen))
        result = compose_camera_view(panels, 1280)
        cv2.putText(result, self.details, (10, result.shape[0]-50), cv2.FONT_HERSHEY_SIMPLEX,
                    .45, (255, 255, 255), 1)
        cv2.putText(result, self.state, (10, result.shape[0]-30), cv2.FONT_HERSHEY_SIMPLEX,
                    .45, (255, 255, 255), 1)
        cv2.putText(result, 'S: STOP | R: PREPARE / retry | ESC: close monitor',
                    (10, result.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationView()
    title = 'Pinky startup localization'
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            cv2.imshow(title, node.render())
            key = cv2.waitKey(1) & 255
            if key in (27, ord('q')):
                break
            client = node.abort if key == ord('s') else node.recalibrate if key == ord('r') else None
            if client and client.service_is_ready():
                client.call_async(Trigger.Request())
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
