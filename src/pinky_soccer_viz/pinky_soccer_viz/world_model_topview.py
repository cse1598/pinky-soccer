"""Publish an OpenCV top view of /world_model on /world_model/topview (sensor_msgs/Image).

Optional evaluation overlay (simulation only, never a decision input): grey dots from
/ground_truth (robot, PoseStamped in `world` = field centre) and /sim/ball_ground_truth.
Set `snapshot_dir` and `snapshot_period` > 0 to also save PNG frames.
"""

from collections import deque
import math
import os
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
from pinky_soccer_viz.topview import TopView
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class WorldModelTopView(Node):
    """Renders the shared display rules into an image topic."""

    def __init__(self):
        super().__init__('world_model_topview')
        p = self.declare_parameter
        field = Field.from_file(p('field_spec', '').value or None,
                                p('our_goal_color', 'yellow').value)
        self.view = TopView(field, p('px_per_m', 200.0).value)
        self.show_truth = p('show_ground_truth', True).value
        self.hfov = math.radians(p('camera_hfov_deg', 60.0).value)
        self.fov_range = p('fov_range', 1.5).value
        self.snapshot_dir = p('snapshot_dir', '').value
        self.snapshot_period = p('snapshot_period', 0.0).value
        self.wm = self.goal = None
        self.truth = {}
        self.arrivals = deque(maxlen=60)
        self.last_snapshot = 0.0
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'world_model/topview', 2)
        self.create_subscription(WorldModel, 'world_model', self.on_wm, 10)
        self.create_subscription(BehaviorGoal, 'behavior_goal', self.on_goal, 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        if self.show_truth:
            self.create_subscription(PoseStamped, 'ground_truth',
                                     lambda m: self.on_truth('robot', m), 10)
            self.create_subscription(PoseStamped, 'sim/ball_ground_truth',
                                     lambda m: self.on_truth('ball', m), 10)
        self.create_timer(1.0 / p('rate_hz', 10.0).value, self.tick)
        if self.snapshot_dir:
            os.makedirs(self.snapshot_dir, exist_ok=True)

    def on_wm(self, msg):
        """Keep the latest world model and its arrival time."""
        self.wm = msg
        self.arrivals.append(time.monotonic())

    def on_goal(self, msg):
        """Keep the latest behaviour goal."""
        self.goal = msg

    def on_info(self, msg):
        """Use the real horizontal FOV when the camera is calibrated."""
        if msg.k[0] > 0 and msg.width > 0:
            self.hfov = 2 * math.atan(msg.width / (2 * msg.k[0]))

    def on_truth(self, name, msg):
        """Store a ground-truth position (evaluation only)."""
        self.truth[name] = (msg.pose.position.x, msg.pose.position.y)

    def hz(self):
        """Return the /world_model arrival rate over the last samples."""
        now = time.monotonic()
        recent = [t for t in self.arrivals if now - t < 2.0]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def tick(self):
        """Render and publish one frame (and a snapshot if due)."""
        img = self.view.render(self.wm, self.goal, self.truth if self.show_truth else None,
                               self.hz(), self.hfov, self.fov_range)
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        self.pub.publish(msg)
        now = time.monotonic()
        if self.snapshot_dir and self.snapshot_period > 0 and \
                now - self.last_snapshot >= self.snapshot_period:
            self.last_snapshot = now
            sim_t = self.get_clock().now().nanoseconds * 1e-9
            cv2.imwrite(os.path.join(self.snapshot_dir, f'topview_{sim_t:08.2f}.png'), img)


def main(args=None):
    """Run the top view."""
    rclpy.init(args=args)
    node = WorldModelTopView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
