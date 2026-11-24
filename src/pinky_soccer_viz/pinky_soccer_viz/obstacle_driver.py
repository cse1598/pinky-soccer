"""
Obstacle robot for movement tests: stand still or drive back and forth along its heading.

mode static        : publish zero commands (the robot stays where it was placed)
mode lateral       : placed sideways by the launch, so back and forth = left-right on the field
mode longitudinal  : placed facing the other robot, so back and forth = along the field
Both moving modes drive forward until the robot is travel/2 ahead of its start position along
its start heading, then backward to travel/2 behind it, and so on, steering back onto the
start line (heading and sideways offset). Commands go through cmd_vel -> command_watchdog.

Pose source: odometry by default (real robot, no localisation needed); in simulation the
launch passes pose_topic:=/<name>/ground_truth, because wheel slip at every reversal turned
the odometry-only line by about 30 deg per minute in MuJoCo. The obstacle is a test prop,
not a player, so using the simulator truth here does not feed any decision.
"""

import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

MODES = ('static', 'lateral', 'longitudinal')


def yaw_of(q):
    """Return the yaw of a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Shuttle:
    """
    Back-and-forth along the start heading (call begin() first); pure logic for tests.

    Commands take a while to act (watchdog, bridge, motor ramp; in a loaded simulation
    0.35 s of sim time was measured), so turning exactly at +-travel/2 overshoots. The
    turning points move inwards by the overshoot measured at each end (learning gain), so
    the actual swing converges to the set travel whatever the delay.
    """

    GAIN = 0.7

    def __init__(self, speed, travel):
        self.speed, self.half = abs(speed), abs(travel) / 2
        self.start = None
        self.direction = 1.0
        self.margin = {1.0: 0.0, -1.0: 0.0}       # inward shift of each turning point
        self.extreme = {1.0: 0.0, -1.0: 0.0}      # furthest point reached on each side
        self.along = 0.0

    def begin(self, x, y, heading):
        """Remember the start pose."""
        self.start = (x, y, heading)

    def steer(self, x, y, yaw, v, k_heading=2.0, k_offset=4.0, limit=1.0):
        """Turn rate that keeps the robot on its start line (heading + sideways offset)."""
        sx, sy, heading = self.start
        offset = -(x - sx) * math.sin(heading) + (y - sy) * math.cos(heading)   # left +
        err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        w = k_heading * err - k_offset * offset * math.copysign(1.0, v)
        return max(-limit, min(limit, w))

    def command(self, x, y):
        """Return the linear speed for the current position."""
        sx, sy, heading = self.start
        along = (x - sx) * math.cos(heading) + (y - sy) * math.sin(heading)
        self.along = along
        for side in (1.0, -1.0):
            self.extreme[side] = max(self.extreme[side], side * along)
        side = self.direction
        if side * along >= self.half - self.margin[side]:
            # Turning at this side: learn from the other side's completed excursion.
            other = -side
            if self.extreme[other] > 0:
                error = self.extreme[other] - self.half
                self.margin[other] = min(max(self.margin[other] + self.GAIN * error, 0.0),
                                         0.8 * self.half)
            self.extreme[other] = 0.0
            self.direction = other
        return self.direction * self.speed


class ObstacleDriver(Node):
    """cmd_vel for an obstacle robot."""

    def __init__(self):
        super().__init__('obstacle_driver')
        p = self.declare_parameter
        self.mode = p('mode', 'static').value
        if self.mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.shuttle = Shuttle(p('speed', 0.10).value, p('travel', 0.4).value)
        self.odom = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        pose_topic = p('pose_topic', '').value
        if pose_topic:
            self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        else:
            self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'obstacle {self.mode}: speed {self.shuttle.speed:.2f} m/s, '
                               f'travel {2 * self.shuttle.half:.2f} m')

    def on_odom(self, msg):
        """Keep the latest odometry pose."""
        q = msg.pose.pose.orientation
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(q))

    def on_pose(self, msg):
        """Keep the latest pose (simulation ground truth)."""
        self.odom = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))

    def tick(self):
        """Publish the obstacle command."""
        cmd = Twist()
        if self.mode != 'static' and self.odom is not None:
            if self.shuttle.start is None:
                self.shuttle.begin(*self.odom)
            cmd.linear.x = self.shuttle.command(self.odom[0], self.odom[1])
            cmd.angular.z = self.shuttle.steer(*self.odom, cmd.linear.x)
        self.pub.publish(cmd)


def main(args=None):
    """Run the obstacle driver."""
    rclpy.init(args=args)
    node = ObstacleDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:  # noqa: B902 - context may already be shut down
            pass
        node.destroy_node()
        rclpy.try_shutdown()
