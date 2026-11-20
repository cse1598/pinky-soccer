#!/usr/bin/env python3
"""Stop the drive when the application command heartbeat disappears."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CommandWatchdog(Node):
    """A separate process keeps UI failure from leaving a held drive command."""

    def __init__(self):
        """Require fresh finite commands and emit zero after a 0.5 s wall timeout."""
        super().__init__('soccer_command_watchdog')
        self.last = -math.inf
        self.command = Twist()
        self.output = self.create_publisher(Twist, 'drive/cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel', self.receive, 10)
        self.create_timer(.05, self.tick)

    def receive(self, msg):
        """Accept velocity commands only when all components are finite."""
        if all(math.isfinite(v) for vec in (msg.linear, msg.angular)
               for v in (vec.x, vec.y, vec.z)):
            self.command, self.last = msg, time.monotonic()

    def tick(self):
        """Use wall time so paused simulation clocks cannot suspend timeout checks."""
        self.output.publish(self.command if time.monotonic()-self.last < .5 else Twist())


def main(args=None):
    """Run independently from calibration or application processes."""
    rclpy.init(args=args)
    node = CommandWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
