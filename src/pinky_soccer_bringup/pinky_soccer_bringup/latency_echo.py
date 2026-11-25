#!/usr/bin/env python3
"""Diagnostic ping-pong responder (robot side); independent of the drive command path.

Request  calibration/ping  std_msgs/String JSON {"seq": int, ...}
Reply    calibration/pong  the request fields plus this robot's receive/send times:
         robot_recv_unix_ns, robot_recv_mono_ns, robot_send_unix_ns, robot_send_mono_ns.
Monotonic times are only compared on the same device; unix times are only comparable with
the PC when the clocks are verified to be synchronized (calibration UI decides).
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)


class LatencyEcho(Node):
    def __init__(self):
        super().__init__('calibration_latency_echo')
        self.pub = self.create_publisher(String, 'calibration/pong', QOS)
        self.create_subscription(String, 'calibration/ping', self.on_ping, QOS)

    def on_ping(self, msg):
        recv_unix, recv_mono = time.time_ns(), time.monotonic_ns()
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or 'seq' not in data:
                return
        except ValueError:
            return
        data.update(robot_recv_unix_ns=recv_unix, robot_recv_mono_ns=recv_mono,
                    robot_host=self.get_namespace())
        data['robot_send_mono_ns'] = time.monotonic_ns()
        data['robot_send_unix_ns'] = time.time_ns()
        self.pub.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = LatencyEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
