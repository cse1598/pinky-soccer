#!/usr/bin/env python3
"""Reuse ADC battery samples instead of racing a second reader on I2C address 0x08."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


def battery_values(voltage, empty_voltage=6.8, full_voltage=7.6):
    """Retain pinkylib's nominal voltage estimate, not a measured state of charge."""
    if not all(math.isfinite(v) for v in (voltage, empty_voltage, full_voltage)):
        raise ValueError('Battery voltage and reference values must be finite')
    if voltage <= 0 or not 0 < empty_voltage < full_voltage:
        raise ValueError('Invalid battery voltage or reference interval')
    fraction = (voltage-empty_voltage)/(full_voltage-empty_voltage)
    percent = round(max(0., min(100., fraction*100)), 2)
    return float(voltage), percent


class BatteryStateRelay(Node):
    """Publish existing battery interfaces from one fresh ADC sample every five seconds."""

    def __init__(self):
        """Create ROS endpoints without opening or touching an I2C device."""
        super().__init__('soccer_battery_state_relay')
        self.empty = float(self.declare_parameter('empty_voltage', 6.8).value)
        self.full = float(self.declare_parameter('full_voltage', 7.6).value)
        battery_values(self.empty, self.empty, self.full)
        self.sample = None
        self.voltage = self.create_publisher(Float32, 'battery/voltage', 10)
        self.percent = self.create_publisher(Float32, 'battery/percent', 10)
        self.create_subscription(BatteryState, 'batt_state', self.receive, qos_profile_sensor_data)
        self.create_timer(5., self.publish)

    def receive(self, msg):
        """Keep one valid sample; an invalid observation immediately invalidates the cache."""
        try:
            values = battery_values(msg.voltage, self.empty, self.full)
        except ValueError:
            self.sample = None
            return
        self.sample = (time.monotonic(), values)

    def publish(self):
        """Never manufacture measurements or replay a disconnected ADC's last sample."""
        sample = self.sample
        if sample is None or time.monotonic()-sample[0] > 2.:
            self.get_logger().warning('Fresh batt_state missing; battery outputs withheld',
                                      throttle_duration_sec=10)
            return
        voltage, percent = sample[1]
        self.voltage.publish(Float32(data=voltage))
        self.percent.publish(Float32(data=percent))


def main(args=None):
    """Spin the battery adapter without any hardware access."""
    rclpy.init(args=args)
    node = BatteryStateRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
