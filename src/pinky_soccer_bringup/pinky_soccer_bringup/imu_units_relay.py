#!/usr/bin/env python3
"""Convert the unmodified official BNO055 driver's degrees/s to ROS radians/s."""

from copy import deepcopy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def to_si(message):
    """Convert gyro and covariance only; retain original sample time and sensor axes."""
    result = deepcopy(message)
    scale = math.pi / 180.0
    gyro = result.angular_velocity
    acc = result.linear_acceleration
    if not all(math.isfinite(v) for v in (gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z)):
        raise ValueError('Nonfinite IMU observation')
    gyro.x *= scale
    gyro.y *= scale
    gyro.z *= scale
    if result.angular_velocity_covariance[0] != -1:
        result.angular_velocity_covariance = [
            float(value) * scale**2 for value in result.angular_velocity_covariance]
    return result


class ImuUnitsRelay(Node):
    """Expose imu_raw in SI and keep the native driver stream separately accessible."""

    def __init__(self):
        """Create sensor-data QoS endpoints without accessing hardware."""
        super().__init__('imu_units_relay')
        self.publisher = self.create_publisher(Imu, 'imu_raw', qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu_native', self.on_message, qos_profile_sensor_data)

    def on_message(self, message):
        """Drop malformed observations instead of substituting artificial values."""
        try:
            self.publisher.publish(to_si(message))
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5)


def main(args=None):
    """Run the unit adapter next to the optional official BNO055 driver."""
    rclpy.init(args=args)
    node = ImuUnitsRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
