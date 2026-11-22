#!/usr/bin/env python3
"""
Run a single-owner BNO055 NDOF driver without changing the official submodule.

Registers/units: Bosch BST-BNO055-DS000 rev 1.8. Linux I2C_RDWR preserves
the repeated-start transaction. No second process may configure this device.
"""

from copy import deepcopy
import ctypes
import fcntl
import json
import math
import os
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String


class I2CMessage(ctypes.Structure):
    """Linux struct i2c_msg."""

    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16),
                ('len', ctypes.c_uint16), ('buf', ctypes.POINTER(ctypes.c_uint8))]


class I2CTransfer(ctypes.Structure):
    """Linux struct i2c_rdwr_ioctl_data."""

    _fields_ = [('msgs', ctypes.POINTER(I2CMessage)), ('nmsgs', ctypes.c_uint32)]


class BNO055:
    """Bounded startup and exact-length register transfers."""

    def __init__(self, interface):
        self.fd = os.open(interface, os.O_RDWR)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if self.read(0, 1) != b'\xa0':
                raise OSError('BNO055 chip ID is not 0xA0')
            self.write(0x3D, 0)  # CONFIG; no reset of another device on the bus
            time.sleep(.025)
            self.write(0x07, 0)  # register page 0
            self.write(0x3E, 0)  # normal power
            self.write(0x3B, 0)  # m/s², deg/s, Android convention, Celsius
            self.write(0x3D, 0x0C)  # NDOF, accel + gyro + magnetometer fusion
            time.sleep(.02)
            if self.read(0x3D, 1)[0] != 0x0C:
                raise OSError('BNO055 did not enter NDOF mode')
        except Exception:
            os.close(self.fd)
            raise

    def transfer(self, messages):
        """Perform one atomic I2C transaction."""
        array = (I2CMessage * len(messages))(*messages)
        request = I2CTransfer(array, len(messages))
        fcntl.ioctl(self.fd, 0x0707, request)  # I2C_RDWR

    def read(self, register, count):
        """Read registers with a repeated start."""
        reg = (ctypes.c_uint8 * 1)(register)
        data = (ctypes.c_uint8 * count)()
        self.transfer([I2CMessage(0x28, 0, 1, reg), I2CMessage(0x28, 1, count, data)])
        return bytes(data)

    def write(self, register, value):
        """Write a single byte register."""
        data = (ctypes.c_uint8 * 2)(register, value)
        self.transfer([I2CMessage(0x28, 0, 2, data)])

    def close(self):
        """Release the device without resetting other sensors."""
        os.close(self.fd)


def decode_sample(data):
    """Decode 0x08..0x27 (SI accel/gyro/tesla, quaternion in x,y,z,w)."""
    values = struct.unpack('<16h', data)
    q = [values[i] / 16384. for i in (13, 14, 15, 12)]
    norm = math.sqrt(sum(v*v for v in q))
    if not .9 < norm < 1.1:
        raise ValueError('Invalid BNO055 quaternion')
    return ([v/100. for v in values[:3]],
            [math.radians(v/16.) for v in values[6:9]],
            [v/16.*1e-6 for v in values[3:6]], [v/norm for v in q])


class MagneticIMU(Node):
    """Publish native compatibility, SI IMU, raw magnetic field and quality."""

    def __init__(self):
        super().__init__('soccer_magnetic_imu')
        interface = self.declare_parameter('interface', '/dev/i2c-0').value
        rate = float(self.declare_parameter('rate', 100.).value)
        if not math.isfinite(rate) or not 1 <= rate <= 100:
            raise ValueError('BNO055 rate must be 1..100 Hz')
        self.frame = self.declare_parameter('frame_id', 'imu_link').value
        self.device = BNO055(interface)
        self.imu = self.create_publisher(Imu, 'imu_raw', qos_profile_sensor_data)
        self.native = self.create_publisher(Imu, 'imu_native', 10)
        self.mag = self.create_publisher(MagneticField, 'imu/mag', qos_profile_sensor_data)
        self.status = self.create_publisher(String, 'imu/status', qos_profile_sensor_data)
        self.timer = self.create_timer(1/rate, self.sample)

    def sample(self):
        """Drop failed samples; never publish stale or synthetic hardware readings."""
        try:
            accel, gyro, magnetic, q = decode_sample(self.device.read(0x08, 32))
            quality = self.device.read(0x35, 1)[0]
            mode = self.device.read(0x3D, 1)[0]
            if mode != 0x0C:
                raise OSError('BNO055 mode changed; check for a second IMU driver')
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.frame
            for vector, values in [(msg.linear_acceleration, accel), (msg.angular_velocity, gyro)]:
                vector.x, vector.y, vector.z = values
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            # Zero covariance means unknown; do not pretend these are measured variances.
            mag = MagneticField(header=deepcopy(msg.header))
            mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = magnetic
            native = deepcopy(msg)
            native.angular_velocity.x, native.angular_velocity.y, native.angular_velocity.z = [
                math.degrees(v) for v in gyro]
            self.imu.publish(msg)
            self.native.publish(native)
            self.mag.publish(mag)
            self.status.publish(String(data=json.dumps({
                'stamp': self.get_clock().now().nanoseconds/1e9,
                'source': 'bno055_ndof', 'mode': 'NDOF',
                'system': (quality >> 6) & 3, 'gyro': (quality >> 4) & 3,
                'accel': (quality >> 2) & 3, 'mag': quality & 3})))
        except (OSError, ValueError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.)

    def destroy_node(self):
        """Stop polling before releasing the bus descriptor."""
        self.timer.cancel()
        self.device.close()
        return super().destroy_node()


def main(args=None):
    """Run on the robot instead of the official IMU driver, never alongside it."""
    rclpy.init(args=args)
    node = None
    try:
        node = MagneticIMU()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
