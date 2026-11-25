#!/usr/bin/env python3
"""Official Pinky motor driver + read-only motor diagnostics (opt-in: motor_diagnostics:=true).

Runs the UNMODIFIED official pinky_bringup.bringup.Pinky node in this process with a driver
subclass, so only one program owns /dev/ttyAMA4. Differences from the official node:
  * the periodic feedback bulk read covers the contiguous X-series control-table block
    124..145 (Present PWM, Present Current, Present Velocity, Present Position,
    Velocity Trajectory, Position Trajectory, Present Input Voltage) instead of 128..135;
    velocity/position are decoded exactly like the official driver (same odom/joints)
  * model number and Profile Acceleration are read once at start (motor/info, latched)
  * every received drive command is timestamped on receipt and after the serial write;
    Twist.linear.z (ignored by the driver) is reported as the command tag the calibration
    UI uses to match PC commands to applied commands
  * optional feedback rate parameter feedback_rate_hz (default 30 = official)
Published on motor/diagnostics (std_msgs/String JSON) after every feedback cycle.
Addresses follow the ROBOTIS X-series (XL330) control table; the model number is reported
so a different motor can be recognised. Nothing is written except what the official node
writes.
"""
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

import pinky_bringup.bringup as official
from dynamixel_sdk import COMM_SUCCESS, GroupBulkRead

BLOCK_ADDR, BLOCK_LEN = 124, 22           # 124 PWM(2) .. 144 input voltage(2)
FIELDS = (('present_pwm', 124, 2), ('present_current', 126, 2), ('present_velocity', 128, 4),
          ('present_position', 132, 4), ('velocity_trajectory', 136, 4),
          ('position_trajectory', 140, 4), ('present_input_voltage', 144, 2))
MODEL_NAMES = {1200: 'XL330-M288', 1190: 'XL330-M077', 1060: 'XL430-W250', 1020: 'XM430-W350'}


def signed(value, size):
    bits = 8 * size
    return value - (1 << bits) if value >= 1 << (bits - 1) else value


class DiagnosticDriver(official.DynamixelDriver):
    """Same commands; wider read-only feedback block."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.diag_read = GroupBulkRead(self.portHandler, self.packetHandler)
        self.last_diag = None
        self.diag_failures = 0

    def get_feedback(self):
        self.diag_read.clearParam()
        for dxl_id in self.DXL_IDS:
            self.diag_read.addParam(dxl_id, BLOCK_ADDR, BLOCK_LEN)
        read_ns = time.time_ns()
        if self.diag_read.txRxPacket() != COMM_SUCCESS or not all(
                self.diag_read.isAvailable(i, BLOCK_ADDR, BLOCK_LEN) for i in self.DXL_IDS):
            self.diag_failures += 1
            self.last_diag = None
            return super().get_feedback()             # official read path as fallback
        done_ns = time.time_ns()
        motors = []
        for dxl_id in self.DXL_IDS:
            row = {'id': dxl_id}
            for name, addr, size in FIELDS:
                row[name] = signed(self.diag_read.getData(dxl_id, addr, size), size)
            motors.append(row)
        self.last_diag = {'read_start_unix_ns': read_ns, 'read_done_unix_ns': done_ns,
                          'motors': motors}
        left, right = motors
        rpm_l = left['present_velocity'] / self.RPM_TO_VALUE_SCALE
        rpm_r = right['present_velocity'] / self.RPM_TO_VALUE_SCALE
        return rpm_l, rpm_r, left['present_position'], right['present_position']


class DiagnosticPinky(official.Pinky):
    def __init__(self):
        official.DynamixelDriver = DiagnosticDriver     # used by Pinky.__init__ (this process)
        super().__init__()
        self.diag_pub = self.create_publisher(String, 'motor/diagnostics', 10)
        self.info_pub = self.create_publisher(
            String, 'motor/info', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.commands = []
        rate = float(self.declare_parameter('feedback_rate_hz', 30.0).value)
        if not self.is_initialized:
            return
        if not 5.0 <= rate <= 100.0:
            raise ValueError('feedback_rate_hz must be 5..100')
        if rate != 30.0:
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / rate, self.update_and_publish)
        info = {'feedback_rate_hz': rate, 'block': [BLOCK_ADDR, BLOCK_LEN], 'motors': []}
        for dxl_id in self.driver.DXL_IDS:
            model, res, err = self.driver.packetHandler.read2ByteTxRx(self.driver.portHandler,
                                                                     dxl_id, 0)
            accel, res2, _ = self.driver.packetHandler.read4ByteTxRx(
                self.driver.portHandler, dxl_id, self.driver.ADDR_PROFILE_ACCEL)
            info['motors'].append({'id': dxl_id, 'model_number': model if res == COMM_SUCCESS else None,
                                   'model': MODEL_NAMES.get(model, 'unknown'),
                                   'profile_acceleration': accel if res2 == COMM_SUCCESS else None})
        self.info_pub.publish(String(data=json.dumps(info)))

    def twist_callback(self, msg):
        recv = time.time_ns()
        super().twist_callback(msg)
        self.commands.append({'tag': msg.linear.z, 'v': msg.linear.x, 'w': msg.angular.z,
                              'recv_unix_ns': recv, 'written_unix_ns': time.time_ns()})

    def update_and_publish(self):
        super().update_and_publish()
        out = {'stamp_unix_ns': time.time_ns(), 'feedback': self.driver.last_diag,
               'diag_read_failures': self.driver.diag_failures, 'commands': self.commands}
        self.commands = []
        self.diag_pub.publish(String(data=json.dumps(out)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DiagnosticPinky()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.driver.terminate()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
