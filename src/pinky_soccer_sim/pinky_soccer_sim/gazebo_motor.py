#!/usr/bin/env python3
"""XL330 profile and lag surrogate before Gazebo velocity-controlled DiffDrive.

Gazebo retains its chassis/contact physics. This adapter is not a torque servo.
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pinky_soccer_dynamics import params
from pinky_soccer_dynamics.command_adapter import CommandAdapter
from pinky_soccer_dynamics.profile_model import motor_step


class GazeboMotor(Node):
    def __init__(self):
        super().__init__('gazebo_motor')
        profile = self.declare_parameter('calibration_profile','').value
        self.params = params.load('gazebo',profile=profile or None)
        self.adapter = CommandAdapter(self.params)
        self.target = self.ref = self.wheel = np.zeros(2)
        self.last = -math.inf
        self.sim_time = None
        self.pub = self.create_publisher(Twist,'drive/motor_cmd_vel',10)
        self.create_subscription(Twist,'drive/cmd_vel',self.receive,10)
        self.create_timer(.01,self.tick)

    def receive(self,msg):
        v,w = msg.linear.x,msg.angular.z
        if math.isfinite(v) and math.isfinite(w):
            l,r,_ = self.adapter.driver_applied(v,w)
            self.target = np.array([l,r])
            self.last = time.monotonic()

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        dt = .01 if self.sim_time is None else now-self.sim_time
        self.sim_time = now
        if dt < 0 or dt > .5:
            self.ref = self.wheel = np.zeros(2)
            dt = 0.
        target = self.target if time.monotonic()-self.last < .5 else np.zeros(2)
        self.wheel,self.ref = motor_step(self.params,self.wheel,self.ref,target,dt)
        v,w = self.adapter.to_twist(*self.wheel)
        msg = Twist()
        msg.linear.x,msg.angular.z = float(v),float(w)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GazeboMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
