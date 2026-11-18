#!/usr/bin/env python3
"""
Republish wheel joint states under the URDF joint names.

The official pinky_bringup driver publishes left_wheel_joint / right_wheel_joint, while
pinky_description (and Gazebo / MuJoCo) name the joints l_wheel_joint / r_wheel_joint.
This node renames them so robot_state_publisher can move the wheels, without changing
the official driver.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def rename_joints(names, from_names, to_names):
    """Return names with every entry of from_names replaced by the matching to_names."""
    if len(from_names) != len(to_names):
        raise ValueError('from_names and to_names must have the same length')
    mapping = dict(zip(from_names, to_names))
    return [mapping.get(name, name) for name in names]


class WheelJointStateRelay(Node):

    def __init__(self):
        super().__init__('wheel_joint_state_relay')
        self.from_names = list(self.declare_parameter(
            'from_names', ['left_wheel_joint', 'right_wheel_joint']).value)
        self.to_names = list(self.declare_parameter(
            'to_names', ['l_wheel_joint', 'r_wheel_joint']).value)
        rename_joints([], self.from_names, self.to_names)  # validate once
        self.pub = self.create_publisher(JointState, 'wheel_joint_states', 10)
        self.create_subscription(JointState, 'wheel_joint_states_raw', self.on_msg, 10)

    def on_msg(self, msg):
        msg.name = rename_joints(msg.name, self.from_names, self.to_names)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
