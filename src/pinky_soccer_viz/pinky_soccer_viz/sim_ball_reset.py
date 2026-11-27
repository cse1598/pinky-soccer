"""Simulation-only ball placement after a referee exit and robot settling."""
import json
import math


class ResetGate:
    def __init__(self, robots):
        self.robots = robots
        self.status, self.odom = {}, {}
        self.sequence = -1
        self.pending = None
        self.settled_since = None

    def event(self, sequence, event, stamp):
        if sequence <= self.sequence:
            return
        self.sequence = sequence
        if event in ('GOAL', 'OUT', 'UNVERIFIED_EXIT'):
            self.pending = (event, stamp)
            self.settled_since = None

    def ready(self, now):
        if self.pending is None:
            return False
        event, stamp = self.pending
        allowed = ('READY',) if event == 'GOAL' else ('STOPPED', 'READY')
        for name in self.robots:
            status = self.status.get(name)
            odom = self.odom.get(name)
            if (not status or not odom or status[0] < stamp or odom[0] < stamp or
                    not 0 <= now-status[0] < .5 or not 0 <= now-odom[0] < .3 or
                    status[1] not in allowed or not status[2] or
                    not math.isfinite(odom[1]) or not math.isfinite(odom[2]) or
                    abs(odom[1]) >= .012 or abs(odom[2]) >= .08):
                self.settled_since = None
                return False
        if self.settled_since is None:
            self.settled_since = now
        return now-self.settled_since >= .4


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from pinky_soccer_referee.msg import RefereeEvent

    rclpy.init(args=args)
    node = Node('sim_ball_reset')
    if not node.get_parameter('use_sim_time').value:
        raise RuntimeError('sim_ball_reset requires use_sim_time; never run on hardware')
    gate = ResetGate(node.declare_parameter('robots', ['']).value)
    now = lambda: node.get_clock().now().nanoseconds*1e-9
    pub = node.create_publisher(Point, '/sim/set_ball', 1)
    def event(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if -.05 <= now()-stamp < 1.:
            gate.event(msg.sequence, msg.event, stamp)
    node.create_subscription(RefereeEvent, '/referee/events', event,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def status(msg, name):
        try:
            data = json.loads(msg.data)
            gate.status[name] = (float(data['stamp_ros']), data['state'],
                                 bool(data.get('localization_valid')))
        except (KeyError, TypeError, ValueError):
            gate.status.pop(name, None)
    def odom(msg, name):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        gate.odom[name] = (stamp, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
    for name in gate.robots:
        prefix = '/'+name+'/' if name else '/'
        node.create_subscription(String, prefix+'preparation/status',
            lambda m, n=name: status(m, n), 10)
        node.create_subscription(Odometry, prefix+'odom', lambda m, n=name: odom(m, n), 10)
    def tick():
        if gate.ready(now()):
            pub.publish(Point(x=0., y=0., z=0.))
            node.get_logger().info('Robots settled: ball placed at (0,0); waiting for START')
            gate.pending = None
    node.create_timer(.05, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
