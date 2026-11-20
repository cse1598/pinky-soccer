"""
Track /behavior_goal targets with a simple proportional controller -> /cmd_vel.

mode REACTIVE : drive to target position, then turn to target heading.
mode NAVIGATE : same proportional law for now (Nav2 integration is TODO).
mode STOP     : zero velocity.
Safety: zero velocity when /behavior_goal or /world_model is older than its timeout
(steady clock), independent of the robot-side command_watchdog.
The robot pose comes from /world_model (map frame), the same frame as the targets.
"""

from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import Twist
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
import rclpy
from rclpy.node import Node


@dataclass
class Gains:
    """Controller limits and gains (starting assumptions; tune in simulation)."""

    k_lin: float = 1.2          # [1/s] linear speed per metre of distance
    k_ang: float = 2.5          # [1/s] angular speed per radian of heading error
    max_lin: float = 0.25       # [m/s]
    max_ang: float = 2.0        # [rad/s]
    pos_tol: float = 0.03       # [m] position reached
    ang_tol: float = 0.08       # [rad] heading reached
    turn_in_place: float = 0.9  # [rad] heading error above which we only rotate


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, limit):
    """Clamp v to [-limit, limit]."""
    return max(-limit, min(limit, v))


def compute_command(pose, target, gains, max_speed=0.0):
    """
    Return (v, w) to move pose (x, y, th) towards target (x, y, th) in the same frame.

    max_speed > 0 lowers the linear limit (BehaviorGoal.max_speed).
    """
    v_max = min(gains.max_lin, max_speed) if max_speed > 0 else gains.max_lin
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    dist = math.hypot(dx, dy)
    if dist <= gains.pos_tol:
        err = wrap(target[2] - pose[2])
        if abs(err) <= gains.ang_tol:
            return 0.0, 0.0
        return 0.0, clamp(gains.k_ang * err, gains.max_ang)
    err = wrap(math.atan2(dy, dx) - pose[2])
    w = clamp(gains.k_ang * err, gains.max_ang)
    if abs(err) > gains.turn_in_place:
        return 0.0, w
    v = clamp(gains.k_lin * dist, v_max) * math.cos(err) ** 2
    return max(0.0, v), w


class ReactiveController(Node):
    """BehaviorGoal follower."""

    def __init__(self):
        super().__init__('reactive_controller')
        p = self.declare_parameter
        defaults = Gains()
        self.gains = Gains(**{k: p(k, float(getattr(defaults, k))).value
                              for k in Gains.__dataclass_fields__})
        self.goal_timeout = p('goal_timeout', 0.5).value
        self.pose_timeout = p('pose_timeout', 0.5).value
        rate = p('rate_hz', 20.0).value
        self.goal = self.goal_time = None
        self.pose = self.pose_time = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(BehaviorGoal, 'behavior_goal', self.on_goal, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_goal(self, msg):
        """Store the latest goal with its arrival time."""
        self.goal, self.goal_time = msg, time.monotonic()

    def on_world(self, msg):
        """Store the latest self pose with its arrival time."""
        s = msg.self.pose
        self.pose, self.pose_time = (s.x, s.y, s.theta), time.monotonic()

    def tick(self):
        """Publish the command for the current goal, or zero if anything is stale."""
        now = time.monotonic()
        cmd = Twist()
        fresh = (self.goal is not None and now - self.goal_time <= self.goal_timeout
                 and self.pose is not None and now - self.pose_time <= self.pose_timeout)
        if fresh and self.goal.mode != BehaviorGoal.STOP:
            # TODO(nav2): NAVIGATE should send far targets to Nav2; proportional for now.
            t = self.goal.target
            v, w = compute_command(self.pose, (t.x, t.y, t.theta), self.gains,
                                   float(self.goal.max_speed))
            if self.goal.action == 'SpinInPlace' and self.goal.max_speed > 0:
                w = clamp(w, float(self.goal.max_speed))    # max_speed is a turn rate here
                v = 0.0
            cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)


def main(args=None):
    """Run the controller."""
    rclpy.init(args=args)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.pub.publish(Twist())   # best effort; the command_watchdog also stops
        except Exception:  # noqa: B902 - context may already be shut down by a signal
            pass
        node.destroy_node()
        rclpy.try_shutdown()
