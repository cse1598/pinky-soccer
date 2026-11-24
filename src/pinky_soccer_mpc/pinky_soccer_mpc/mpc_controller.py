"""
NMPC BehaviorGoal follower: /behavior_goal + /world_model -> /cmd_vel (replaces the P control).

Every cycle it solves PoseMpc from the current pose (/world_model self, map frame) and an
estimate of the wheel speeds, sends the first wheel-speed target through the CommandAdapter
(the driver's own radius/separation, so calibration is applied once) and publishes the
predicted trajectory on /mpc/predicted_path.

Wheel speed state: with wheel_feedback:=true the measured /joint_states velocities (with the
target's joint_sign) are used while fresh; otherwise they are propagated from the commands
actually sent with the wheel response model (first-order lag + acceleration limit). The real
robot defaults to the model because its right wheel sign is still unverified
(pinky_soccer_dynamics README); an open-loop estimate makes the MPC overcorrect when the
motors respond slower than the model (weaving), so enable feedback once the sign is checked.

Safety (same as reactive_controller, plus): zero command when /behavior_goal or /world_model
is stale, on STOP, or when the solver fails twice in a row; the robot-side command_watchdog
stays in place.
"""

import json
import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from pinky_soccer_dynamics import params as dyn_params
from pinky_soccer_dynamics.command_adapter import CommandAdapter, wheels_from_joint_state
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .pose_mpc import PoseMpc, step_wheels


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Arrival:
    """Goal reached with hysteresis: hold still until the goal moves or the pose drifts."""

    def __init__(self, pos_tol=0.02, ang_tol=0.05):
        self.pos_tol, self.ang_tol = pos_tol, ang_tol
        self.holding = None

    def update(self, pose, goal, wheel_speed):
        """Return True while the robot should hold still at goal."""
        dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        err = abs(wrap(goal[2] - pose[2]))
        if self.holding is not None:
            moved = math.hypot(goal[0] - self.holding[0], goal[1] - self.holding[1]) > 0.01 \
                or abs(wrap(goal[2] - self.holding[2])) > 0.03
            if moved or dist > 2 * self.pos_tol or err > 2 * self.ang_tol:
                self.holding = None
        elif dist <= self.pos_tol and err <= self.ang_tol and wheel_speed < 1.0:
            self.holding = tuple(goal)
        return self.holding is not None


class MpcController(Node):
    """BehaviorGoal follower with the point-to-pose NMPC."""

    def __init__(self):
        super().__init__('mpc_controller')
        p = self.declare_parameter
        target = p('target', 'real').value
        profile = p('calibration_profile', '').value
        self.params = dyn_params.load(target, profile=profile or None)
        self.max_lin = float(p('max_lin', 0.0).value)      # 0: only the wheel limits
        self.max_ang = float(p('max_ang', 0.0).value)
        self.goal_timeout = p('goal_timeout', 0.5).value
        self.pose_timeout = p('pose_timeout', 0.5).value
        rate = p('rate_hz', 20.0).value
        self.mpc = PoseMpc(self.params, p('horizon', 1.5).value, int(p('steps', 30).value))
        self.adapter = CommandAdapter(self.params)
        self.arrival = Arrival(p('pos_tol', 0.02).value, p('ang_tol', 0.05).value)
        # Ball-related actions keep the camera (front) towards the ball: no reverse driving.
        self.forward_only = set(p('forward_only_actions', [
            'GoBehindBall', 'PushBall', 'GoToBlockPoint', 'FaceBall', 'TurnToLastSeen']).value)
        self.dt = 1.0 / rate
        self.goal = self.goal_time = None
        self.pose = self.pose_time = None
        self.omega = (0.0, 0.0)
        self.failures = 0
        self.last_goal = None
        self.measured = self.measured_time = None
        if p('wheel_feedback', False).value:
            self.create_subscription(JointState, 'joint_states', self.on_joints, 20)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'mpc/predicted_path', 10)
        self.status_pub = self.create_publisher(String, 'mpc/status', 10)
        self.create_subscription(BehaviorGoal, 'behavior_goal', self.on_goal, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'NMPC {target}: wheel <= {self.mpc.wmax:.2f} rad/s '
            f'({self.mpc.wmax * 30 / math.pi:.1f} rpm), accel <= {self.mpc.amax:.1f} rad/s^2, '
            f'v <= {self.mpc.v_limit:.3f} m/s, '
            f'w <= {self.mpc.w_limit:.2f} rad/s; caps v {self.max_lin or "-"} w '
            f'{self.max_ang or "-"}; profile {profile or "none"}')

    def on_goal(self, msg):
        """Store the latest goal."""
        self.goal, self.goal_time = msg, time.monotonic()

    def on_joints(self, msg):
        """Store measured wheel speeds in the model convention."""
        try:
            self.measured = wheels_from_joint_state(self.params, msg.name, msg.velocity)
            self.measured_time = time.monotonic()
        except (KeyError, ValueError):
            pass

    def on_world(self, msg):
        """Store the latest self pose (map frame)."""
        s = msg.self.pose
        self.pose, self.pose_time = (s.x, s.y, s.theta), time.monotonic()

    def caps(self, goal):
        """Linear/angular speed caps for this goal (0 = none)."""
        v_cap, w_cap = self.max_lin, self.max_ang
        if goal.max_speed > 0:
            if goal.action == 'SpinInPlace':       # max_speed is a turn rate there
                w_cap = min(w_cap, goal.max_speed) if w_cap else float(goal.max_speed)
            else:
                v_cap = min(v_cap, goal.max_speed) if v_cap else float(goal.max_speed)
        return v_cap, w_cap

    def stop(self, reason):
        """Publish zero and reset the predictor."""
        self.pub.publish(Twist())
        self.omega = step_wheels(self.params, self.omega, (0.0, 0.0), self.dt, self.mpc.amax)
        self.mpc.reset()
        self.publish_status({'state': 'stop', 'reason': reason})

    def tick(self):
        """Solve and publish one command."""
        now = time.monotonic()
        fresh = (self.goal is not None and now - self.goal_time <= self.goal_timeout
                 and self.pose is not None and now - self.pose_time <= self.pose_timeout)
        if not fresh:
            return self.stop('stale input')
        if self.goal.mode == BehaviorGoal.STOP:
            return self.stop('STOP goal')
        t = self.goal.target
        goal = (t.x, t.y, t.theta)
        if self.last_goal is None or math.hypot(goal[0] - self.last_goal[0],
                                                goal[1] - self.last_goal[1]) > 0.3:
            self.mpc.reset()                       # big jump: drop the warm start
        self.last_goal = goal
        if self.arrival.update(self.pose, goal, abs(self.omega[0]) + abs(self.omega[1])):
            return self.stop('arrived')
        v_cap, w_cap = self.caps(self.goal)
        if self.measured is not None and now - self.measured_time < 0.2:
            self.omega = self.measured
        x0 = (*self.pose, *self.omega)
        res = self.mpc.solve(x0, goal, v_cap, w_cap,
                             forward_only=self.goal.action in self.forward_only)
        if not res['ok']:
            self.failures += 1
            self.get_logger().warn(f'NMPC status {res["status"]} ({self.failures} in a row)',
                                   throttle_duration_sec=1.0)
            if self.failures >= 2:
                return self.stop(f'solver status {res["status"]}')
        else:
            self.failures = 0
        cmd = res['u']
        v, w = self.adapter.to_twist(*cmd)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)
        self.omega = step_wheels(self.params, self.omega, cmd, self.dt, self.mpc.amax)
        self.publish_path(res['xs'])
        self.publish_status({'state': 'track', 'status': res['status'], 'ms': round(res['ms'], 1),
                             'v': round(v, 3), 'w': round(w, 2),
                             'v_bound': round(res['v_bound'], 3),
                             'w_bound': round(res['w_bound'], 2)})

    def publish_path(self, xs):
        """Publish the predicted trajectory (map frame)."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for x in xs:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x[0]), float(x[1])
            ps.pose.orientation.z, ps.pose.orientation.w = (math.sin(x[2] / 2),
                                                            math.cos(x[2] / 2))
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_status(self, data):
        """Publish a small JSON diagnostics message."""
        self.status_pub.publish(String(data=json.dumps(data)))


def main(args=None):
    """Run the controller."""
    rclpy.init(args=args)
    node = MpcController()
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
