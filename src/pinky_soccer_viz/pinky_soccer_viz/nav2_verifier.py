"""Native Nav2 adapter for GoToPose verification (simulation only).
Nav2's action owns planning/control; this node owns command gating and MotionResult.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from std_msgs.msg import String
from pinky_soccer_msgs.msg import BehaviorGoal, MotionResult, WorldModel
import json


def limit_twist(v, w, vcap, wcap, radius, track, wheel_max):
    """Scale both axes together to respect chassis and physical wheel limits."""
    if not all(math.isfinite(x) for x in (v, w)):
        return 0., 0.
    peak = max(abs(v-track*w/2), abs(v+track*w/2))/radius
    scale = min(1., vcap/max(abs(v), 1e-12), wcap/max(abs(w), 1e-12),
                wheel_max/max(peak, 1e-12))
    return v*scale, w*scale


class Nav2Verifier(Node):
    def __init__(self):
        super().__init__('nav2_verifier')
        p = self.declare_parameter
        self.vcap = float(p('max_lin', .25).value)
        self.wcap = float(p('max_ang', 2.).value)
        self.radius = float(p('wheel_radius', .028).value)
        self.track = float(p('wheel_separation', .0971).value)
        self.wheel_max = float(p('wheel_max', .9*103*2*math.pi/60).value)
        self.goal = self.world = None
        self.goal_at = self.world_at = 0.
        self.key = None
        self.generation = 0
        self.pending = self.handle = None
        self.canceling = False
        self.result = None
        self.cmd_at = 0.
        self.cmd = (0., 0.)
        self.started = 0.
        self.settle_since = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.results = self.create_publisher(MotionResult, 'motion/result', 10)
        self.path = self.create_publisher(Path, 'mpc/predicted_path', 10)
        self.status = self.create_publisher(String, 'mpc/status', 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(BehaviorGoal, 'behavior_goal', self.on_goal, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_subscription(Twist, 'nav2_verify/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Path, 'plan', self.on_path, 10)
        self.create_timer(.02, self.tick)

    def on_goal(self, msg):
        self.goal, self.goal_at = msg, time.monotonic()

    def on_world(self, msg):
        self.world, self.world_at = msg, time.monotonic()

    def active(self):
        return (self.goal is not None and self.goal.mode != BehaviorGoal.STOP
                and self.goal.action == 'GoToPose'
                and time.monotonic()-self.goal_at < .5
                and self.world is not None and time.monotonic()-self.world_at < .5
                and self.world.self.state != 2)

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_at = time.monotonic()

    def on_path(self, msg):
        if self.active() and self.handle is not None and not self.canceling:
            self.path.publish(msg)

    def send(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def cancel(self):
        self.generation += 1
        self.cmd_at = 0.
        self.cmd = (0., 0.)
        self.settle_since = None
        self.send()
        if self.handle is not None and not self.canceling:
            self.canceling = True
            self.handle.cancel_goal_async()
        # A pending send is retained until its response; stale accepted handles are cancelled.
        self.key = None
        self.result = None

    def on_accept(self, future, generation):
        self.pending = None
        try:
            handle = future.result()
        except Exception as exc:
            if generation == self.generation:
                self.result = (False, str(exc))
            return
        if not handle.accepted:
            if generation == self.generation:
                self.result = (False, 'Nav2 rejected goal')
            return
        self.handle = handle
        self.cmd_at = 0.
        self.canceling = generation != self.generation or not self.active()
        if self.canceling:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self.on_result(f, generation))

    def on_result(self, future, generation):
        self.handle = None
        self.canceling = False
        self.cmd_at = 0.
        self.send()
        if generation != self.generation:
            return
        try:
            result = future.result()
            ok = result.status == GoalStatus.STATUS_SUCCEEDED
            self.result = (ok, 'Nav2 succeeded' if ok else f'Nav2 status {result.status}')
        except Exception as exc:
            self.result = (False, str(exc))

    def start(self, key):
        self.key = key
        self.started = time.monotonic()
        self.result = None
        self.settle_since = None
        self.cmd_at = 0.

    def dispatch(self):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        t = self.goal.target
        g.pose.pose.position.x, g.pose.pose.position.y = t.x, t.y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(t.theta/2), math.cos(t.theta/2)
        generation = self.generation
        self.pending = self.client.send_goal_async(g)
        self.pending.add_done_callback(lambda f: self.on_accept(f, generation))

    def report(self, state, reason):
        t, p = self.goal.target, self.world.self.pose
        dist = math.hypot(t.x-p.x, t.y-p.y)
        ang = abs(math.atan2(math.sin(t.theta-p.theta), math.cos(t.theta-p.theta)))
        m = MotionResult()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.goal_id, m.state, m.reason = self.goal.goal_id, state, reason
        m.position_error, m.heading_error = dist, ang
        self.results.publish(m)
        s = String()
        s.data = json.dumps(dict(backend='nav2_mppi', state=state, message=reason,
                                 dist=dist, ang=ang, goal_id=int(m.goal_id)))
        self.status.publish(s)

    def tick(self):
        now = time.monotonic()
        if not self.active():
            if self.key is not None:
                self.cancel()
            self.send()
            if (self.goal is not None and self.world is not None
                    and self.goal.mode != BehaviorGoal.STOP
                    and self.goal.action != 'GoToPose'
                    and now-self.goal_at < .5):
                self.report(MotionResult.FAILURE, 'Nav2 comparison supports GoToPose only')
            return
        t = self.goal.target
        if not all(math.isfinite(x) for x in (t.x, t.y, t.theta, self.goal.max_speed)):
            self.cancel()
            return
        key = (self.goal.goal_id, t.x, t.y, t.theta)
        if self.key != key:
            if self.key is not None:
                self.cancel()
            if self.pending is not None or self.handle is not None:
                self.send()
                return
            self.start(key)
        if self.result is not None:
            self.send()
            ok, reason = self.result
            if ok:
                p = self.world.self.pose
                dist = math.hypot(t.x-p.x, t.y-p.y)
                ang = abs(math.atan2(math.sin(t.theta-p.theta), math.cos(t.theta-p.theta)))
                if dist > .02 or ang > math.radians(3):
                    self.result = (False, 'Nav2 result outside common 2 cm / 3 deg tolerance')
                    self.report(MotionResult.FAILURE, self.result[1])
                    return
                # Compare the same 0.3 s pose settling interval, not just the Nav2 action result.
                sim_now = self.get_clock().now().nanoseconds/1e9
                if self.settle_since is None:
                    self.settle_since = sim_now
                if sim_now-self.settle_since < .3:
                    self.report(MotionResult.RUNNING, 'settling')
                    return
            self.report(MotionResult.SUCCESS if ok else MotionResult.FAILURE, reason)
            return
        if self.handle is None and self.pending is None:
            if self.client.server_is_ready():
                self.dispatch()
            elif now-self.started > 10.:
                self.result = (False, 'Nav2 action server not ready')
        if self.handle is None or self.canceling or now-self.cmd_at > .25:
            self.send()
        else:
            cap = min(self.vcap, self.goal.max_speed) if self.goal.max_speed > 0 else self.vcap
            self.send(*limit_twist(*self.cmd, cap, self.wcap, self.radius, self.track, self.wheel_max))
        self.report(MotionResult.RUNNING, 'Nav2 NavigateToPose')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Verifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cancel()
            node.send()
        node.destroy_node()
        rclpy.try_shutdown()
