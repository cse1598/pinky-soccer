"""
ROS node: run the striker playbook (or one skill) on the robot or in a full simulation.

Inputs  /world_model (self pose, ball estimate, opponents, playing flag; map frame)
        /odom (measured twist), /us_sensor/range (optional)
Output  /cmd_vel (Twist) -> command_watchdog -> drive/cmd_vel, and /skills/status (JSON)

Fills the same SkillContext the MuJoCo bench uses, so skills run unchanged. Stops (zero
command) when not playing or when /world_model is stale. No ground truth is read.
"""

import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String

from . import params as params_mod
from .context import BallEstimate, ESTIMATED, FieldInfo, LOST, OBSERVED, RUNNING, SkillContext
from .geometry import Pose
from .motion import Limiter
from .playbook import BEHAVIORS
from .skills import SKILLS

STATE = {0: OBSERVED, 1: ESTIMATED, 2: LOST}


def stamp_sec(t):
    """builtin_interfaces/Time -> seconds."""
    return t.sec + t.nanosec * 1e-9


class SkillPlayer(Node):
    """Playbook runner."""

    def __init__(self):
        super().__init__('skill_player')
        p = self.declare_parameter
        params_file = p('params_file', '').value
        self.rate = p('rate_hz', 30.0).value
        self.behavior = p('behavior', 'striker').value
        self.skill_name = p('skill', '').value              # run one skill instead
        self.skill_args = json.loads(p('skill_args', '{}').value or '{}')
        field_spec = p('field_spec', '').value
        self.our_color = p('our_goal_color', 'yellow').value
        self.wm_timeout = p('world_model_timeout', 0.5).value
        overrides = {'robot': {}}
        for key in ('max_lin', 'max_ang'):
            value = p(key, 0.0).value
            if value > 0.0:
                overrides['robot'][key] = value
        self.P = params_mod.load(params_file or None, overrides)
        from pinky_soccer_field.field_geometry import Field
        self.field = FieldInfo.from_field(Field.from_file(field_spec or None, self.our_color))
        self.limiter = Limiter(self.P.robot, 1.0 / self.rate)
        self.task = None
        self.wm = None
        self.wm_time = None
        self.last_seen = None
        self.odom = (0.0, 0.0)
        self.us = math.nan
        from pinky_soccer_msgs.msg import WorldModel
        self.create_subscription(WorldModel, '/world_model', self.on_wm, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Range, '/us_sensor/range', self.on_us, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/skills/status', 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"skill_player: {'skill ' + self.skill_name if self.skill_name else self.behavior}"
            f', limits {self.P.robot.max_lin:.2f} m/s {self.P.robot.max_ang:.2f} rad/s')

    # ------------------------------------------------------------------ inputs
    def on_wm(self, msg):
        """Latest world model."""
        self.wm = msg
        self.wm_time = time.monotonic()

    def on_odom(self, msg):
        """Measured twist."""
        self.odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_us(self, msg):
        """Ultrasonic range (keeps -inf/inf as published)."""
        self.us = msg.range

    def build_context(self, now):
        """Build the SkillContext from the latest messages (None when not usable)."""
        wm = self.wm
        if wm is None or time.monotonic() - self.wm_time > self.wm_timeout:
            return None
        s = wm.self.pose
        ctx = SkillContext(t=now, pose=Pose(s.x, s.y, s.theta), v=self.odom[0],
                           w=self.odom[1], us_range=self.us, field=self.field,
                           last_cmd=(self.limiter.v, self.limiter.w))
        b = wm.ball
        if b.last_seen.sec or b.last_seen.nanosec:
            seen_t = stamp_sec(b.last_seen)
            seen_now = self.last_seen is not None and seen_t > self.last_seen
            self.last_seen = seen_t
            c = list(b.covariance)
            sigma = math.sqrt(max(c[0], c[4], 0.0))
            ctx.ball = BallEstimate(b.pose.x, b.pose.y, b.velocity.x, b.velocity.y,
                                    STATE.get(b.state, LOST),
                                    max(0.0, stamp_sec(wm.header.stamp) - seen_t), sigma,
                                    seen_now)
        ctx.opponents = [(o.pose.x, o.pose.y) for o in wm.opponents if o.state != 2]
        return ctx

    # ------------------------------------------------------------------ loop
    def new_task(self):
        """Fresh playbook or skill."""
        if self.skill_name:
            return SKILLS[self.skill_name](self.P, **self.skill_args)
        return BEHAVIORS[self.behavior](self.P)

    def tick(self):
        """Control cycle."""
        now = self.get_clock().now().nanoseconds * 1e-9
        ctx = self.build_context(now)
        status = {'t': round(now, 3)}
        if ctx is None or not self.wm.playing:
            self.task = None
            self.limiter.reset()
            self.publish(0.0, 0.0)
            status['state'] = 'no_world_model' if ctx is None else 'paused'
            self.status_pub.publish(String(data=json.dumps(status)))
            return
        if self.task is None:
            self.task = self.new_task()
        out = self.task.step(ctx)
        v, w = self.limiter(out.v, out.w)
        if out.status != RUNNING and self.skill_name:
            v, w = 0.0, 0.0
            self.limiter.reset()
        self.publish(v, w)
        status.update(state=out.status, reason=out.reason, phase=out.phase,
                      skill=out.info.get('skill', getattr(self.task, 'name', '')),
                      mode=out.info.get('mode', ''), v=round(v, 3), w=round(w, 3))
        self.status_pub.publish(String(data=json.dumps(status)))

    def publish(self, v, w):
        """Send a velocity command."""
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = SkillPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
