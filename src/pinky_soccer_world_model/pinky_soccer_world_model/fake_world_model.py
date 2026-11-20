"""Fake /world_model for testing the behaviour tree and UIs without a robot or simulator.

Scenario (period `cycle_sec`, default 12 s):
  * ball rolls slowly on an ellipse inside the field (the true path),
  * ball state cycles OBSERVED (first third) -> ESTIMATED (sigma grows from the last fix,
    pose extrapolated) -> LOST (last estimate kept),
  * one opponent patrols the opponent half with its own OBSERVED/ESTIMATED/LOST cycle,
  * self follows /cmd_vel (unicycle integration) when commands arrive, else drives a
    slow circle. The search grid uses the fake camera field of view.
The true ball is also published on /sim/ball_ground_truth (evaluation overlay only).
"""

import math

from geometry_msgs.msg import PoseStamped, Twist
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import TrackedObject, WorldModel
from pinky_soccer_world_model.geometry import wrap
from pinky_soccer_world_model.msgs import tracked, world_model
from pinky_soccer_world_model.search_grid import SearchGrid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class FakeWorldModel(Node):
    """Scripted world for UI and behaviour tree tests."""

    def __init__(self):
        super().__init__('fake_world_model')
        p = self.declare_parameter
        self.field = Field.from_file(p('field_spec', '').value or None,
                                     p('our_goal_color', 'yellow').value)
        self.rate = p('rate_hz', 30.0).value
        self.cycle = p('cycle_sec', 12.0).value
        self.follow = p('follow_cmd_vel', True).value
        self.playing = p('playing', True).value
        self.hfov = math.radians(p('camera_hfov_deg', 60.0).value)
        self.max_range = p('camera_max_range', 1.5).value
        walls = self.field.inner_walls
        self.bounds = walls
        self.grid = SearchGrid(walls)
        start = p('initial_pose', [-0.5, 0.0, 0.0]).value
        self.pose = list(start)
        self.cmd, self.cmd_time = (0.0, 0.0), -math.inf
        self.t = 0.0
        self.t0 = self.get_clock().now().nanoseconds * 1e-9   # scenario t -> ROS time
        self.last_fix = None   # (t, x, y, vx, vy) of the last OBSERVED ball
        self.pub = self.create_publisher(WorldModel, 'world_model', 10)
        self.truth = self.create_publisher(PoseStamped, 'sim/ball_ground_truth', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, 'game/playing', self.on_playing, 10)
        self.create_timer(1.0 / self.rate, self.tick)

    def on_cmd(self, msg):
        """Remember the latest velocity command."""
        self.cmd, self.cmd_time = (msg.linear.x, msg.angular.z), self.t

    def on_playing(self, msg):
        """Allow /game/playing to override the default."""
        self.playing = msg.data

    def true_ball(self, t):
        """Return the scripted ball position and velocity at time t."""
        a, b, w = 0.7, 0.45, 2 * math.pi / (3 * self.cycle)
        return (a * math.cos(w * t), b * math.sin(w * t),
                -a * w * math.sin(w * t), b * w * math.cos(w * t))

    def tick(self):
        """Advance the scenario by one step and publish."""
        dt = 1.0 / self.rate
        self.t += dt
        t = self.t
        if self.follow and t - self.cmd_time < 0.5:
            v, w = self.cmd
        else:
            v, w = 0.1, 0.25
        x, y, th = self.pose
        x, y = self.bounds.clamp(x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, 0.06)
        self.pose = [x, y, wrap(th + w * dt)]

        bx, by, bvx, bvy = self.true_ball(t)
        phase = (t % self.cycle) / self.cycle
        if phase < 1 / 3:
            state, sigma = TrackedObject.OBSERVED, 0.02
            self.last_fix = (t, bx, by, bvx, bvy)
            est = (bx, by, bvx, bvy)
        else:
            t0, x0, y0, vx0, vy0 = self.last_fix or (t, bx, by, 0.0, 0.0)
            age = t - t0
            decay = math.exp(-age / 1.5)
            est = (x0 + vx0 * 1.5 * (1 - decay), y0 + vy0 * 1.5 * (1 - decay),
                   vx0 * decay, vy0 * decay)
            sigma = 0.02 + 0.08 * age
            state = TrackedObject.ESTIMATED if phase < 2 / 3 else TrackedObject.LOST
        cov = [sigma ** 2, 0, 0, 0, (0.7 * sigma) ** 2, 0, 0, 0, 1e6]
        last_seen = self.t0 + (self.last_fix[0] if self.last_fix else 0.0)
        ball = tracked('ball', (est[0], est[1], 0.0), est[2:], cov, state, last_seen,
                       {0: 1.0, 1: 0.5, 2: 0.0}[state])

        s = self.field.attack_sign
        ox, oy = s * (0.6 + 0.2 * math.cos(0.5 * t)), 0.4 * math.sin(0.35 * t)
        opp_phase = ((t + self.cycle / 2) % self.cycle) / self.cycle
        opp_state = (TrackedObject.OBSERVED if opp_phase < 0.5 else
                     TrackedObject.ESTIMATED if opp_phase < 0.8 else TrackedObject.LOST)
        opponents = [] if opp_state == TrackedObject.LOST else [tracked(
            'opp_0', (ox, oy, math.pi if s > 0 else 0.0), cov=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 1],
            state=opp_state,
            last_seen=self.t0 + (t if opp_state == TrackedObject.OBSERVED else t - 1.0),
            confidence=1.0 if opp_state == TrackedObject.OBSERVED else 0.5)]

        self.grid.relax(dt)
        mask = self.grid.fov_mask(x, y, self.pose[2], self.hfov, 0.15, self.max_range)
        self.grid.observe(mask, dt, (bx, by) if state == TrackedObject.OBSERVED else None)

        me = tracked('self', self.pose, (v * math.cos(th), v * math.sin(th)),
                     [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-3], TrackedObject.OBSERVED, self.t0 + t,
                     1.0)
        stamp = self.get_clock().now().to_msg()
        self.pub.publish(world_model(stamp, self.field, me, ball, opponents, self.playing,
                                     self.grid))
        truth = PoseStamped()
        truth.header.stamp, truth.header.frame_id = stamp, 'map'
        truth.pose.position.x, truth.pose.position.y = bx, by
        truth.pose.orientation.w = 1.0
        self.truth.publish(truth)


def main(args=None):
    """Run the fake world model."""
    rclpy.init(args=args)
    node = FakeWorldModel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
