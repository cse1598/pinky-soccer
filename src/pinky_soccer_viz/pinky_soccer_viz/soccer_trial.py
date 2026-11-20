"""Score a solo goal trial in simulation (evaluation tool, not part of the robot stack).

Waits for /world_model, publishes /game/playing true, then watches the ball ground
truth (/sim/ball_ground_truth from mujoco_eval_bridge) until the ball centre crosses
the opponent goal mouth (pinky_soccer_field.ball_in_goal) or `timeout_sec` of SIM time
passes. Writes a JSON summary (result, time, branch/action histogram, ball state
fractions, last decision) to `result_file`.

ros2 run pinky_soccer_viz soccer_trial --ros-args -p use_sim_time:=true \
    -p result_file:=/tmp/trial.json
"""

from collections import Counter
import json
import sys
import time

from geometry_msgs.msg import PoseStamped
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

STATE = {0: 'OBSERVED', 1: 'ESTIMATED', 2: 'LOST'}


class SoccerTrial(Node):
    """Trial monitor."""

    def __init__(self):
        super().__init__('soccer_trial')
        p = self.declare_parameter
        self.field = Field.from_file(p('field_spec', '').value or None,
                                     p('our_goal_color', 'yellow').value)
        self.timeout = p('timeout_sec', 120.0).value
        self.settle = p('settle_sec', 3.0).value
        self.result_file = p('result_file', '').value
        self.wall_timeout = p('wall_timeout_sec', 900.0).value
        self.playing_pub = self.create_publisher(Bool, 'game/playing', 10)
        self.ball = None
        self.robot = None
        self.wm = None
        self.goal = None
        self.actions = Counter()
        self.ball_states = Counter()
        self.first_observed = None
        self.trace = []
        self.create_subscription(PoseStamped, 'sim/ball_ground_truth', self.on_ball, 10)
        self.create_subscription(PoseStamped, 'ground_truth', self.on_robot, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_wm, 10)
        self.create_subscription(BehaviorGoal, 'behavior_goal', self.on_goal, 10)

    def now(self):
        """Return simulation seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_ball(self, msg):
        """Ball ground truth."""
        self.ball = (msg.pose.position.x, msg.pose.position.y)

    def on_robot(self, msg):
        """Robot ground truth."""
        self.robot = (msg.pose.position.x, msg.pose.position.y)

    def on_wm(self, msg):
        """World model statistics."""
        self.wm = msg
        self.ball_states[STATE.get(msg.ball.state, '?')] += 1
        if msg.ball.state == 0 and self.first_observed is None:
            self.first_observed = self.now()

    def on_goal(self, msg):
        """Behaviour statistics."""
        self.goal = msg
        self.actions[f'{msg.branch}/{msg.action}'] += 1

    def run(self):
        """Run one trial; return the summary dict."""
        wall0 = time.monotonic()
        while rclpy.ok() and (self.wm is None or self.now() <= 0.0):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - wall0 > 120:
                return {'result': 'error', 'reason': 'no /world_model or /clock within 120 s'}
        t_ready = self.now()
        while rclpy.ok() and self.now() - t_ready < self.settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        start = self.now()
        start_ball, start_robot = self.ball, self.robot
        result, next_pub, next_trace = 'timeout', 0.0, 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            t = self.now() - start
            if t >= next_pub:
                self.playing_pub.publish(Bool(data=True))
                next_pub = t + 1.0
            if t >= next_trace:
                self.trace.append({
                    't': round(t, 2), 'ball': self.ball and [round(v, 3) for v in self.ball],
                    'robot': self.robot and [round(v, 3) for v in self.robot],
                    'action': self.goal and f'{self.goal.branch}/{self.goal.action}',
                    'wm_ball_state': self.wm and STATE.get(self.wm.ball.state),
                    'wm_ball': self.wm and [round(self.wm.ball.pose.x, 3),
                                            round(self.wm.ball.pose.y, 3)],
                    'wm_self': self.wm and [round(self.wm.self.pose.x, 3),
                                            round(self.wm.self.pose.y, 3),
                                            round(self.wm.self.pose.theta, 2)],
                    'wm_opponents': self.wm and [
                        [round(o.pose.x, 2), round(o.pose.y, 2), STATE.get(o.state)]
                        for o in self.wm.opponents]})
                next_trace = t + 2.0
            if self.ball and self.field.ball_in_goal(*self.ball, self.field.opp_goal_color):
                result = 'goal'
                break
            if self.ball and self.field.ball_in_goal(*self.ball, self.field.our_goal_color):
                result = 'own_goal'
                break
            if t > self.timeout:
                break
            if time.monotonic() - wall0 > self.wall_timeout:
                result = 'wall_timeout'
                break
        self.playing_pub.publish(Bool(data=False))
        total = sum(self.ball_states.values()) or 1
        summary = {
            'result': result,
            'sim_time_sec': round(self.now() - start, 2),
            'wall_time_sec': round(time.monotonic() - wall0, 1),
            'ball_truth_available': self.ball is not None,
            'start_ball': start_ball, 'start_robot': start_robot,
            'end_ball': self.ball, 'end_robot': self.robot,
            'first_observed_after_sec': (None if self.first_observed is None
                                         else round(self.first_observed - start, 2)),
            'ball_state_fraction': {k: round(v / total, 3) for k, v in self.ball_states.items()},
            'actions': dict(self.actions.most_common()),
            'last_action': self.goal and f'{self.goal.branch}/{self.goal.action}',
            'trace': self.trace,
        }
        return summary


def main(args=None):
    """Run the trial and print/save the summary; exit 0 only on a goal."""
    rclpy.init(args=args)
    node = SoccerTrial()
    try:
        summary = node.run()
    finally:
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        result_file = node.result_file
        node.destroy_node()
        rclpy.try_shutdown()
    text = json.dumps(summary, indent=2)
    print(text)
    if result_file:
        with open(result_file, 'w') as f:
            f.write(text + '\n')
    sys.exit(0 if summary.get('result') == 'goal' else 1)
