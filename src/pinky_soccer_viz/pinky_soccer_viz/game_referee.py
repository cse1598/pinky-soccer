"""
Referee: detects goals and drives the kickoff cycle for every robot.

Goal: the ball has crossed the goal line between the posts (by default the whole ball, i.e.
its centre is one radius past the line), while the game is playing.
On a goal it publishes (all global topics, shared by every robot namespace):
  /game/goal      std_msgs/String JSON {"goal": "blue"|"yellow", "scorer": ..., "score": {...}}
  /game/playing   false  -> every world model reports playing = false
  /game/kickoff   true   -> every bt_node drives to its kickoff pose and waits there
  /game/score     std_msgs/String JSON, latched
In simulation the ball is put back on the centre spot (/sim/set_ball) after reset_delay.
The next /game/playing true (the human start signal) clears /game/kickoff.

Ball source:
  truth        simulation ground truth (/sim/ball_ground_truth); a referee is not a decision
               input, like the evaluation tools
  world_model  real robots: any listed robot world model whose ball is not LOST and whose
               2-sigma ellipse lies past the line, for `confirm` consecutive messages
"""

import json
import math

from geometry_msgs.msg import Point, PoseStamped
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import WorldModel
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

LOST = 2


def crossed(field, x, y, color, past=0.0):
    """Return True when (x, y) is `past` metres beyond `color`'s goal line inside the posts."""
    s = field.color_sign(color)
    return s * x > field.half_length + past and abs(y) < field.spec['goal_width'] / 2


class GameReferee(Node):
    """Goal detection and kickoff state."""

    def __init__(self):
        super().__init__('game_referee')
        p = self.declare_parameter
        self.field = Field.from_file(p('field_spec', '').value or None)
        self.source = p('source', 'truth').value
        self.ball_radius = p('ball_radius', 0.02135).value
        self.full_crossing = p('full_crossing', True).value
        self.reset_ball = p('reset_ball', False).value
        self.reset_delay = p('reset_delay', 1.0).value
        self.confirm = int(p('confirm', 3).value)
        sigma_limit = p('max_sigma', 0.05).value
        self.sigma_limit = sigma_limit
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.playing = False
        self.kickoff = False
        self.score = {'yellow': 0, 'blue': 0}
        self.streak = 0
        self.reset_timer = None
        self.playing_pub = self.create_publisher(Bool, '/game/playing', 10)
        self.kickoff_pub = self.create_publisher(Bool, '/game/kickoff', latched)
        self.goal_pub = self.create_publisher(String, '/game/goal', 10)
        self.score_pub = self.create_publisher(String, '/game/score', latched)
        self.ball_pub = self.create_publisher(Point, '/sim/set_ball', 10)
        self.create_subscription(Bool, '/game/playing', self.on_playing, 10)
        if self.source == 'truth':
            self.create_subscription(PoseStamped, p('ball_truth_topic',
                                                    '/sim/ball_ground_truth').value,
                                     self.on_truth, 10)
        elif self.source == 'world_model':
            for topic in p('world_model_topics', ['/world_model']).value:
                self.create_subscription(WorldModel, topic, self.on_world, 10)
        else:
            raise ValueError('source must be truth or world_model')
        # The BT subscription is volatile with depth 1: repeat the flag so late joiners see it.
        self.create_timer(0.5, self.publish_state)
        self.publish_state()
        self.get_logger().info(f'referee: goals from {self.source}, '
                               f'{"whole ball" if self.full_crossing else "ball centre"} '
                               f'past the line; ball reset {"on" if self.reset_ball else "off"}')

    # ------------------------------------------------------------------ inputs
    def on_playing(self, msg):
        """Start signal clears the kickoff state."""
        self.playing = bool(msg.data)
        if self.playing and self.kickoff:
            self.kickoff = False
            self.streak = 0
            self.get_logger().info('kickoff done: game restarted')
            self.publish_state()

    def past(self):
        return self.ball_radius if self.full_crossing else 0.0

    def check(self, x, y, margin=0.0):
        """Return the goal colour the ball is in, or None."""
        for color in ('yellow', 'blue'):
            if crossed(self.field, x, y, color, self.past() + margin):
                return color
        return None

    def on_truth(self, msg):
        """Judge from the true ball position (simulation)."""
        self.update(self.check(msg.pose.position.x, msg.pose.position.y), confirm=1)

    def on_world(self, msg):
        """Real robots: a confident world model ball estimate decides."""
        b = msg.ball
        if b.state == LOST:
            return
        sigma = math.sqrt(max(b.covariance[0], b.covariance[4], 0.0))
        if sigma > self.sigma_limit:
            return
        self.update(self.check(b.pose.x, b.pose.y, 2 * sigma), confirm=self.confirm)

    # ------------------------------------------------------------------ state
    def update(self, goal_color, confirm):
        """Count a goal once per kickoff cycle."""
        if not self.playing or self.kickoff:
            self.streak = 0
            return
        self.streak = self.streak + 1 if goal_color else 0
        if goal_color and self.streak >= confirm:
            self.goal(goal_color)

    def goal(self, goal_color):
        """Score, stop the game and request the kickoff positions."""
        scorer = 'blue' if goal_color == 'yellow' else 'yellow'
        self.score[scorer] += 1
        self.kickoff, self.playing, self.streak = True, False, 0
        event = {'goal': goal_color, 'scorer': scorer, 'score': self.score,
                 'stamp': self.get_clock().now().nanoseconds * 1e-9}
        self.goal_pub.publish(String(data=json.dumps(event)))
        self.playing_pub.publish(Bool(data=False))
        self.publish_state()
        self.get_logger().info(f'GOAL in the {goal_color} goal: {scorer} scores '
                               f'(yellow {self.score["yellow"]} : blue {self.score["blue"]})')
        if self.reset_ball:
            if self.reset_timer is not None:
                self.reset_timer.cancel()
            self.reset_timer = self.create_timer(self.reset_delay, self.put_ball_back)

    def put_ball_back(self):
        """Teleport the ball to the centre spot, at rest (simulation)."""
        self.reset_timer.cancel()
        self.reset_timer = None
        self.ball_pub.publish(Point(x=0.0, y=0.0, z=0.0))
        self.get_logger().info('ball placed on the centre spot')

    def publish_state(self):
        """Publish the kickoff flag and the score."""
        self.kickoff_pub.publish(Bool(data=self.kickoff))
        self.score_pub.publish(String(data=json.dumps(self.score)))


def main(args=None):
    """Run the referee."""
    rclpy.init(args=args)
    node = GameReferee()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
