"""
Gazebo ground truth for display/evaluation: /ground_truth and /sim/ball_ground_truth.

MuJoCo already publishes /ground_truth (and /sim/ball_ground_truth through
mujoco_eval_bridge). Gazebo has neither. This node subscribes to the world pose stream
`/world/<world>/dynamic_pose/info` (gz.msgs.Pose_V) with the gz-transport Python bindings,
because ros_gz_bridge's Pose_V -> TFMessage conversion drops the entity names, and
republishes the robot and ball model poses as PoseStamped in frame `world`
(= field centre = map). Never a decision input.

It also accepts /sim/set_ball (geometry_msgs/Point, x/y in `world`) from the behaviour verifier
and moves the ball model there with the world's set_pose service.
"""

from geometry_msgs.msg import Point, PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node


def pose_stamped(pose, stamp):
    """Convert one gz.msgs.Pose to a PoseStamped in frame `world`."""
    out = PoseStamped()
    out.header.stamp.sec, out.header.stamp.nanosec = stamp
    out.header.frame_id = 'world'
    p, q = pose.position, pose.orientation
    out.pose.position.x, out.pose.position.y, out.pose.position.z = p.x, p.y, p.z
    out.pose.orientation.x, out.pose.orientation.y = q.x, q.y
    out.pose.orientation.z, out.pose.orientation.w = q.z, q.w
    return out


class GazeboTruthRelay(Node):
    """Pick named model poses out of the Gazebo pose stream."""

    def __init__(self):
        super().__init__('gazebo_truth_relay')
        p = self.declare_parameter
        world = p('world_name', 'rcj_soccer').value
        self.ball_name = p('ball_name', 'soccer_ball').value
        self.ball_z = p('ball_rest_z', 0.0215).value
        names = {p('robot_name', 'pinky').value: 'ground_truth',
                 self.ball_name: 'sim/ball_ground_truth',
                 p('opponent_name', 'opponent').value: 'sim/opponent_ground_truth'}
        self.pubs = {name: self.create_publisher(PoseStamped, topic, 10)
                     for name, topic in names.items()}
        self.topic = f'/world/{world}/dynamic_pose/info'
        self.set_pose = f'/world/{world}/set_pose'
        self.gz = GzNode()
        if not self.gz.subscribe(Pose_V, self.topic, self.on_poses):
            raise RuntimeError(f'cannot subscribe to Gazebo topic {self.topic}')
        self.get_logger().info(f'ground truth from {self.topic} (display/evaluation only)')
        self.create_subscription(Point, 'sim/set_ball', self.on_set_ball, 10)

    def on_set_ball(self, msg):
        """Move the ball model to (x, y) on the floor."""
        req = Pose()
        req.name = self.ball_name
        req.position.x, req.position.y, req.position.z = msg.x, msg.y, self.ball_z
        req.orientation.w = 1.0
        ok, rep = self.gz.request(self.set_pose, req, Pose, Boolean, 1000)
        if ok and rep.data:
            self.get_logger().info(f'ball placed at ({msg.x:.3f}, {msg.y:.3f}) by /sim/set_ball')
        else:
            self.get_logger().warn(f'{self.set_pose} failed for {self.ball_name}')

    def close(self):
        """Stop gz-transport callbacks before Python finalizes (else the process aborts)."""
        if self.gz is not None:
            self.gz.unsubscribe(self.topic)
            self.gz = None

    def on_poses(self, msg):
        """Republish the poses of the configured models (gz-transport thread)."""
        stamp = (msg.header.stamp.sec, msg.header.stamp.nsec)
        for pose in msg.pose:
            pub = self.pubs.get(pose.name)
            if pub is not None:
                pub.publish(pose_stamped(pose, stamp))


def main(args=None):
    """Run the relay."""
    rclpy.init(args=args)
    node = GazeboTruthRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
