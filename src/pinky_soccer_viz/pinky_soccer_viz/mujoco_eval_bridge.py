"""Evaluation-only MuJoCo bridge: the soccer bridge plus /sim/ball_ground_truth.

The official/soccer bridges publish the robot ground truth (/ground_truth) but not the
ball. This wrapper patches pinky_mujoco.bridge.Bridge.publish_state at runtime (no file
in pinky_pro or pinky_soccer_sim is modified) to also publish the free ball body pose as
geometry_msgs/PoseStamped in frame `world` (= field centre = map). It is used only by
sim_field.launch.py with eval_ball_truth:=true, for scoring and UI overlay, never as a
decision input. Re-check after updating pinky_mujoco (Bridge/publish_state/physics names).
"""

import sys

BALL_BODY = 'soccer_ball'
TOPIC = '/sim/ball_ground_truth'


def install(bridge_module):
    """Patch Bridge.publish_state to also publish the ball pose."""
    from geometry_msgs.msg import PoseStamped
    original = bridge_module.Bridge.publish_state

    def publish_state(self, now):
        original(self, now)
        if not hasattr(self, '_ball_truth_pub'):
            self._ball_truth_pub = self.create_publisher(PoseStamped, TOPIC, 10)
            try:
                self._ball_body = self.physics.model.body(BALL_BODY).id
            except KeyError:
                self._ball_body = None
                self.get_logger().warn(f'No body {BALL_BODY}; ball ground truth disabled')
        if self._ball_body is None:
            return
        pos = self.physics.data.xpos[self._ball_body]
        quat = self.physics.data.xquat[self._ball_body]   # w, x, y, z
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = now, 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
        (msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z) = map(float, quat)
        self._ball_truth_pub.publish(msg)

    bridge_module.Bridge.publish_state = publish_state


def main(args=None):
    """Install the patch, then run the unchanged soccer bridge entry point."""
    from pinky_mujoco import bridge
    from pinky_soccer_sim.mujoco_soccer_bridge import main as soccer_main
    install(bridge)
    soccer_main(sys.argv[1:] if args is None else args)


if __name__ == '__main__':
    main()
