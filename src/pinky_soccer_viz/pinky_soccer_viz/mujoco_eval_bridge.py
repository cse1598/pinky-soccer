"""Evaluation-only MuJoCo bridge: the soccer bridge plus /sim/ball_ground_truth.

The official/soccer bridges publish the robot ground truth (/ground_truth) but not the
ball. This wrapper patches pinky_mujoco.bridge.Bridge.publish_state at runtime (no file
in pinky_pro or pinky_soccer_sim is modified) to also publish the free ball body pose as
geometry_msgs/PoseStamped in frame `world` (= field centre = map). It is used only by
sim_field.launch.py with eval_ball_truth:=true, for scoring and UI overlay, never as a
decision input. Re-check after updating pinky_mujoco (Bridge/publish_state/physics names).

It also accepts /sim/set_ball (geometry_msgs/Point, x/y in `world`) from the behaviour verifier:
the ball is moved there at rest. The request is applied inside publish_state, i.e. between
physics steps in the bridge loop (under the viewer lock), never concurrently with a step.
"""

import sys

BALL_BODY = 'soccer_ball'
TOPIC = '/sim/ball_ground_truth'
SET_BALL_TOPIC = '/sim/set_ball'


def place_free_body(model, data, body_id, x, y):
    """Move a free-joint body to (x, y) at its current height, at rest, upright."""
    import mujoco
    joint = model.body_jntadr[body_id]
    if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
        return False
    q, v = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
    data.qpos[q:q + 2] = (x, y)
    data.qpos[q + 3:q + 7] = (1.0, 0.0, 0.0, 0.0)
    data.qvel[v:v + 6] = 0.0
    mujoco.mj_forward(model, data)
    return True


def install(bridge_module):
    """Patch Bridge.publish_state to also publish the ball pose."""
    from geometry_msgs.msg import Point, PoseStamped
    original = bridge_module.Bridge.publish_state

    def publish_state(self, now):
        original(self, now)
        if not hasattr(self, '_ball_truth_pub'):
            self._ball_truth_pub = self.create_publisher(PoseStamped, TOPIC, 10)
            self._ball_request = None
            self.create_subscription(Point, SET_BALL_TOPIC,
                                     lambda m: setattr(self, '_ball_request', (m.x, m.y)), 10)
            try:
                self._ball_body = self.physics.model.body(BALL_BODY).id
            except KeyError:
                self._ball_body = None
                self.get_logger().warn(f'No body {BALL_BODY}; ball ground truth disabled')
        if self._ball_body is None:
            return
        if self._ball_request is not None:
            x, y = self._ball_request
            self._ball_request = None
            if place_free_body(self.physics.model, self.physics.data, self._ball_body, x, y):
                self.get_logger().info(f'ball placed at ({x:.3f}, {y:.3f}) by /sim/set_ball')
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
