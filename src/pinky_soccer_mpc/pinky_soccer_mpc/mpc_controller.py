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
is stale, on STOP, or when the solver fails; the robot-side command_watchdog
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

from .pose_mpc import PoseMpc
from pinky_soccer_dynamics.profile_model import motor_step, limit_command


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Arrival:
    """
    Final phase at the goal: track -> align (turn in place) -> hold, with hysteresis.

    Within pos_tol the NMPC hands over to a pure turn in place towards the goal heading, then
    the robot holds still. In MuJoCo the NMPC alone kept correcting around the goal (up to
    11 cm, 80 deg): pivot turns slip a lot there, so its small corrections never matched.
    The window is inside the BT's GoToPose tolerance (5 cm, 0.15 rad). The NMPC takes over
    again when the goal moves or the robot leaves 2 x pos_tol.
    """

    TRACK, ALIGN, HOLD = 'track', 'align', 'hold'

    def __init__(self, pos_tol=0.04, ang_tol=0.12):
        self.pos_tol, self.ang_tol = pos_tol, ang_tol
        self.mode = self.TRACK
        self.goal = None

    def update(self, pose, goal, wheel_speed):
        """Return the phase for this cycle and the heading error [rad]."""
        dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        err = wrap(goal[2] - pose[2])
        if self.mode != self.TRACK:
            moved = math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 0.01 \
                or abs(wrap(goal[2] - self.goal[2])) > 0.03
            if moved or dist > 2 * self.pos_tol:
                self.mode = self.TRACK
        if self.mode == self.TRACK and dist <= self.pos_tol:
            self.mode, self.goal = self.ALIGN, tuple(goal)
        if self.mode == self.ALIGN and abs(err) <= self.ang_tol and wheel_speed < 1.0:
            self.mode = self.HOLD
        elif self.mode == self.HOLD and abs(err) > 2 * self.ang_tol:
            self.mode = self.ALIGN
        return self.mode, err


class MpcController(Node):
    """BehaviorGoal follower with the point-to-pose NMPC."""

    def __init__(self):
        super().__init__('mpc_controller')
        p = self.declare_parameter
        target = p('target', 'real').value
        profile = p('calibration_profile', '').value
        identification = p('identification', '').value
        self.params = dyn_params.load(target, profile=profile or None,
                                      identification=identification or None)
        if identification:
            raise ValueError('Legacy single-lag identification is incompatible with profile-v1; '
                             'identify the two-stage model first')
        if self.params.m('delay') > 0.0:
            raise ValueError('NMPC does not yet model command dead time; use a validated '
                             'zero-delay identification or implement delay compensation first')
        self.max_lin = float(p('max_lin', 0.0).value)      # 0: only the wheel limits
        self.max_ang = float(p('max_ang', 0.0).value)
        self.goal_timeout = p('goal_timeout', 0.5).value
        self.pose_timeout = p('pose_timeout', 0.5).value
        rate = p('rate_hz', 20.0).value
        self.mpc = PoseMpc(self.params, p('horizon', 2.0).value, int(p('steps', 40).value))
        if abs(self.mpc.dt - 1.0/rate) > 1e-6:
            raise ValueError('NMPC prediction step must match control period')
        self.adapter = CommandAdapter(self.params)
        self.arrival = Arrival(p('pos_tol', 0.04).value, p('ang_tol', 0.12).value)
        # Ball-related actions keep the camera (front) towards the ball: no reverse driving.
        self.forward_only = set(p('forward_only_actions', [
            'GoBehindBall', 'PushBall', 'GoToBlockPoint', 'FaceBall', 'TurnToLastSeen']).value)
        self.dt = 1.0 / rate
        self.goal = self.goal_time = None
        self.pose = self.pose_time = None
        self.omega = (0.0, 0.0)
        self.ref = (0.0, 0.0)
        self.last_cmd = (0.0, 0.0)
        self.last_tick = None
        self.feedback_age = None
        self.failures = 0
        self.last_goal = None
        self.measured = self.measured_time = None
        self.wheel_feedback = p('wheel_feedback', False).value
        if self.wheel_feedback:
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(JointState, 'joint_states', self.on_joints, qos_profile_sensor_data)
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
            measured = wheels_from_joint_state(self.params, msg.name, msg.velocity)
            if all(math.isfinite(v) for v in measured):
                self.measured, self.measured_time = measured, time.monotonic()
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

    def send(self, cmd):
        """Store the transmitted wheel command and publish driver twist."""
        self.last_cmd = tuple(float(v) for v in cmd)
        v, w = self.adapter.to_twist(*self.last_cmd)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)
        return v, w

    def stop(self, reason):
        self.send((0., 0.))
        self.mpc.reset()
        self.publish_status({'state': 'stop', 'reason': reason})

    def tick(self):
        now = time.monotonic()
        sim_now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = self.dt if self.last_tick is None else sim_now-self.last_tick
        self.last_tick = sim_now
        if elapsed < 0 or elapsed > .5:
            self.ref = self.omega = self.last_cmd = (0.,0.)
            return self.stop('clock discontinuity')
        self.omega, self.ref = motor_step(self.params,self.omega,self.ref,self.last_cmd,elapsed)
        self.feedback_age = None if self.measured_time is None else now-self.measured_time
        if self.feedback_age is not None and self.feedback_age < .2:
            self.omega = self.measured
        elif self.wheel_feedback:
            return self.stop('stale wheel feedback')
        fresh = (self.goal is not None and now-self.goal_time <= self.goal_timeout
                 and self.pose is not None and now-self.pose_time <= self.pose_timeout)
        if not fresh or self.goal.mode == BehaviorGoal.STOP:
            self.arrival.mode, self.arrival.goal = Arrival.TRACK,None
            return self.stop('STOP goal' if fresh else 'stale input')
        t = self.goal.target
        goal = (t.x,t.y,t.theta)
        if not all(math.isfinite(v) for v in (*goal,*self.pose)):
            return self.stop('nonfinite pose/goal')
        self.last_goal = goal
        phase, err = self.arrival.update(self.pose,goal,sum(abs(v) for v in self.omega))
        v_cap,w_cap = self.caps(self.goal)
        vb = min(v_cap,self.mpc.v_limit) if v_cap else self.mpc.v_limit
        wb = min(w_cap,self.mpc.w_limit) if w_cap else self.mpc.w_limit
        if phase == Arrival.HOLD:
            return self.stop('arrived')
        if phase == Arrival.ALIGN:
            return self.align(err,vb,wb)
        x0 = (*self.pose,*self.omega,*self.ref,*self.last_cmd)
        try:
            res = self.mpc.solve(x0,goal,v_cap,w_cap,
                                 forward_only=self.goal.action in self.forward_only)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().warn(f'NMPC exception: {exc}',throttle_duration_sec=1.)
            return self.stop('solver exception')
        if not res['ok']:
            self.failures += 1
            self.get_logger().warn(f'NMPC status {res["status"]}',throttle_duration_sec=1.)
            self.send((0.,0.))
            self.mpc.reset()
            return self.publish_status({'state':'failed','status':res['status'],
                                        'ms':res['ms'],'residuals':res['residuals']})
        self.failures = 0
        v,w = self.send(res['u'])
        self.publish_path(res['xs'])
        self.publish_status({'state':'track','status':res['status'],'ms':round(res['ms'],2),
                             'v':v,'w':w,'v_bound':res['v_bound'],'w_bound':res['w_bound'],
                             'direction':res['direction'],'residuals':res['residuals'],
                             'period_s':elapsed})

    def align(self, err, vb, wb):
        """Brake translation, then align without a forced minimum rotation."""
        v_actual = (self.mpc.rl*self.omega[0]+self.mpc.rr*self.omega[1])/2
        if abs(err) <= self.arrival.ang_tol or abs(v_actual) > .04:
            cmd = (0.,0.)
        else:
            w = max(-min(wb,2.),min(min(wb,2.),2.5*err))
            cmd = limit_command(self.params,(-w*self.mpc.b/(2*self.mpc.rl),
                                              w*self.mpc.b/(2*self.mpc.rr)),vb,wb)
        self.send(cmd)
        self.mpc.reset()
        self.publish_status({'state':'align','heading_error':err})

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
        data.update(wheel_l=float(self.omega[0]),wheel_r=float(self.omega[1]),
                    profile_l=float(self.ref[0]),profile_r=float(self.ref[1]),
                    wheel_cmd_l=self.last_cmd[0],wheel_cmd_r=self.last_cmd[1],
                    feedback_age=self.feedback_age,feedback_enabled=self.wheel_feedback)
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
