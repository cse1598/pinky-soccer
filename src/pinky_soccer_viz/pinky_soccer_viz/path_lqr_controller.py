"""GoToPose-only TVLQR comparison node; uses the existing verifier wire protocol."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel, MotionResult
from pinky_soccer_dynamics import params as dyn_params
from pinky_soccer_dynamics.command_adapter import CommandAdapter, wheels_from_joint_state
from pinky_soccer_mpc.arrival import Arrival
from .path_lqr_core import DT, Limits, plan, track, wrap


class PathLqrController(Node):
    def __init__(self):
        super().__init__('path_lqr_controller')
        p = self.declare_parameter
        target = p('target', 'mujoco').value
        if target == 'real':
            raise ValueError('LQR first revision is simulation-only')
        profile = p('calibration_profile', '').value
        self.params = dyn_params.load(target, profile=profile or None)
        self.adapter = CommandAdapter(self.params)
        self.kind = p('path_planner', 'bezier').value
        self.v = float(p('max_lin', 0.).value) or .27
        self.w = float(p('max_ang', 0.).value) or 2.
        self.goal = self.world = self.wheels = None
        self.goal_at = self.world_at = self.wheel_at = -math.inf
        self.key = None
        self.trajectory = None
        self.epoch = None
        self.last_clock = None
        self.started = None
        self.finished = None
        self.retries = 0
        self.previous = np.zeros(2)
        self.arrival = Arrival()
        self.plan_ms = 0.
        self.tracking_ms = 0.
        self.wait_since = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'mpc/predicted_path', 10)
        self.status_pub = self.create_publisher(String, 'mpc/status', 10)
        self.result_pub = self.create_publisher(MotionResult, 'motion/result', 10)
        self.create_subscription(BehaviorGoal, 'behavior_goal', self.on_goal, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_timer(DT, self.tick)
        self.get_logger().info(f'GoToPose path + TVLQR {self.kind}: 50 Hz, simulation only')

    def on_goal(self, msg):
        self.goal, self.goal_at = msg, time.monotonic()

    def on_world(self, msg):
        self.world, self.world_at = msg, time.monotonic()

    def on_joints(self, msg):
        try:
            wheel = wheels_from_joint_state(self.params, msg.name, msg.velocity)
            if np.all(np.isfinite(wheel)):
                self.wheels, self.wheel_at = np.asarray(wheel), time.monotonic()
        except (KeyError, ValueError, IndexError):
            pass

    def limits(self):
        p = self.params
        v = min(self.v, self.goal.max_speed) if self.goal.max_speed > 0 else self.v
        return Limits(p.m('left_radius'), p.m('right_radius'), p.m('track'),
                      p.wheel_cmd_max(), p.wheel_accel_cmd_max(), v, self.w,
                      self.adapter.r, self.adapter.b,
                      min(v, self.adapter.max_linear or v),
                      min(self.w, self.adapter.max_angular or self.w))

    def send(self, wheels):
        self.previous = np.asarray(wheels)
        v, w = self.adapter.to_twist(*self.previous)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)
        return float(v), float(w)

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.trajectory is not None:
            for x, y, theta in self.trajectory.states[::4]:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z = math.sin(theta/2)
                pose.pose.orientation.w = math.cos(theta/2)
                msg.poses.append(pose)
            x, y, theta = self.trajectory.states[-1]
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(theta/2), math.cos(theta/2)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def report(self, state, reason, result=MotionResult.RUNNING, dist=0., ang=0., **extra):
        t = self.trajectory
        gid = int(self.goal.goal_id) if self.goal is not None else 0
        data = dict(state=state, reason=reason, goal_id=str(gid), backend='path_tvlqr',
                    position_error=float(dist), heading_error=float(ang),
                    ms=self.tracking_ms, plan_ms=self.plan_ms,
                    planner_requested=self.kind, planner_actual=t.kind if t else None,
                    direction=t.direction if t else None,
                    horizon_steps=len(t.inputs) if t else None,
                    planned_time=t.duration if t else None, retries=self.retries)
        data.update(extra)
        self.status_pub.publish(String(data=json.dumps(data)))
        if gid and self.goal.action == 'GoToPose':
            msg = MotionResult()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.goal_id, msg.state, msg.reason = gid, result, reason
            msg.position_error, msg.heading_error = float(dist), float(ang)
            self.result_pub.publish(msg)

    def reset(self):
        self.trajectory = None
        self.epoch = None
        self.wait_since = None
        self.arrival.reset()
        self.publish_path()

    def stop(self, reason, result=MotionResult.RUNNING, dist=0., ang=0.):
        self.send((0., 0.))
        self.report('failed' if result == MotionResult.FAILURE else
                    'arrived' if result == MotionResult.SUCCESS else 'stop',
                    reason, result, dist, ang)

    def fail(self, reason, dist=0., ang=0.):
        self.finished = (MotionResult.FAILURE, reason)
        self.stop(reason, MotionResult.FAILURE, dist, ang)

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        wall = time.monotonic()
        elapsed = DT if self.last_clock is None else now-self.last_clock
        self.last_clock = now
        if elapsed < 0 or elapsed > .5:
            self.reset()
            self.key = None
            return self.stop('clock discontinuity')
        if self.goal is None or wall-self.goal_at > .5 or self.goal.mode == BehaviorGoal.STOP:
            if self.key is not None:
                self.reset()
            self.key = None
            return self.stop('STOP or stale goal')
        if self.goal.action != 'GoToPose' or not self.goal.goal_id:
            return self.stop('only GoToPose is supported')
        g = self.goal
        key = (int(g.goal_id), bool(g.avoid_walls), bool(g.avoid_opponents), bool(g.avoid_ball))
        if key != self.key:
            self.key = key
            self.reset()
            self.started = now
            self.finished = None
            self.retries = 0
        if self.finished is not None:
            result, reason = self.finished
            return self.stop(reason, result)
        if any(key[1:]):
            return self.fail('LQR avoidance not implemented; disable all avoid options')
        if now-self.started > 28.:
            return self.fail('LQR execution timeout')
        if self.world is None or wall-self.world_at > .5:
            self.reset()
            return self.stop('stale world model')
        obj = self.world.self
        pose = np.array([obj.pose.x, obj.pose.y, obj.pose.theta])
        goal = np.array([g.target.x, g.target.y, g.target.theta])
        if obj.state == obj.LOST or obj.confidence <= 0 or not np.all(np.isfinite([pose, goal])):
            self.reset()
            return self.stop('invalid localization or target')
        if self.wheels is None or wall-self.wheel_at > .2:
            self.reset()
            return self.stop('stale wheel feedback')
        lim = self.limits()
        v_meas, w_meas = lim.twist(self.wheels)
        world_v = math.hypot(obj.velocity.x, obj.velocity.y)
        if not math.isfinite(world_v):
            return self.stop('invalid world velocity')
        inside, done, dist, ang = self.arrival.update(
            now, pose, goal, max(abs(v_meas), world_v), w_meas, self.wheels, True)
        if inside:
            self.send((0., 0.))
            # If a transient crossing leaves the tolerance, start a fresh trajectory.
            self.trajectory = None
            self.epoch = None
            self.wait_since = None
            if done:
                self.finished = (MotionResult.SUCCESS, 'settled')
            return self.report('arrived' if done else 'settle', 'settled' if done else 'waiting for rest',
                               MotionResult.SUCCESS if done else MotionResult.RUNNING, dist, ang)
        if self.trajectory is not None and self.epoch is not None and now-self.epoch >= self.trajectory.duration:
            self.send((0., 0.))
            if self.wait_since is None:
                self.wait_since = now
            if max(abs(self.wheels)) <= .2:
                if self.retries >= 2:
                    return self.fail('terminal pose outside tolerance after two recovery plans', dist, ang)
                self.retries += 1
                self.reset()
            elif now-self.wait_since > 2.:
                return self.fail('wheel speeds did not settle', dist, ang)
            return self.report('settle', 'end of reference; checking terminal error', dist=dist, ang=ang)
        if self.trajectory is None:
            self.send((0., 0.))
            if max(abs(self.wheels)) > .2:
                return self.report('waiting', 'waiting for rest before planning', dist=dist, ang=ang)
            began = time.perf_counter()
            try:
                self.trajectory = plan(pose, goal, lim, 'straight' if self.retries else self.kind)
            except (ValueError, np.linalg.LinAlgError) as exc:
                return self.fail(f'planner failed: {exc}', dist, ang)
            self.plan_ms = (time.perf_counter()-began)*1000
            self.publish_path()
            return self.report('waiting', 'reference generated', dist=dist, ang=ang)
        if self.epoch is None:
            # Do not advance the reference while generating or publishing it.
            self.epoch = now
        i = min(len(self.trajectory.inputs)-1, int((now-self.epoch)/DT))
        began = time.perf_counter()
        u = track(self.trajectory, i, pose)
        if not np.all(np.isfinite(u)):
            return self.fail('nonfinite LQR command', dist, ang)
        wheels = lim.command(u, self.previous, min(max(elapsed, 0.), DT))
        v, w = self.send(wheels)
        self.tracking_ms = (time.perf_counter()-began)*1000
        ref = self.trajectory.states[i]
        self.report('track', 'TVLQR tracking', dist=dist, ang=ang, v=v, w=w,
                    tracking_error_m=float(np.linalg.norm(pose[:2]-ref[:2])),
                    tracking_heading_error_rad=abs(wrap(pose[2]-ref[2])),
                    reference_v=float(self.trajectory.inputs[i, 0]),
                    reference_w=float(self.trajectory.inputs[i, 1]),
                    gain=self.trajectory.gains[i].tolist(), reference_index=i,
                    wheel_l=float(self.wheels[0]), wheel_r=float(self.wheels[1]),
                    wheel_cmd_l=float(wheels[0]), wheel_cmd_r=float(wheels[1]))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathLqrController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.send((0., 0.))
            node.destroy_node()
        rclpy.try_shutdown()
