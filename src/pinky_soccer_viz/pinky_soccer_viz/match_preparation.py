"""Match procedure/command ownership. Reuses sensor localization and GoToPose results."""
import json
import math
import time
from collections import deque

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import BehaviorGoal, MotionResult
from pinky_interfaces.srv import SetLamp, SetLed
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .preparation_core import Preparation, start_pose
from .attack_match import RefereeGate


class MatchPreparation(Node):
    def __init__(self):
        super().__init__('match_preparation')
        p = self.declare_parameter
        self.core = Preparation(p('our_goal_color', 'yellow').value)
        self.require_referee = bool(p('require_referee', False).value)
        self.return_after_out = bool(p('return_after_out', self.require_referee).value)
        self.referee_gate = RefereeGate(float(p('restart_center_radius', .15).value))
        self.referee_reset = None
        self.start_requested = None
        self.start_announce = True
        self.referee_pause_until = -math.inf
        self.referee_client = self.create_client(Trigger, '/referee/reset_play')
        self.field = Field.from_file(p('field_spec', '').value or None, self.core.team)
        self.goal_offset = float(p('start_goal_offset', -1.).value)
        self.explicit_start = p('start_pose', [float('nan')]*3).value
        if all(math.isnan(x) for x in self.explicit_start):
            self.explicit_start = []
        self.target = start_pose(self.field, self.core.team, self.goal_offset, self.explicit_start)
        self.avoid = {k: bool(p('start_'+k, v).value) for k, v in dict(
            avoid_walls=False, avoid_opponents=False, avoid_ball=False).items()}
        self.move_timeout = float(p('move_timeout', 60.).value)
        self.calibration_timeout = float(p('preparation_timeout', 90.).value)
        self.settle_sec = float(p('stop_settle_sec', .3).value)
        self.stop_linear = float(p('stop_linear_mps', .01).value)
        self.stop_angular = float(p('stop_angular_rps', .05).value)
        if (not all(math.isfinite(v) and v > 0 for v in
                    (self.move_timeout, self.calibration_timeout)) or
                not all(math.isfinite(v) and v >= 0 for v in
                    (self.settle_sec, self.stop_linear, self.stop_angular))):
            raise ValueError('Preparation timeouts must be positive; stop thresholds finite/nonnegative')
        self.loc = {}
        self.loc_received = -math.inf
        self.odom_received = -math.inf
        self.camera_received = -math.inf
        self.odom_speed = (math.inf, math.inf)
        self.stopped_since = None
        self.entered = time.monotonic()
        self.goal_id = 0
        self.motion_received = -math.inf
        self.command_received = -math.inf
        self.calibrate_future = None
        self.calibrate_sent = False
        self.calibration_ack = False
        self.previous_record = None
        self.led_future = None
        self.led_key = None
        self.led_sent = -math.inf
        self.led_status = 'unavailable'
        self.last_status = -math.inf
        self.led_cost_ms = deque(maxlen=600)
        self.kickoff_seen = False
        self.kickoff_pending = None
        self.calibrate_client = self.create_client(Trigger, 'localization/calibrate')
        self.abort_client = self.create_client(Trigger, 'localization/abort')
        self.led_client = self.create_client(SetLed, 'set_led')
        self.lamp_client = self.create_client(SetLamp, 'set_lamp')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'preparation/status', latched)
        self.playing_pub = self.create_publisher(Bool, 'preparation/playing', latched)
        self.global_playing_pub = self.create_publisher(Bool, '/game/playing', 10)
        self.team_pub = self.create_publisher(String, 'preparation/team', latched)
        self.start_pub = self.create_publisher(Pose2D, 'preparation/start_pose', latched)
        self.goal_pub = self.create_publisher(BehaviorGoal, 'behavior_goal', 10)
        self.command_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.led_pub = self.create_publisher(String, 'preparation/led', latched)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera/camera_info',
            lambda msg: setattr(self, 'camera_received', time.monotonic()), qos_profile_sensor_data)
        self.create_subscription(MotionResult, 'motion/result', self.on_result, 10)
        self.create_subscription(BehaviorGoal, 'preparation/strategy_goal', self.on_strategy, 10)
        self.create_subscription(Twist, 'preparation/localized_cmd_vel', self.on_command, 10)
        self.create_subscription(Bool, '/game/playing', self.on_playing, 10)
        self.create_subscription(Bool, '/game/kickoff', self.on_kickoff, latched)
        for name, action in [('prepare', self.prepare), ('retry', self.prepare),
                             ('start', self.start), ('stop', self.stop)]:
            self.create_service(Trigger, 'preparation/'+name,
                lambda req, res, fn=action: self.reply(res, fn()))
        self.create_service(SetBool, 'preparation/set_team', self.set_team)
        if self.require_referee:
            from pinky_soccer_referee.msg import RefereeState, RefereeEvent
            self.create_subscription(RefereeState, '/referee/state', self.on_referee, latched)
            self.create_subscription(RefereeEvent, '/referee/events', self.on_referee_event, latched)
        self.create_timer(.05, self.tick)
        self.publish_team()

    @staticmethod
    def reply(response, result):
        response.success, response.message = result
        return response

    def localization_valid(self):
        return bool(self.loc.get('ready') and time.monotonic()-self.loc_received < 1.)

    def publish_team(self):
        self.team_pub.publish(String(data=self.core.team))
        self.start_pub.publish(Pose2D(x=self.target[0], y=self.target[1], theta=self.target[2]))

    def set_team(self, req, res):
        if self.core.state not in ('IDLE', 'ERROR', 'STOPPED'):
            return self.reply(res, (False, 'Select team before preparation'))
        team = 'blue' if req.data else 'yellow'
        try:
            target = start_pose(self.field, team, self.goal_offset, self.explicit_start)
        except ValueError as e:
            return self.reply(res, (False, str(e)))
        self.core.team, self.target = team, target
        self.core.prepared = self.core.arrived = False
        self.core.state, self.core.reason = 'IDLE', ''
        self.publish_team()
        return self.reply(res, (True, team))

    def prepare(self):
        result = self.core.prepare()
        if result[0]:
            self.entered = time.monotonic()
            self.previous_record = self.loc.get('records')
            self.loc = {}
            self.calibration_ack = False
            self.calibrate_sent = False
            self.calibrate_future = None
            self.stopped_since = None
            self.command_pub.publish(Twist())
            self.publish_stop_goal()
        return result

    def referee_now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_referee(self, msg):
        age = self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        self.referee_gate.update(self.referee_now(), bool(msg.metric_valid and msg.calibrated and -.05 <= age < .5),
            (msg.ball.position.x, msg.ball.position.y) if msg.ball.visible else None, msg.armed)

    def on_referee_event(self, msg):
        age = self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        action = self.referee_gate.event(msg.sequence, msg.event,
            -.05 <= age < 1. and self.core.prepared and (self.core.state == 'PLAYING' or
                (self.core.state == 'STOPPED' and self.referee_now() <= self.referee_pause_until)))
        if action:
            self.stop()
            self.core.reason = 'Referee: '+msg.event
            if (action == 'RETURN' or self.return_after_out) and self.localization_valid():
                self.kickoff_pending = time.monotonic()+.15

    def start(self, announce=True):
        if self.core.state == 'PLAYING':
            return True, 'Already playing'
        if self.require_referee:
            if self.start_requested is not None:
                return False, 'Start already pending referee confirmation'
            if not self.referee_gate.can_start(self.referee_now()):
                return False, 'Fresh referee and ball at centre required'
            if not self.core.prepared or self.core.state not in ('READY', 'STOPPED') or not self.localization_valid():
                return False, 'Preparation must be complete before start'
            if not self.referee_client.service_is_ready():
                return False, 'Referee reset_play service unavailable'
            self.referee_reset = self.referee_client.call_async(Trigger.Request())
            self.start_requested, self.start_announce = time.monotonic(), announce
            return True, 'Waiting for referee to arm; robot remains stopped'
        result = self.core.start(self.localization_valid())
        if result[0] and announce:
            self.global_playing_pub.publish(Bool(data=True))
        return result

    def stop(self, announce=True):
        self.referee_pause_until = -math.inf
        if self.referee_reset is not None:
            self.referee_reset.cancel()
        self.start_requested = self.referee_reset = None
        calibration = self.core.state == 'CALIBRATING'
        self.core.stop()
        if announce or not self.kickoff_seen:
            self.kickoff_pending = None
        self.command_pub.publish(Twist())
        self.publish_stop_goal()
        if announce:
            self.global_playing_pub.publish(Bool(data=False))
        if calibration and self.abort_client.service_is_ready():
            self.abort_client.call_async(Trigger.Request())
        return True, 'Stopped'

    def fail(self, reason):
        was_calibrating = self.core.state == 'CALIBRATING'
        self.core.fail(reason)
        self.command_pub.publish(Twist())
        self.publish_stop_goal()
        if was_calibrating and self.abort_client.service_is_ready():
            self.abort_client.call_async(Trigger.Request())

    def on_playing(self, msg):
        if msg.data:
            ok, reason = self.start(announce=False)
            if not ok:
                self.get_logger().warn(reason, throttle_duration_sec=2.)
        elif self.core.state in ('CALIBRATING', 'MOVING_TO_START', 'PLAYING'):
            was_playing = self.core.state == 'PLAYING'
            self.stop(announce=False)
            if was_playing:
                self.referee_pause_until = self.referee_now()+.5

    def on_kickoff(self, msg):
        rising = msg.data and not self.kickoff_seen
        self.kickoff_seen = msg.data
        if rising and self.core.prepared and self.localization_valid():
            # Referee publishes pause then kickoff. Defer through the callback queue.
            self.kickoff_pending = time.monotonic()+.15

    def on_localization(self, msg):
        try:
            self.loc = json.loads(msg.data)
            self.loc_received = time.monotonic()
        except (ValueError, TypeError):
            return
        if self.core.state == 'CALIBRATING' and self.calibration_ack:
            if not self.loc.get('records') or self.loc.get('records') == self.previous_record:
                return
            if (self.loc.get('ready') and self.loc.get('records')
                    and self.loc.get('records') != self.previous_record):
                self.core.calibrated()
                self.entered = time.monotonic()
                self.goal_id = time.time_ns()
                self.motion_received = -math.inf
            elif self.loc.get('state') in ('GOAL_COLOR_FAILED', 'CALIBRATION_TIMEOUT',
                    'ABORTED', 'REINITIALIZATION_REQUIRED', 'ROTATION_LIMIT'):
                self.fail('Localization: '+str(self.loc.get('state')))

    def on_odom(self, msg):
        self.odom_received = time.monotonic()
        v = msg.twist.twist
        self.odom_speed = (abs(v.linear.x), abs(v.angular.z))

    def on_result(self, msg):
        if self.core.state != 'MOVING_TO_START' or msg.goal_id != self.goal_id:
            return
        self.motion_received = time.monotonic()
        if msg.state == MotionResult.FAILURE:
            self.fail('GoToPose: '+msg.reason)
        elif msg.state == MotionResult.SUCCESS:
            self.core.arrival()
            self.publish_stop_goal()
            self.command_pub.publish(Twist())

    def on_strategy(self, msg):
        if self.core.state == 'PLAYING' and self.localization_valid():
            self.goal_pub.publish(msg)

    def on_command(self, msg):
        if self.core.command_allowed and (self.core.state == 'CALIBRATING' or
                                          self.localization_valid()):
            if all(math.isfinite(x) for v in (msg.linear, msg.angular) for x in (v.x, v.y, v.z)):
                self.command_received = time.monotonic()
                self.command_pub.publish(msg)  # preserve every controller sample

    def publish_stop_goal(self):
        goal = BehaviorGoal()
        goal.mode, goal.action, goal.branch = BehaviorGoal.STOP, 'PreparationStop', 'STOP'
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal)

    def move_goal(self):
        goal = BehaviorGoal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.mode, goal.branch, goal.action = BehaviorGoal.NAVIGATE, 'PREPARATION', 'GoToPose'
        goal.goal_id = self.goal_id
        goal.target = Pose2D(x=self.target[0], y=self.target[1], theta=self.target[2])
        for key, value in self.avoid.items():
            setattr(goal, key, value)
        self.goal_pub.publish(goal)

    def update_led(self, now):
        color, blink = self.core.led
        actual = color if not blink or int(now*2) % 2 == 0 else (0, 0, 0)
        if self.led_future is not None:
            if not self.led_future.done():
                if now-self.led_sent < .3:
                    return
                self.led_future.cancel()
                self.led_status = 'timeout'
            else:
                try:
                    response = self.led_future.result()
                    self.led_status = 'ok' if response and getattr(response, 'success',
                        getattr(response, 'result', False)) else 'failed'
                except Exception:
                    self.led_status = 'failed'
            self.led_future = None
        if actual == self.led_key and now-self.led_sent < 2.:
            return
        self.led_pub.publish(String(data=json.dumps(dict(state=self.core.state,
            color=list(color), blink=blink, actual=list(actual)))))
        if self.led_client.service_is_ready():
            req = SetLed.Request(command='fill', r=actual[0], g=actual[1], b=actual[2])
            self.led_future = self.led_client.call_async(req)
            self.led_status = 'pending'
        elif self.lamp_client.service_is_ready():
            req = SetLamp.Request(mode=1, time=0)
            req.color.r, req.color.g, req.color.b = [v/255. for v in actual]
            req.color.a = 1.
            self.led_future = self.lamp_client.call_async(req)
            self.led_status = 'pending'
        else:
            self.led_status = 'unavailable'
        self.led_key, self.led_sent = actual, now

    def tick(self):
        now = time.monotonic()
        if (self.kickoff_pending is not None and now >= self.kickoff_pending
                and self.core.prepared and self.localization_valid()):
            self.kickoff_pending = None
            self.core.prepared = self.core.arrived = False
            self.core.state = 'MOVING_TO_START'
            self.entered = now
            self.goal_id = time.time_ns()
            self.motion_received = -math.inf
            self.stopped_since = None
        if self.require_referee:
            if self.start_requested is not None:
                if now-self.start_requested > 3. or not self.referee_gate.can_start(self.referee_now()):
                    self.stop()
                    self.core.reason = 'Referee start confirmation failed'
                elif self.referee_reset is not None and self.referee_reset.done():
                    try:
                        reply = self.referee_reset.result()
                    except Exception:
                        reply = None
                    self.referee_reset = None
                    if not reply or not reply.success:
                        self.stop()
                        self.core.reason = 'Referee rejected reset_play'
                elif self.referee_reset is None and self.referee_gate.armed:
                    ok, reason = self.core.start(self.localization_valid())
                    self.start_requested = None
                    if ok and self.start_announce:
                        self.global_playing_pub.publish(Bool(data=True))
                    elif not ok:
                        self.core.reason = reason
            if self.core.state in ('PLAYING', 'MOVING_TO_START') and not self.referee_gate.fresh(self.referee_now()):
                self.stop()
                self.core.reason = 'Referee disconnected or invalid'
        state = self.core.state
        if state == 'CALIBRATING':
            if not self.calibrate_sent and self.calibrate_client.service_is_ready():
                self.calibrate_future = self.calibrate_client.call_async(Trigger.Request())
                self.calibrate_sent = True
            if self.calibrate_future is not None and self.calibrate_future.done():
                try:
                    reply = self.calibrate_future.result()
                    if not reply or not reply.success:
                        self.fail('Calibration service rejected request')
                    else:
                        self.calibration_ack = True
                except Exception as e:
                    self.fail('Calibration service: '+str(e))
                self.calibrate_future = None
            if now-self.entered > self.calibration_timeout:
                self.fail('Preparation calibration timeout')
        elif state in ('MOVING_TO_START', 'READY', 'PLAYING'):
            if not self.localization_valid():
                self.fail('Localization lost or status stale')
            elif state == 'MOVING_TO_START':
                if now-self.entered > self.move_timeout:
                    self.fail('GoToPose / stop confirmation timeout')
                elif self.core.arrived:
                    stopped = (now-self.odom_received < .5 and
                        self.odom_speed[0] <= self.stop_linear and
                        self.odom_speed[1] <= self.stop_angular)
                    self.stopped_since = (self.stopped_since or now) if stopped else None
                    if self.stopped_since and now-self.stopped_since >= self.settle_sec:
                        self.core.stopped_at_start()
                elif now-self.entered > 3 and now-self.motion_received > .7:
                    self.fail('GoToPose result missing or stale')
                else:
                    self.move_goal()
        if not self.core.command_allowed:
            self.command_pub.publish(Twist())
        elif now-self.command_received > .5:
            self.command_pub.publish(Twist())
        if self.core.state != 'PLAYING' and not (
                self.core.state == 'MOVING_TO_START' and not self.core.arrived):
            self.publish_stop_goal()
        self.playing_pub.publish(Bool(data=self.core.state == 'PLAYING'))
        led_started = time.perf_counter()
        self.update_led(now)
        self.led_cost_ms.append(1000*(time.perf_counter()-led_started))
        if now-self.last_status > .2:
            color, blink = self.core.led
            self.status_pub.publish(String(data=json.dumps(dict(
                state=self.core.state, ready=self.core.prepared,
                stamp_ros=self.get_clock().now().nanoseconds*1e-9,
                reason=self.core.reason, team=self.core.team, start_pose=self.target,
                start_pending=self.start_requested is not None,
                referee_start_allowed=(not self.require_referee or self.referee_gate.can_start(self.referee_now())),
                own_goal=self.field.own_goal if self.core.team == self.field.our_goal_color
                         else self.field.opp_goal,
                localization=self.loc, localization_valid=self.localization_valid(),
                arrived=self.core.arrived, odom_fresh=now-self.odom_received < .5,
                sensors=dict(lidar=self.loc.get('sensors', {}).get('lidar_outer_walls', False),
                    imu=self.loc.get('sensors', {}).get('imu', False),
                    odom=now-self.odom_received < .5, camera=now-self.camera_received < 2.),
                led=dict(color=color, blink=blink, service=self.led_status),
                led_callback_ms=dict(mean=sum(self.led_cost_ms)/len(self.led_cost_ms),
                    p95=sorted(self.led_cost_ms)[int(.95*(len(self.led_cost_ms)-1))],
                    maximum=max(self.led_cost_ms)),
                goal_id=self.goal_id))))
            self.last_status = now


def main(args=None):
    rclpy.init(args=args)
    node = MatchPreparation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.command_pub.publish(Twist())
            node.publish_stop_goal()
        node.destroy_node()
        rclpy.try_shutdown()
