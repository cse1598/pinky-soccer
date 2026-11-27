"""Calibrated geometric path following for the attack-only match."""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from pinky_soccer_dynamics import params as dynamics
from pinky_soccer_dynamics.command_adapter import CommandAdapter, wheels_from_joint_state
from pinky_soccer_dynamics.profile_model import motor_step, step
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_mpc.collision_geometry import FOOTPRINT_RECTS, footprint_polygons
from pinky_soccer_mpc.response_calibration import apply_response
from pinky_soccer_msgs.msg import BehaviorGoal, MotionResult, WorldModel
from .attack_motion import Follower, Grid, wrap, contact_speed_limit


class AttackController(Node):
    def __init__(self):
        super().__init__('attack_controller')
        p = self.declare_parameter
        target = p('target', 'real').value
        self.params, self.calibration = apply_response(dynamics.load(target,
            profile=p('calibration_profile', '').value or None), p('identification', '').value)
        self.adapter = CommandAdapter(self.params)
        self.field = Field.from_file(p('field_spec', '').value or None)
        self.position_tol = float(p('position_tolerance', .035).value)
        self.heading_tol = float(p('heading_tolerance', .09).value)
        self.margin = float(p('obstacle_margin', .015).value)
        self.body_radius = max(math.hypot(x, y) for a, b, c, d in FOOTPRINT_RECTS
                               for x, y in ((a,b), (c,d), (a,d), (c,b)))
        self.follower = Follower(self.params, self.position_tol, self.heading_tol)
        self.path_mpc = None
        if bool(p('path_mpc_approach', False).value):
            if target != 'mujoco':
                raise ValueError('path_mpc approach is experimental and MuJoCo only')
            from .path_mpc_approach import PathMpcApproach
            self.path_mpc = PathMpcApproach(self.params, self.field, self.margin,
                float(p('path_mpc_max_lin', 0.).value), float(p('path_mpc_max_ang', 2.).value))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.goal = self.world = self.scan = self.wheels = None
        self.goal_at = self.world_at = self.scan_at = self.wheel_at = -math.inf
        self.scan_cache = None
        self.scan_cache_stamp = -math.inf
        self.tick_started = time.perf_counter()
        self.key = None
        self.path, self.path_target = [], None
        self.planned_at = -math.inf
        self.profile = np.zeros(2)
        self.last_tick = None
        self.entered = None
        self.travel_reverse = None
        self.finished = False
        self.failed = None
        self.progress_pose = None
        self.progress_at = 0.
        self.stuck_since = None
        self.recovery_until = 0.
        self.recovery_command = None
        self.recovery_attempts = 0
        self.block_reason = ''
        self.last_progress_warning = -math.inf
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'motion/path', 10)
        self.result = self.create_publisher(MotionResult, 'motion/result', 10)
        self.status = self.create_publisher(String, 'motion/status', 10)
        self.create_subscription(BehaviorGoal, 'behavior_goal', self.on_goal, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_subscription(JointState, 'joint_states', self.on_wheels, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_goal(self, msg):
        self.goal, self.goal_at = msg, self.now()

    def on_world(self, msg):
        self.world, self.world_at = msg, self.now()

    def on_wheels(self, msg):
        try:
            u = np.asarray(wheels_from_joint_state(self.params, msg.name, msg.velocity))
            if np.isfinite(u).all():
                self.wheels, self.wheel_at = u, self.now()
        except (ValueError, KeyError, IndexError):
            pass

    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = self.now()

    def send(self, command):
        v, w = self.adapter.to_twist(*command)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)
        return v, w

    def report(self, state, reason, result=MotionResult.RUNNING):
        self.status.publish(String(data=json.dumps(dict(state=state, reason=reason,
            backend='geometric_path', phase=self.follower.phase,
            goal_id=str(self.goal.goal_id) if self.goal else '0',
            action=self.goal.action if self.goal else '',
            stalled_s=max(0., self.now()-self.progress_at) if self.progress_pose is not None else 0.,
            recovery_attempts=self.recovery_attempts, block_reason=self.block_reason,
            processing_ms=1000*(time.perf_counter()-self.tick_started),
            scan_age_s=self.now()-self.scan_cache_stamp if self.scan_cache is not None else None))))
        if self.goal and self.goal.goal_id:
            msg = MotionResult()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.goal_id, msg.state, msg.reason = self.goal.goal_id, result, reason
            if self.world:
                pose = self.world.self.pose
                msg.position_error = math.hypot(self.goal.target.x-pose.x, self.goal.target.y-pose.y)
                msg.heading_error = abs(wrap(self.goal.target.theta-pose.theta))
            self.result.publish(msg)

    def stop(self, reason, failure=False):
        if self.path_mpc is not None:
            self.path_mpc.reset()
        self.send((0., 0.))
        if self.wheels is not None:
            _, self.profile = motor_step(self.params, self.wheels, self.profile, np.zeros(2), .02)
        self.follower.previous[:] = 0
        self.path = []
        if failure:
            self.failed = reason
        self.report('failed' if failure else 'stop', reason,
                    MotionResult.FAILURE if failure else MotionResult.RUNNING)

    def scan_points(self):
        msg = self.scan
        transform = self.tf.lookup_transform('map', msg.header.frame_id,
                                             Time.from_msg(msg.header.stamp))
        q, t = transform.transform.rotation, transform.transform.translation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        r = np.asarray(msg.ranges)
        a = msg.angle_min+np.arange(len(r))*msg.angle_increment+yaw
        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        if not valid.any():
            raise ValueError('no valid LiDAR ranges')
        return np.column_stack((t.x+r[valid]*np.cos(a[valid]), t.y+r[valid]*np.sin(a[valid])))

    def obstacles(self, points, pushing):
        # The horizontal LiDAR sits above the golf ball. A return next to a
        # tracked ball can be the opponent's bumper; never erase that return.
        return points

    def static_clearance(self, pose):
        walls = self.field.inner_walls
        half = self.field.spec['goal_width']/2+self.field.spec.get('goal_thickness', .005)
        result = math.inf
        for poly in footprint_polygons(pose):
            lo, hi = poly.min(axis=0), poly.max(axis=0)
            result = min(result, lo[0]-walls.xmin, walls.xmax-hi[0],
                         lo[1]-walls.ymin, walls.ymax-hi[1])
            for x0,x1 in ((walls.xmin,-self.field.half_length),
                          (self.field.half_length,walls.xmax)):
                dx = max(x0-hi[0], lo[0]-x1)
                dy = max(-half-hi[1], lo[1]-half)
                gap = math.hypot(max(dx,0.),max(dy,0.)) if max(dx,dy)>0 else max(dx,dy)
                result = min(result,gap)
        return result

    def point_clearance(self, pose, points):
        if not len(points):
            return math.inf
        delta = points-pose[:2]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        local = np.column_stack((c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1]))
        result = math.inf
        for a,b,c,d in FOOTPRINT_RECTS:
            dx = np.maximum(a-local[:,0], local[:,0]-c)
            dy = np.maximum(b-local[:,1], local[:,1]-d)
            gap = np.hypot(np.maximum(dx,0.), np.maximum(dy,0.)) + np.minimum(np.maximum(dx,dy),0.)
            result = min(result, float(gap.min()))
        return result

    def recover(self, pose, points, dt, reason):
        if self.path_mpc is not None:
            self.path_mpc.reset()
        now = self.now()
        self.block_reason = reason
        # Progress during an active escape is its intended effect. Do not apply
        # the *entry* stall threshold again and cancel it after the first 8 mm.
        if now-self.progress_at < 1. and now >= self.recovery_until:
            self.stop(reason)
            return
        if now-self.last_progress_warning >= 1.:
            self.last_progress_warning = now
            self.get_logger().warning('No motion progress >=1s: '+reason+
                '; recovery attempt '+str(self.recovery_attempts+1))
        if now >= self.recovery_until:
            self.recovery_attempts += 1
            # Independent escape motion: do not require a route to the ball.
            # Alternate turn preference after an unsuccessful escape.
            sign = (1. if self.recovery_attempts % 2 else -1.) * (1. if self.world.our_goal_color == 'yellow' else -1.)
            candidates = [(v,w) for v in (-.12,.12,0.) for w in (0.,sign*1.2,-sign*1.2)]
            choices = []
            previous = self.follower.previous.copy()
            for v,w in candidates:
                self.follower.previous = previous.copy()
                u = self.follower.wheel_command(v,w,.5)
                if self.safe(pose,u,points):
                    x = np.array([*pose,*self.wheels,*self.profile,*previous])
                    initial_clear = min(self.static_clearance(pose),self.point_clearance(pose,points))
                    valid = True
                    for _ in range(20):
                        x = step(self.params,x,u,.02)
                        clear = min(self.static_clearance(x[:3]),self.point_clearance(x[:3],points))
                        if clear < min(self.margin,initial_clear)-1e-5:
                            valid = False; break
                    if valid:
                        displacement = np.linalg.norm(x[:2]-pose[:2])
                        # Escape is judged by increased free space, not just motion.
                        score = 2*clear + displacement - .05*np.linalg.norm(x[:2])
                        choices.append((score, (v,w)))
            self.follower.previous = previous
            if not choices:
                self.recovery_until = now+.1
                self.recovery_command = None
            else:
                self.recovery_command = max(choices,key=lambda x:x[0])[1]
                self.recovery_until = now+.6
        if self.recovery_command is None:
            return self.stop(reason+'; no physically clear escape')
        command = self.follower.wheel_command(*self.recovery_command,dt)
        if not self.safe(pose,command,points):
            self.recovery_until = 0.
            return self.stop(reason+'; escape blocked, selecting another')
        self.send(command)
        self.follower.previous = command.copy()
        _,self.profile = motor_step(self.params,self.wheels,self.profile,command,dt)
        self.path = []
        self.follower.phase = 'TRAVEL'
        self.follower.done = False
        self.report('recover',reason+'; independent escape before replanning')

    def safe(self, pose, command, points):
        # If already closer than the desired margin, permit only trajectories
        # which do not reduce that clearance. Never allow physical penetration.
        initial = self.static_clearance(pose)
        if initial < 0:
            return False
        required = min(self.margin, initial)
        point_required = min(self.margin, self.point_clearance(pose,points))
        x = np.array([*pose, *self.wheels, *self.profile, *self.follower.previous])
        predicted = [x[:3].copy()] if self.path_mpc is not None else None
        for i in range(25):
            x = step(self.params, x, command if i < 5 else np.zeros(2), .02)
            if self.static_clearance(x[:3])+1e-9 < required:
                return False
            if self.point_clearance(x[:3],points)+1e-9 < point_required:
                return False
            if predicted is not None:
                predicted.append(x[:3].copy())
        if predicted is not None and self.world is not None:
            if not self.path_mpc.opponent_safe(self.world, np.asarray(predicted), .02):
                return False
        return True

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x,y in self.path:
            p = PoseStamped(); p.header = msg.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.w = 1.
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def tick(self):
        self.tick_started = time.perf_counter()
        now = self.get_clock().now().nanoseconds/1e9
        wall = time.monotonic()
        dt = .02 if self.last_tick is None else now-self.last_tick
        self.last_tick = now
        if not 0 < dt < .2:
            return self.stop('clock discontinuity')
        g = self.goal
        if g is None or now-self.goal_at > .5 or g.mode == BehaviorGoal.STOP:
            self.key = None
            self.progress_pose = None
            self.recovery_until = 0.
            return self.stop('stopped or no fresh goal')
        key = (g.action, g.goal_id)
        if key != self.key:
            if self.path_mpc is not None:
                self.path_mpc.reset()
            if self.progress_pose is None or g.goal_id:
                self.progress_pose = None
                self.recovery_until = 0.
            self.key, self.entered = key, now
            self.follower.reset(); self.path = []; self.path_target = None
            self.finished, self.failed = False, None
            self.travel_reverse = None
        if self.failed:
            return self.stop(self.failed, True)
        if self.finished:
            self.send((0., 0.))
            return self.report('arrived', 'completion latched', MotionResult.SUCCESS)
        if self.world is None or now-self.world_at > .3 or self.world.self.state == self.world.self.LOST or self.world.self.confidence <= 0:
            return self.stop('localization unavailable')
        if self.wheels is None or now-self.wheel_at > .2 or self.scan is None or now-self.scan_at > .5:
            return self.stop('wheel feedback or LiDAR unavailable')
        if g.goal_id and now-self.entered > 30:
            return self.stop('motion timeout', True)
        pose = np.array([self.world.self.pose.x, self.world.self.pose.y, self.world.self.pose.theta])
        target = np.array([g.target.x, g.target.y, g.target.theta])
        if not np.isfinite([*pose, *target]).all():
            return self.stop('non-finite pose/target', True)
        measured_v = (self.params.m('left_radius')*self.wheels[0]+self.params.m('right_radius')*self.wheels[1])/2
        measured_w = (self.params.m('right_radius')*self.wheels[1]-self.params.m('left_radius')*self.wheels[0])/self.params.m('track')
        wheel_progress = abs(measured_v) > .012 or abs(measured_w) > .08
        if self.progress_pose is None or (wheel_progress and
                (np.linalg.norm(pose[:2]-self.progress_pose[:2]) > .008 or
                 abs(wrap(pose[2]-self.progress_pose[2])) > .06)):
            self.progress_pose, self.progress_at = pose.copy(), now
        pushing = g.action == 'PushBall'
        try:
            self.scan_cache = self.scan_points()
            self.scan_cache_stamp = self.scan.header.stamp.sec+self.scan.header.stamp.nanosec/1e9
        except Exception as exc:
            if self.scan_cache is None or not 0 <= now-self.scan_cache_stamp <= .2:
                return self.stop('scan transform unavailable: '+type(exc).__name__)
        points = self.obstacles(self.scan_cache.copy(), pushing)
        if now < self.recovery_until and self.recovery_command is not None:
            return self.recover(pose,points,dt,self.block_reason)
        mpc_approach = self.path_mpc is not None and g.action == 'GoBehindBall'
        if now-self.progress_at >= (3. if mpc_approach else 1.) and (self.follower.phase == 'TRAVEL' or not g.goal_id):
            return self.recover(pose,points,dt,self.block_reason or 'motion command without progress')
        if g.action == 'AlignToBall':
            command = self.follower.command(pose,target,target[:2],self.wheels,now,dt)
        elif g.action == 'SpinInPlace':
            command = self.follower.wheel_command(0., 1.2, dt)
        else:
            radius = self.body_radius+self.margin
            bounds = self.field.inner_walls
            blocks = [] if pushing or g.action == 'Backoff' else [(s*self.field.half_length if s>0 else bounds.xmin,
                -self.field.spec['goal_width']/2, bounds.xmax if s>0 else -self.field.half_length,
                self.field.spec['goal_width']/2) for s in (-1,1)]
            grid = Grid((bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax), radius, blocks)
            # Wall/goal returns use known geometry; internal returns stay obstacles.
            internal = points[(np.abs(points[:, 0]) < self.field.half_length-.03) &
                              (np.abs(points[:, 1]) < bounds.ymax-.04)]
            if self.world.ball.state != self.world.ball.LOST:
                ball = np.array([self.world.ball.pose.x, self.world.ball.pose.y])
                if g.action == 'GoBehindBall':
                    # The forward bumper is shorter than the robot's covering
                    # disc. Inflating the ball by that entire disc makes the
                    # near-wall behind-ball pose impossible by construction.
                    front = max(c for a,b,c,d in FOOTPRINT_RECTS)
                    axis = np.array([math.cos(target[2]),math.sin(target[2])])
                    aligned = abs(wrap(pose[2]-target[2])) < .3 and np.dot(pose[:2]-ball,axis) < 0
                    clearance = front+.02135+.01 if aligned else radius+.02135
                    grid.circles = [(ball[0],ball[1],clearance)]
                elif g.goal_id:
                    internal = np.vstack((internal, ball))
            if g.goal_id and g.branch == 'PREPARATION':
                # Keep the centre restart spot free while people replace the ball.
                ring = np.linspace(0, 2*math.pi, 12, endpoint=False)
                internal = np.vstack((internal, np.column_stack((.025*np.cos(ring), .025*np.sin(ring)))))
            grid.points = internal
            # An occupied goal is transient, not a failed request. Keep ownership
            # and retry planning as observations change, just like a blocked route.
            target[:2] = np.clip(target[:2], [bounds.xmin+radius, bounds.ymin+radius],
                                [bounds.xmax-radius, bounds.ymax-radius])
            if not g.goal_id and not grid.clear(target[:2]):
                # Ball-behind targets can lie in a wall/goal or an opponent.
                # Project only streaming tactical targets, never fixed GoToPose success.
                candidates = [target[:2]+distance*np.array([math.cos(a),math.sin(a)])
                    for distance in (.04,.08,.12,.16,.20,.24,.28)
                    for a in np.linspace(0,2*math.pi,16,endpoint=False)]
                free = [q for q in candidates if grid.clear(q)]
                if free:
                    target[:2] = min(free,key=lambda q:np.linalg.norm(q-target[:2])+.1*np.linalg.norm(q-pose[:2]))
            changed = self.path_target is None or np.linalg.norm(target[:2]-self.path_target[:2]) > .06
            if not g.goal_id and changed:
                previous = self.follower.previous.copy()
                self.follower.reset()
                self.follower.previous = previous
            if self.follower.phase == 'TRAVEL' and (not self.path or changed or
                    (wall-self.planned_at > .2 and not grid.line(pose[:2], self.path[min(1,len(self.path)-1)]))):
                self.path = grid.plan(pose[:2], target[:2])
                if not self.path and not grid.clear(pose[:2]):
                    # A disc can overestimate the narrow front near a goal. Leave
                    # along the current body axis; exact footprint replay below
                    # remains authoritative for this first, straight escape leg.
                    axis = np.array([math.cos(pose[2]), math.sin(pose[2])])
                    escapes = [pose[:2]+direction*distance*axis
                        for distance in (.08, .16, .24) for direction in (-1., 1.)]
                    escapes.sort(key=lambda q:np.linalg.norm(q-pose[:2])+np.linalg.norm(q-target[:2]))
                    for escape in escapes:
                        route = grid.plan(escape, target[:2])
                        if route:
                            self.path = [pose[:2].copy(), *route]
                            break
                self.path_target, self.planned_at = target.copy(), wall
                self.publish_path()
            if not self.path and self.follower.phase == 'TRAVEL':
                return self.recover(pose,points,dt,'no collision-free path; start_clear='+str(grid.clear(pose[:2]))+' target_clear='+str(grid.clear(target[:2])))
            while len(self.path)>2 and np.linalg.norm(pose[:2]-self.path[1]) < .055:
                self.path.pop(0)
            waypoint = self.path[min(1, len(self.path)-1)] if self.path else target[:2]
            if self.travel_reverse is None:
                bearing = math.atan2(waypoint[1]-pose[1], waypoint[0]-pose[0])
                self.travel_reverse = (g.action == 'Backoff' or
                    (bool(g.goal_id) and abs(wrap(bearing-pose[2])) > math.pi/2))
            self.block_reason = ''
            speed_cap = g.max_speed
            if g.action in ('PushBall','GoBehindBall') and self.world.ball.state != self.world.ball.LOST:
                b = self.world.ball
                maximum = min(self.params.m('left_radius'),self.params.m('right_radius'))*self.params.wheel_cmd_max()
                impact_limit = contact_speed_limit(pose,(b.pose.x,b.pose.y),(b.velocity.x,b.velocity.y),maximum)
                speed_cap = min(speed_cap,impact_limit) if speed_cap>0 else impact_limit
            if mpc_approach:
                route = [pose[:2].copy(), *self.path[1:]] if self.path else []
                if len(route) < 2 or not grid.line(route[0], route[1]):
                    self.path = []
                    self.path_mpc.reset()
                    command = np.zeros(2)
                else:
                    command = self.path_mpc.command(now, wall, pose, self.wheels, self.profile,
                        self.follower.previous, target, route, self.world, speed_cap)
            else:
                command = self.follower.command(pose, target, waypoint, self.wheels, now,
                    dt, streaming=pushing, speed_cap=speed_cap, reverse=self.travel_reverse)
        safe = False
        for scale in (1., .5, .0):
            candidate = command*scale
            if self.safe(pose, candidate, points):
                command, safe = candidate, True
                break
        if not safe:
            return self.recover(pose,points,dt,'obstacle in stopping envelope')
        self.follower.previous = command.copy()
        _, self.profile = motor_step(self.params, self.wheels, self.profile, command, dt)
        self.send(command)
        if self.follower.done:
            # Never keep correcting the same accepted goal; the BT owns the next action.
            self.finished = bool(g.goal_id)
            self.report('arrived', 'position/heading accepted; stopped', MotionResult.SUCCESS)
        else:
            self.report('track', self.path_mpc.status if mpc_approach else 'following collision-checked path')


def main(args=None):
    rclpy.init(args=args)
    node = AttackController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send((0., 0.))
        if node.path_mpc is not None:
            node.path_mpc.close()
        node.destroy_node(); rclpy.try_shutdown()
