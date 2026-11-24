"""
Two Pinky robots in one MuJoCo world (wrapper around the official single-robot bridge).

The official pinky_mujoco Physics/Bridge drive one robot (fixed names, ctrl[:] of two
actuators, pose from the first free joint). Without touching the submodule this module:

  * places robot 1 at its spawn pose and duplicates its body subtree, actuators and sensors
    with the prefix `opp_` for robot 2. Robot collision geoms are fully transparent in the
    official model and mj_ray ignores alpha-0 geoms, so no ray could ever hit a robot. With two
    robots the collision geoms get alpha 0.01 and their own group (robot 1: 5, robot 2: 3):
    each robot's LiDAR/range mask includes the other robot's group but not its own. Groups
    3 and 5 are not rendered by cameras or the viewer.
  * replaces the base Physics.step with a joint step of both robots (same velocity ramp,
    real-driver scaling and encoder odometry as robot 1),
  * publishes robot 2 from the same bridge node under absolute /<opponent>/ topics:
    drive/cmd_vel (in), odom, tf, joint_states, ground_truth and, with sensors enabled,
    sim/lidar/scan, sim/camera/image_raw + camera_info, sim/imu, sim/<range>/scan.
The material contact kernel is extended to robot 2's wheels (install_after).
"""

import copy
import math
import time
import xml.etree.ElementTree as ET

import numpy as np

PREFIX = 'opp_'
ROBOT1_GROUP, ROBOT2_GROUP = '5', '3'     # collision groups when two robots are present
RAY_ALPHA = '0.01'                       # > 0 so mj_ray hits them; still invisible


def yaw_quat(yaw):
    """Return the MuJoCo quaternion (w x y z) of a yaw angle."""
    return f'{math.cos(yaw / 2):.9g} 0 0 {math.sin(yaw / 2):.9g}'


def place_robots(scene_path, ours=None, opponent=None):
    """Set robot 1's spawn pose and add robot 2 (prefixed copy) to a built scene."""
    tree = ET.parse(scene_path)
    root = tree.getroot()
    world = root.find('worldbody')
    base = world.find("body[@name='base_link']")
    if base is None:
        raise ValueError('scene has no top-level base_link body')
    z = base.get('pos', '0 0 0.03').split()[2]
    if ours is not None:
        base.set('pos', f'{ours[0]:.9g} {ours[1]:.9g} {z}')
        base.set('quat', yaw_quat(ours[2]))
    if opponent is not None:
        twin = copy.deepcopy(base)
        names = set()
        for robot, group in ((base, ROBOT1_GROUP), (twin, ROBOT2_GROUP)):
            for el in robot.iter('geom'):
                if el.get('group') == '1':
                    el.set('group', group)
                    rgba = el.get('rgba', '0.4 0.4 0.4 0').split()
                    el.set('rgba', ' '.join(rgba[:3] + [RAY_ALPHA]))
        for el in twin.iter():
            if 'name' in el.attrib:
                names.add(el.get('name'))
                el.set('name', PREFIX + el.get('name'))
        twin.set('pos', f'{opponent[0]:.9g} {opponent[1]:.9g} {z}')
        twin.set('quat', yaw_quat(opponent[2]))
        world.append(twin)
        actuators = root.find('actuator')
        for act in list(actuators):
            dup = copy.deepcopy(act)
            dup.set('name', PREFIX + act.get('name'))
            dup.set('joint', PREFIX + act.get('joint'))
            actuators.append(dup)
        sensors = root.find('sensor')
        for sensor in list(sensors) if sensors is not None else []:
            refs = [k for k in ('site', 'objname') if sensor.get(k) in names]
            if refs:
                dup = copy.deepcopy(sensor)
                dup.set('name', PREFIX + sensor.get('name'))
                for k in refs:
                    dup.set(k, PREFIX + sensor.get(k))
                sensors.append(dup)
        contact = root.find('contact')
        for item in list(contact) if contact is not None else []:
            refs = [k for k in ('geom1', 'geom2', 'body1', 'body2') if item.get(k) in names]
            if refs:
                dup = copy.deepcopy(item)
                for k in refs:
                    dup.set(k, PREFIX + item.get(k))
                if 'name' in dup.attrib:
                    dup.set('name', PREFIX + dup.get('name'))
                contact.append(dup)
    tree.write(scene_path, encoding='unicode')
    return scene_path


def driver_scale(left, right, max_speed):
    """Scale both wheels like the real driver so the faster one stays within max_speed."""
    peak = max(abs(left), abs(right))
    return max_speed / peak if max_speed and peak > max_speed else 1.0


class Robot2:
    """Indices and odometry state of the duplicated robot."""

    def __init__(self, model):
        self.wheel_q = [model.joint(f'{PREFIX}{s}_wheel_joint').qposadr[0] for s in 'lr']
        self.wheel_v = [model.joint(f'{PREFIX}{s}_wheel_joint').dofadr[0] for s in 'lr']
        self.act = [model.actuator(f'{PREFIX}{s}_motor').id for s in 'lr']
        free = model.joint(f'{PREFIX}floating_base')
        self.qpos = free.qposadr[0]
        self.lidar_id = model.site(f'{PREFIX}lidar').id
        self.joint_names = ['l_wheel_joint', 'r_wheel_joint', 'caster_rotate_joint',
                            'caster_wheel_joint']
        self.q_indices = [model.joint(PREFIX + n).qposadr[0] for n in self.joint_names]
        self.v_indices = [model.joint(PREFIX + n).dofadr[0] for n in self.joint_names]
        self.odom = np.zeros(3)
        self.twist = np.zeros(2)
        self.command = (0.0, 0.0)
        self.command_time = -math.inf

    def pose(self, data):
        """Return the true planar x, y and yaw from the free joint."""
        q = data.qpos[self.qpos:self.qpos + 7]
        w, x, y, z = q[3:7]
        return np.array([q[0], q[1], math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])


def install_base(bridge, max_rpm=0.0):
    """Insert the two-robot step right above the official Physics (install first)."""
    official = bridge.Physics

    class MultiPhysics(official):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.act = [self.model.actuator(f'{s}_motor').id for s in 'lr']
            try:
                self.model.body(PREFIX + 'base_link')
            except KeyError:
                self.robot2 = None
                return
            self.robot2 = Robot2(self.model)
            # Each robot sees the field (groups 0 and 4) and the other robot's collisions.
            self.ray_group = np.array([1, 0, 0, 1, 1, 0], dtype=np.uint8)
            self.ray_group2 = np.array([1, 0, 0, 0, 1, 1], dtype=np.uint8)

        def step(self, linear=0.0, angular=0.0, steps=None):
            if self.robot2 is None:
                return super().step(linear, angular, steps)
            steps = steps or round(self.bridge_period / self.model.opt.timestep)
            motors = self.settings['motors']
            r2 = self.robot2
            t1 = np.clip([(linear - angular * self.track / 2) / self.radius,
                          (linear + angular * self.track / 2) / self.radius],
                         -motors['max_speed'], motors['max_speed'])
            timeout = self.settings['control']['command_timeout']
            fresh = time.monotonic() - r2.command_time < timeout
            v, w = r2.command if fresh else (0.0, 0.0)
            half = w * self.track / 2
            wheels = [(v - half) / self.radius, (v + half) / self.radius]
            k = driver_scale(*wheels, max_rpm * 2 * math.pi / 60) if max_rpm else 1.0
            t2 = np.clip(np.array(wheels) * k, -motors['max_speed'], motors['max_speed'])
            before1 = self.data.qpos[self.wheel_q].copy()
            before2 = self.data.qpos[r2.wheel_q].copy()
            ramp = motors.get('wheel_accel')
            limit = ramp * self.model.opt.timestep if ramp else np.inf
            for _ in range(steps):
                for ids, target in ((self.act, t1), (r2.act, t2)):
                    ctrl = self.data.ctrl[ids]
                    self.data.ctrl[ids] = ctrl + np.clip(target - ctrl, -limit, limit)
                self.mj.mj_step(self.model, self.data)
            dt = steps * self.model.opt.timestep
            for before, q, odom, twist in ((before1, self.wheel_q, self.odom, self.twist),
                                           (before2, r2.wheel_q, r2.odom, r2.twist)):
                dl, dr = (self.data.qpos[q] - before) * self.radius
                distance, angle = (dl + dr) / 2, (dr - dl) / self.track
                theta = odom[2] + angle / 2
                odom += [distance * math.cos(theta), distance * math.sin(theta), angle]
                twist[:] = [distance / dt, angle / dt]

        def scan2(self):
            """Return the LiDAR ranges of robot 2 (same model as robot 1)."""
            r2 = self.robot2
            self.mj.mj_forward(self.model, self.data)
            rotation = self.data.site_xmat[r2.lidar_id].reshape(3, 3)
            rays = np.ascontiguousarray(self.ray_vectors @ rotation.T)
            distances = np.empty(len(rays), dtype=np.float64)
            geom_ids = np.empty(len(rays), dtype=np.int32)
            lidar = self.settings['lidar']
            self.mj.mj_multiRay(self.model, self.data, self.data.site_xpos[r2.lidar_id],
                                rays.ravel(), self.ray_group2, True, -1, geom_ids,
                                distances, None, len(rays), lidar['range_max'])
            distances[(distances < lidar['range_min']) | (distances > lidar['range_max'])] = np.inf
            return distances

    bridge.Physics = MultiPhysics


def install_after(bridge, material_cfg=None):
    """Extend the material contact kernel to robot 2's wheels (install last)."""
    base = bridge.Physics

    class MultiMaterialPhysics(base):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            if getattr(self, 'robot2', None) is None or not hasattr(self, 'material_contacts'):
                return
            from pinky_soccer_sim.materials import parameters
            for name, pair in [('l_wheel', 'wheel_floor'), ('r_wheel', 'wheel_floor'),
                               ('caster_wheel', 'caster_floor')]:
                body = self.model.body(PREFIX + name)
                geom = self.model.geom(f'{PREFIX}{name}_collision_0')
                self.material_contacts.append((body.id, geom.id,
                                               parameters(material_cfg, pair, None)))

    bridge.Physics = MultiMaterialPhysics


def install_bridge(bridge, opponent, sensors, sensor_cfg=None, imu=False, adc=False):
    """Publish robot 2 from the same bridge node (absolute /<opponent>/ topics)."""
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from tf2_msgs.msg import TFMessage
    base = bridge.Bridge
    ns = f'/{opponent}'

    class MultiBridge(base):
        def __init__(self, physics):
            super().__init__(physics)
            if physics.robot2 is None:
                return
            r2 = physics.robot2

            def on_cmd(msg):
                if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
                    r2.command, r2.command_time = (msg.linear.x, msg.angular.z), time.monotonic()
            self.create_subscription(Twist, f'{ns}/drive/cmd_vel', on_cmd, 10)
            self.r2_odom = self.create_publisher(Odometry, f'{ns}/odom', 10)
            self.r2_tf = self.create_publisher(TFMessage, f'{ns}/tf', 10)
            self.r2_joints = self.create_publisher(JointState, f'{ns}/joint_states', 10)
            self.r2_truth = self.create_publisher(PoseStamped, f'{ns}/ground_truth', 10)
            self.r2_sensors = sensors
            if sensors:
                self.r2_scan = self.create_publisher(LaserScan, f'{ns}/sim/lidar/scan',
                                                     qos_profile_sensor_data)
                self.r2_camera = Camera2(self, physics, ns) if physics.settings.get(
                    'camera', {}).get('enabled') else None
                self.r2_imu = self.create_publisher(Imu, f'{ns}/sim/imu',
                                                    qos_profile_sensor_data) if imu else None
                from pinky_soccer_sim.sensor_model import RANGE_LINKS
                self.r2_ranges = {n: self.create_publisher(LaserScan, f'{ns}/sim/{n}/scan',
                                                           qos_profile_sensor_data)
                                  for n in RANGE_LINKS} if adc else {}
            self.get_logger().info(f'second robot {opponent}: topics under {ns}/, '
                                   f'sensors {"on" if sensors else "off (obstacle)"}')

        def advance(self):
            super().advance()
            if self.physics.robot2 is None:
                return
            now = bridge.stamp(self.physics.data.time)
            if self.tick % self.state_ticks == 0:
                self.publish_robot2(now)
            if self.r2_sensors:
                if self.tick % self.scan_ticks == 0:
                    self.publish_scan2(now)
                if self.r2_camera and self.r2_camera.due(self.physics.data.time):
                    self.r2_camera.publish(self.physics.data, now)
                if self.r2_imu is not None:
                    self.publish_imu2(now)
                if self.r2_ranges and self.tick % 5 == 0:
                    self.publish_ranges2(now)

        def publish_robot2(self, now):
            p, r2 = self.physics, self.physics.robot2
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = now, 'odom'
            odom.child_frame_id = 'base_footprint'
            bridge.set_planar_pose(odom.pose.pose, r2.odom)
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = map(float, r2.twist)
            for i, variance in enumerate([0.0025, 0.0025, 1e6, 1e6, 1e6, 0.01]):
                odom.pose.covariance[7 * i] = variance
                odom.twist.covariance[7 * i] = variance
            self.r2_odom.publish(odom)
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, odom.child_frame_id
            tf.transform.translation.x = odom.pose.pose.position.x
            tf.transform.translation.y = odom.pose.pose.position.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.r2_tf.publish(TFMessage(transforms=[tf]))
            joints = JointState()
            joints.header.stamp = now
            joints.name = r2.joint_names
            joints.position = p.data.qpos[r2.q_indices].tolist()
            joints.velocity = p.data.qvel[r2.v_indices].tolist()
            self.r2_joints.publish(joints)
            truth = PoseStamped()
            truth.header.stamp, truth.header.frame_id = now, 'world'
            bridge.set_planar_pose(truth.pose, r2.pose(p.data))
            self.r2_truth.publish(truth)

        def publish_scan2(self, now):
            p = self.physics
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now, 'rplidar_link'
            scan.angle_min, scan.angle_max = float(p.angles[0]), float(p.angles[-1])
            scan.angle_increment = float(p.angles[1] - p.angles[0])
            scan.time_increment, scan.scan_time = 0.0, 1 / p.settings['rates']['scan_hz']
            scan.range_min = float(p.settings['lidar']['range_min'])
            scan.range_max = float(p.settings['lidar']['range_max'])
            scan.ranges = p.scan2().astype('float32').tolist()
            self.r2_scan.publish(scan)

        def publish_imu2(self, now):
            data = self.physics.data
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = now, 'imu_link'
            w, x, y, z = map(float, data.sensor(f'{PREFIX}soccer_quat').data)
            msg.orientation.w, msg.orientation.x = w, x
            msg.orientation.y, msg.orientation.z = y, z
            for field, name in [('angular_velocity', 'gyro'), ('linear_acceleration', 'acc')]:
                vector = getattr(msg, field)
                values = data.sensor(f'{PREFIX}soccer_{name}').data
                vector.x, vector.y, vector.z = map(float, values)
            self.r2_imu.publish(msg)

        def publish_ranges2(self, now):
            for name, publisher in self.r2_ranges.items():
                item = sensor_cfg['ultrasonic' if name == 'ultrasonic_link' else 'ir']
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = now, name
                scan.range_min, scan.range_max = item['min_range'], item['max_range']
                scan.ranges = ray_ranges2(self.physics, name, item)
                publisher.publish(scan)

    bridge.Bridge = MultiBridge


def ray_ranges2(physics, link, item):
    """Range sensor rays of robot 2 (same model as sensor_model.ray_ranges for robot 1)."""
    angles = (np.linspace(-item['field_of_view'] / 2, item['field_of_view'] / 2, item['samples'])
              if link == 'ultrasonic_link' else [0.0])
    site = physics.model.site(f'{PREFIX}soccer_{link}').id
    rotation = physics.data.site_xmat[site].reshape(3, 3)
    group = np.array([1, 1, 0, 0, 1, 1], dtype=np.uint8)   # field, ball and robot 1
    out = []
    for angle in angles:
        direction = rotation @ np.array([math.cos(angle), math.sin(angle), 0.0])
        d = physics.mj.mj_ray(physics.model, physics.data, physics.data.site_xpos[site],
                              direction, group, True, -1, np.zeros(1, dtype=np.int32))
        out.append(float('inf') if d < 0 or d > item['max_range']
                   else float('-inf') if d < item['min_range'] else float(d))
    return out


class Camera2:
    """Offscreen render of robot 2's front camera (same settings as robot 1's)."""

    def __init__(self, node, physics, ns):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image
        self.Image = Image
        self.cfg = physics.settings['camera']
        self.model = physics.model
        self.width, self.height = int(self.cfg['width']), int(self.cfg['height'])
        self.period = 1.0 / float(self.cfg['fps'])
        self.last_time = -math.inf
        self.renderer = None
        self.disabled = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.image_pub = node.create_publisher(Image, f'{ns}/sim/camera/image_raw', qos)
        self.info_pub = node.create_publisher(CameraInfo, f'{ns}/sim/camera/camera_info', qos)
        fovy = math.radians(float(self.model.cam(f'{PREFIX}front_camera').fovy[0]))
        f = (self.height / 2) / math.tan(fovy / 2)
        cx, cy = self.width / 2, self.height / 2
        self.info = CameraInfo(width=self.width, height=self.height, distortion_model='plumb_bob',
                               d=[0.0] * 5, k=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
                               r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                               p=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.info.header.frame_id = self.cfg['frame_id']
        self.node = node

    def due(self, sim_time):
        """Return True when the next frame is due."""
        return not self.disabled and sim_time - self.last_time >= self.period - 1e-9

    def publish(self, data, stamp):
        """Render and publish one frame."""
        import mujoco
        if self.renderer is None:
            try:
                self.renderer = mujoco.Renderer(self.model, self.height, self.width)
                self.option = mujoco.MjvOption()
                for g in (1, 3, 5):
                    self.option.geomgroup[g] = 0
            except Exception as error:  # noqa: B902 - no OpenGL: keep simulating
                self.node.get_logger().warning(f'robot 2 camera disabled ({error})')
                self.disabled = True
                return
        self.last_time = data.time
        self.renderer.update_scene(data, camera=f'{PREFIX}front_camera',
                                   scene_option=self.option)
        if self.cfg.get('simple_render'):
            flag = mujoco.mjtRndFlag
            for f in (flag.mjRND_SHADOW, flag.mjRND_REFLECTION, flag.mjRND_SKYBOX, flag.mjRND_FOG):
                self.renderer.scene.flags[f] = 0
        rgb = self.renderer.render()
        image = self.Image()
        image.header.stamp, image.header.frame_id = stamp, self.cfg['frame_id']
        image.height, image.width = self.height, self.width
        image.encoding, image.is_bigendian, image.step = 'rgb8', 0, self.width * 3
        image.data = rgb.tobytes()
        self.image_pub.publish(image)
        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)
