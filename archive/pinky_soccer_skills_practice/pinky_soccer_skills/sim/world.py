"""
Headless MuJoCo world for skill tests: the same model chain as mujoco_soccer.launch.py.

Builds the official pinky_mujoco model from the soccer URDF (bumper), swaps in the generated
field MJCF, adds the free golf ball and the shared materials (pinky_soccer_sim), then steps
the official Physics class directly without ROS. Opponents are kinematic (mocap) boxes that
the scenario moves; they push the ball like a rigid robot but have no motor model.
"""

import math
from pathlib import Path
import tempfile
import types
import xml.etree.ElementTree as ET

import numpy as np
import yaml

OPPONENT_SIZE = (0.06, 0.055, 0.05)    # half extents [m]: ~Pinky footprint without bumper


def _share(pkg):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(pkg))


def add_opponents(scene_path, count):
    """Append `count` mocap boxes named opponent_<i> (parked outside the field)."""
    tree = ET.parse(scene_path)
    world = tree.getroot().find('worldbody')
    for i in range(count):
        body = ET.SubElement(world, 'body', name=f'opponent_{i}', mocap='true',
                             pos=f'{5 + i} 5 {OPPONENT_SIZE[2] + 0.002}')
        ET.SubElement(body, 'geom', name=f'opponent_{i}_geom', type='box',
                      size=' '.join(map(str, OPPONENT_SIZE)), rgba='0.15 0.15 0.15 1',
                      group='0', contype='15', conaffinity='15')
    ET.indent(tree)
    tree.write(scene_path, encoding='unicode')
    return scene_path


class SoccerWorld:
    """Physics + ground truth access. Time advances in 10 ms bridge periods."""

    def __init__(self, field='hardboard', opponents=0, model_dir=None, cam_tilt_deg=8.0,
                 ball_mass=0.0459, ball_radius=0.02135):
        from pinky_mujoco import model as pm
        from pinky_mujoco.description import expand_urdf
        from pinky_soccer_sim import materials, rcj_field
        from pinky_soccer_sim.mujoco_soccer_bridge import add_ball

        self.field_name = field
        out = Path(model_dir or tempfile.mkdtemp(prefix='pinky_skills_'))
        out.mkdir(parents=True, exist_ok=True)
        urdf = expand_urdf(package='pinky_soccer_description',
                           filename='urdf/soccer_robot.urdf.xacro',
                           mappings={'cam_tilt_deg': str(cam_tilt_deg),
                                     'screen_tilt_deg': '-25', 'bumper_enabled': 'true'})
        (out / 'robot.urdf').write_text(urdf)
        sim_share = _share('pinky_soccer_sim')
        mat_cfg = materials.load_materials(str(sim_share / 'config/materials.yaml'))
        sensors = yaml.safe_load((sim_share / 'config/sensors.yaml').read_text())
        physics = yaml.safe_load((_share('pinky_mujoco') / 'config/mujoco.yaml').read_text())
        physics['physics']['timestep'] = mat_cfg['timestep']
        physics['camera'].update({k: sensors['camera'][k]
                                  for k in ('width', 'height', 'fps', 'horizontal_fov')})
        physics_path = out / 'physics.yaml'
        physics_path.write_text(yaml.safe_dump(physics))
        self.sensor_cfg = sensors
        self.camera_cfg = physics['camera']

        ns = types.SimpleNamespace(build_model=pm.build_model, Physics=pm.Physics)
        rcj_field.install_mujoco(ns)
        base_build = ns.build_model

        def build(*a, **kw):
            path = add_ball(base_build(*a, **kw), 0.3, 0.0, ball_radius, ball_mass)
            return add_opponents(path, opponents)
        ns.build_model = build
        materials.install_mujoco(ns, mat_cfg)
        world = sim_share / 'worlds' / f'{field}_soccer.world'
        path = ns.build_model(str(_share('pinky_mujoco')), str(out), urdf, str(physics_path),
                              str(world), simple_render=True)
        self.physics = ns.Physics(path, str(physics_path))
        self.model, self.data = self.physics.model, self.physics.data
        self.mj = self.physics.mj
        ball = self.model.body('soccer_ball')
        self.ball_q = self.model.jnt_qposadr[ball.jntadr[0]]
        self.ball_v = self.model.jnt_dofadr[ball.jntadr[0]]
        self.ball_geom = self.model.geom('soccer_ball_geom').id
        self.ball_radius = ball_radius
        self.opponent_mocap = [self.model.body(f'opponent_{i}').mocapid[0]
                               for i in range(opponents)]
        self.opponent_geoms = [self.model.geom(f'opponent_{i}_geom').id
                               for i in range(opponents)]
        self.camera_id = self.model.camera('front_camera').id
        self.us_body = self.model.body('ultrasonic_link').id
        self.base_body = self.model.body('base_link').id
        self.mj.mj_forward(self.model, self.data)
        self.z0 = float(self.data.qpos[2])

    # ------------------------------------------------------------------ state
    @property
    def time(self):
        """Return the simulation time [s]."""
        return float(self.data.time)

    def set_robot(self, x, y, theta):
        """Place the robot (zero velocity)."""
        q = self.data.qpos
        q[0:3] = [x, y, self.z0]
        q[3:7] = [math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2)]
        self.data.qvel[0:6] = 0.0
        self.data.ctrl[:] = 0.0
        self.physics.odom[:] = [x, y, theta]
        self.mj.mj_forward(self.model, self.data)

    def set_ball(self, x, y, vx=0.0, vy=0.0):
        """Place the ball, optionally rolling (spin matched to rolling without slip)."""
        q, v, r = self.ball_q, self.ball_v, self.ball_radius
        self.data.qpos[q:q + 7] = [x, y, r + 0.0005, 1, 0, 0, 0]
        self.data.qvel[v:v + 6] = [vx, vy, 0.0, -vy / r, vx / r, 0.0]
        self.mj.mj_forward(self.model, self.data)

    def set_opponent(self, i, x, y, theta=0.0):
        """Move a kinematic opponent."""
        m = self.opponent_mocap[i]
        self.data.mocap_pos[m] = [x, y, OPPONENT_SIZE[2] + 0.002]
        self.data.mocap_quat[m] = [math.cos(theta / 2), 0, 0, math.sin(theta / 2)]

    def robot_pose(self):
        """Return the true (x, y, theta)."""
        return tuple(float(v) for v in self.physics.pose())

    def robot_twist(self):
        """Wheel-odometry twist (v, w) as the robot would measure it."""
        return float(self.physics.twist[0]), float(self.physics.twist[1])

    def ball_state(self):
        """Return the true (x, y, vx, vy)."""
        q, v = self.ball_q, self.ball_v
        d = self.data
        return (float(d.qpos[q]), float(d.qpos[q + 1]), float(d.qvel[v]), float(d.qvel[v + 1]))

    def opponent_xy(self, i):
        """Return the true opponent centre."""
        p = self.data.mocap_pos[self.opponent_mocap[i]]
        return (float(p[0]), float(p[1]))

    def step(self, v, w):
        """Advance one bridge period (10 ms) with a velocity command."""
        self.physics.step(v, w)

    # ------------------------------------------------------------------ sensing
    def camera_pose(self):
        """Camera position and rotation (MuJoCo camera looks along -z, up +y)."""
        self.mj.mj_forward(self.model, self.data)
        return (self.data.cam_xpos[self.camera_id].copy(),
                self.data.cam_xmat[self.camera_id].reshape(3, 3).copy())

    def ray(self, origin, direction, max_dist):
        """First hit (distance, geom id) of a ray against world/ball/opponent geoms."""
        geomid = np.zeros(1, dtype=np.int32)
        group = np.array([1, 0, 0, 0, 1, 0], dtype=np.uint8)
        d = self.mj.mj_ray(self.model, self.data, np.asarray(origin, float),
                           np.asarray(direction, float), group, 1, -1, geomid)
        if d < 0 or d > max_dist:
            return math.inf, -1
        return float(d), int(geomid[0])

    def ultrasonic_origin(self):
        """Ultrasonic sensor position and forward axis."""
        self.mj.mj_forward(self.model, self.data)
        return (self.data.xpos[self.us_body].copy(),
                self.data.xmat[self.us_body].reshape(3, 3)[:, 0].copy())
