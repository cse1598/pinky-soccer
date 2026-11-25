#!/usr/bin/env python3
"""
Run the official bridge with a dynamic ball and optional physical-interface sensors.

pinky_mujoco/world.py turns every world object into a static geom, so a ball placed in a
world file cannot be pushed. This wrapper keeps the official bridge (physics, odometry,
lidar) and post-processes scene.xml to append a free-floating sphere to <worldbody>.
With --sensor-config the runtime classes also add native IMU/ray observations and
route camera frames through the common sensor adapter.

Depends on pinky_mujoco internals (bridge.build_model, scene.xml layout, collision masks);
re-check after updating the submodule.

Own arguments (the rest is passed to pinky_mujoco.bridge unchanged):
    --ball / --no-ball, --ball-x, --ball-y, --ball-radius, --ball-mass
    --real-driver-max-rpm N              real driver wheel scaling instead of v/w clamps
    --wheel-radius, --wheel-separation   odometry/command conversion constants (calibration
        profile `launch` values). Like the physical robot's driver parameters they change only
        how /cmd_vel becomes wheel speeds and how wheel encoders become /odom; the wheel
        collision geometry (the physics) is left untouched.
"""
import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

BALL_NAME = 'soccer_ball'
# pinky_mujoco masks: floor/world contype=1 conaffinity=2, robot contype=2 conaffinity=1.
# 3/3 makes the ball collide with both. Group 0 = drawn by default and hit by the lidar.
BALL_CONTYPE = '3'
BALL_CONAFFINITY = '3'
BALL_GROUP = '0'


def add_ball(scene_path, x=0.3, y=0.0, radius=0.02135, mass=0.0459, name=BALL_NAME):
    """Append a free solid sphere (golf ball) as the last body of <worldbody>."""
    if radius <= 0 or mass <= 0:
        raise ValueError('ball radius and mass must be > 0')
    scene_path = Path(scene_path)
    tree = ET.parse(scene_path)
    worldbody = tree.getroot().find('worldbody')
    if worldbody is None:
        raise ValueError(f'{scene_path} has no <worldbody>')
    if worldbody.find(f"body[@name='{name}']") is not None:
        raise ValueError(f'{scene_path} already has a body named {name}')
    # Approximate the photographed glossy fluorescent-orange product in RGB;
    # this is not a measured spectral/UV fluorescence material model.
    asset = tree.getroot().find('asset')
    if asset is None:
        asset = ET.Element('asset')
        tree.getroot().insert(0, asset)
    material = f'{name}_material'
    ET.SubElement(asset, 'material', name=material, rgba='1 0.30 0 1',
                  specular='0.5', shininess='0.55')
    # Appended last: pinky_mujoco Physics reads the robot pose from qpos[0:7], so the
    # robot's free joint must stay first.
    body = ET.SubElement(worldbody, 'body', name=name,
                         pos=f'{x:.12g} {y:.12g} {radius + 0.001:.12g}')
    ET.SubElement(body, 'freejoint', name=f'{name}_free')
    inertia = 2.0 * mass * radius * radius / 5.0   # solid sphere
    ET.SubElement(body, 'inertial', pos='0 0 0', mass=f'{mass:.12g}',
                  diaginertia=f'{inertia:.12g} {inertia:.12g} {inertia:.12g}')
    ET.SubElement(body, 'geom', name=f'{name}_geom', type='sphere', size=f'{radius:.12g}',
                  material=material, group=BALL_GROUP,
                  contype=BALL_CONTYPE, conaffinity=BALL_CONAFFINITY)
    ET.indent(tree)
    tree.write(scene_path, encoding='unicode')
    return scene_path


def parse_args(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--ball', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--ball-x', type=float, default=0.3)
    parser.add_argument('--ball-y', type=float, default=0.0)
    parser.add_argument('--ball-radius', type=float, default=0.02135)
    parser.add_argument('--ball-mass', type=float, default=0.0459)
    parser.add_argument('--sensor-config')
    parser.add_argument('--material-config')
    parser.add_argument('--imu-enabled', action='store_true')
    parser.add_argument('--adc-enabled', action='store_true')
    parser.add_argument('--wheel-radius', type=float)
    parser.add_argument('--wheel-separation', type=float)
    parser.add_argument('--spawn', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                        help='robot 1 start pose (default: official spawn at the origin)')
    parser.add_argument('--opponent-name', default='',
                        help='add a second robot publishing under /<name>/ (mujoco_multi)')
    parser.add_argument('--opponent-spawn', type=float, nargs=3, default=(0.4, 0.0, math.pi),
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--opponent-sensors', action='store_true',
                        help='also simulate robot 2 LiDAR/camera/IMU/ranges (game mode)')
    parser.add_argument('--opponent-camera', action='store_true',
                        help='obstacle robot 2: front camera only, rendered on demand')
    parser.add_argument('--real-driver-max-rpm', type=float, default=0.0,
                        help='> 0: scale both wheels like pinky_bringup (MAX_RPM, ratio kept)')
    return parser.parse_known_args(argv)


def driver_scale(linear, angular, radius, track, max_rpm):
    """Return the factor the real driver applies so the faster wheel stays <= max_rpm."""
    peak = max(abs(linear - angular * track / 2), abs(linear + angular * track / 2)) / radius
    peak_rpm = peak * 60.0 / (2.0 * math.pi)
    return max_rpm / peak_rpm if peak_rpm > max_rpm else 1.0


def install_real_driver(bridge, max_rpm):
    """
    Make cmd_vel -> wheels behave like the real pinky_bringup driver.

    The official MuJoCo bridge clamps v and w separately (control.max_linear/max_angular,
    0.25 m/s and 1.5 rad/s), which the real robot does not do: its driver only scales both
    wheels by the same factor when one exceeds MAX_RPM. The launch lifts the bridge clamps and
    this wrapper applies the real scaling before Physics.step.
    """
    if not max_rpm or max_rpm <= 0:
        return
    base = bridge.Physics

    class RealDriverPhysics(base):
        def step(self, linear=0.0, angular=0.0, steps=None):
            k = driver_scale(linear, angular, self.radius, self.track, max_rpm)
            return super().step(linear * k, angular * k, steps)

    bridge.Physics = RealDriverPhysics


def install_drive_constants(bridge, radius=None, separation=None):
    """Override Physics.radius/track (cmd_vel conversion and encoder odometry) only."""
    if radius is None and separation is None:
        return
    for name, value in (('wheel radius', radius), ('wheel separation', separation)):
        if value is not None and not (0.0 < value < 1.0):
            raise ValueError(f'{name} {value} out of range')
    base = bridge.Physics

    class CalibratedDrivePhysics(base):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.model_radius, self.model_track = self.radius, self.track
            if radius is not None:
                self.radius = float(radius)
            if separation is not None:
                self.track = float(separation)

    bridge.Physics = CalibratedDrivePhysics


def main(args=None):
    from pinky_mujoco import bridge
    from pinky_soccer_sim.rcj_field import install_mujoco

    from pinky_soccer_sim import mujoco_multi

    options, rest = parse_args(sys.argv[1:] if args is None else args)
    install_mujoco(bridge)
    # Two-robot step directly above the official Physics (before every other wrapper).
    mujoco_multi.install_base(bridge, options.real_driver_max_rpm)
    if options.sensor_config:
        from pinky_soccer_sim.mujoco_sensors import install
        install(bridge, options)
    if options.ball:
        official_build_model = bridge.build_model

        def build_model_with_ball(*a, **kw):
            return add_ball(official_build_model(*a, **kw), options.ball_x, options.ball_y,
                            options.ball_radius, options.ball_mass)

        bridge.build_model = build_model_with_ball
    from pinky_soccer_sim.materials import install_mujoco as install_materials, load_materials
    material_cfg = load_materials(options.material_config)
    install_materials(bridge, material_cfg)
    install_drive_constants(bridge, options.wheel_radius, options.wheel_separation)
    install_real_driver(bridge, options.real_driver_max_rpm)
    mujoco_multi.install_after(bridge, material_cfg)
    if options.spawn is not None or options.opponent_name:
        inner_build = bridge.build_model

        def build_with_robots(*a, **kw):
            return mujoco_multi.place_robots(
                inner_build(*a, **kw), options.spawn,
                options.opponent_spawn if options.opponent_name else None)
        bridge.build_model = build_with_robots
    if options.opponent_name:
        from pinky_soccer_sim.sensor_model import load_config
        mujoco_multi.install_bridge(
            bridge, options.opponent_name, options.opponent_sensors,
            load_config(options.sensor_config) if options.sensor_config else None,
            options.imu_enabled, options.adc_enabled, camera_only=options.opponent_camera)
    bridge.main(rest)


if __name__ == '__main__':
    main()
