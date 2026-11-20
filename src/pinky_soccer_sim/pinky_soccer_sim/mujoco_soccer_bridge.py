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
"""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

BALL_NAME = 'soccer_ball'
# pinky_mujoco masks: floor/world contype=1 conaffinity=2, robot contype=2 conaffinity=1.
# 3/3 makes the ball collide with both. Group 0 = drawn by default and hit by the lidar.
BALL_CONTYPE = '3'
BALL_CONAFFINITY = '3'
BALL_GROUP = '0'


def add_ball(scene_path, x=0.3, y=0.0, radius=0.021335, mass=0.0459, name=BALL_NAME):
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
    # Appended last: pinky_mujoco Physics reads the robot pose from qpos[0:7], so the
    # robot's free joint must stay first.
    body = ET.SubElement(worldbody, 'body', name=name,
                         pos=f'{x:.12g} {y:.12g} {radius + 0.001:.12g}')
    ET.SubElement(body, 'freejoint', name=f'{name}_free')
    inertia = 2.0 * mass * radius * radius / 5.0   # solid sphere
    ET.SubElement(body, 'inertial', pos='0 0 0', mass=f'{mass:.12g}',
                  diaginertia=f'{inertia:.12g} {inertia:.12g} {inertia:.12g}')
    ET.SubElement(body, 'geom', name=f'{name}_geom', type='sphere', size=f'{radius:.12g}',
                  rgba='1 0.45 0 1', group=BALL_GROUP,
                  contype=BALL_CONTYPE, conaffinity=BALL_CONAFFINITY)
    ET.indent(tree)
    tree.write(scene_path, encoding='unicode')
    return scene_path


def parse_args(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--ball', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--ball-x', type=float, default=0.3)
    parser.add_argument('--ball-y', type=float, default=0.0)
    parser.add_argument('--ball-radius', type=float, default=0.021335)
    parser.add_argument('--ball-mass', type=float, default=0.0459)
    parser.add_argument('--sensor-config')
    parser.add_argument('--imu-enabled', action='store_true')
    parser.add_argument('--adc-enabled', action='store_true')
    return parser.parse_known_args(argv)


def main(args=None):
    from pinky_mujoco import bridge
    from pinky_soccer_sim.rcj_field import install_mujoco

    options, rest = parse_args(sys.argv[1:] if args is None else args)
    install_mujoco(bridge)
    if options.sensor_config:
        from pinky_soccer_sim.mujoco_sensors import install
        install(bridge, options)
    if options.ball:
        official_build_model = bridge.build_model

        def build_model_with_ball(*a, **kw):
            return add_ball(official_build_model(*a, **kw), options.ball_x, options.ball_y,
                            options.ball_radius, options.ball_mass)

        bridge.build_model = build_model_with_ball
    bridge.main(rest)


if __name__ == '__main__':
    main()
