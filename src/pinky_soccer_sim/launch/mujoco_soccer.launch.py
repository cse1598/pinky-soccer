"""
MuJoCo simulation of Pinky Pro with the soccer bumper.

Mirrors pinky_mujoco/launch/launch_sim.launch.py (official, unmodified), which always
expands pinky_description/urdf/robot.urdf.xacro. Here the official expand_urdf() helper
expands pinky_soccer_description/urdf/soccer_robot.urdf.xacro instead, and the URDF is
handed to the official bridge with --urdf-file through mujoco_soccer_bridge. The wrapper
adds a dynamic ball and native IMU/ray sensors, configures the camera and normalizes
sensor interfaces to the physical robot. Official wheel physics/odometry/lidar remain.

Run it, for instance, as
    ros2 launch pinky_soccer_sim mujoco_soccer.launch.py
    ros2 launch pinky_soccer_sim mujoco_soccer.launch.py viewer:=false camera:=false
    ros2 launch pinky_soccer_sim mujoco_soccer.launch.py world_name:=pinky_factory.world
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pinky_soccer_bringup.calibration_profile import arguments as profile_arguments, configure_urdf
from pinky_soccer_sim import sensor_launch
from pinky_soccer_sim.sensor_model import load_config
import yaml


def lc(name, context):
    return LaunchConfiguration(name).perform(context)


def resolve_world(context):
    """Return the world path; same rules as the official MuJoCo launch."""
    explicit = lc('world', context)
    if explicit:
        return Path(explicit).expanduser()
    name = lc('world_name', context)
    if name.endswith('.yaml'):
        return Path(get_package_share_directory('pinky_mujoco')) / 'worlds' / name
    return Path(get_package_share_directory('pinky_gz_sim')) / 'worlds' / name


def setup(context):
    from pinky_mujoco.description import expand_urdf

    mappings = {
        'cam_tilt_deg': lc('cam_tilt_deg', context),
        'screen_tilt_deg': lc('screen_tilt_deg', context),
        'bumper_enabled': lc('bumper_enabled', context),
    }
    params_file = lc('bumper_params_file', context)
    if params_file:
        mappings['bumper_params_file'] = params_file
    urdf = expand_urdf(package='pinky_soccer_description',
                       filename='urdf/soccer_robot.urdf.xacro', mappings=mappings)
    urdf = configure_urdf(urdf, lc('calibration_profile', context), True)

    # Separate cache from the official launch so both can be used side by side.
    model_dir = Path(lc('model_dir', context)).expanduser()
    model_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = model_dir / 'robot.urdf'
    urdf_path.write_text(urdf)
    world = resolve_world(context)
    if not world.is_file():
        raise FileNotFoundError(f'World file not found: {world}')

    use_gpu = lc('use_gpu', context).lower() == 'true'
    env = {}
    if not use_gpu:
        env = {'MUJOCO_GL': 'glfw', 'LIBGL_ALWAYS_SOFTWARE': '1',
               '__GLX_VENDOR_LIBRARY_NAME': 'mesa'}
    args = ['--urdf-file', str(urdf_path), '--output-dir', str(model_dir), '--world', str(world)]
    sensor_path = lc('sensor_config', context)
    cfg = load_config(sensor_path)
    physics = yaml.safe_load((Path(get_package_share_directory('pinky_mujoco'))
                              / 'config/mujoco.yaml').read_text())
    physics['camera'].update({key: cfg['camera'][key] for key in
                              ('width', 'height', 'fps', 'horizontal_fov')})
    physics_path = model_dir / 'physics.yaml'
    physics_path.write_text(yaml.safe_dump(physics))
    args += ['--physics-config', str(physics_path), '--sensor-config', sensor_path]
    for name in ('imu', 'adc'):
        if lc(f'{name}_enabled', context).lower() == 'true':
            args.append(f'--{name}-enabled')
    camera = (lc('camera', context).lower() == 'true'
              and lc('camera_enabled', context).lower() == 'true')
    if lc('viewer', context).lower() == 'true':
        args.append('--viewer')
    if not camera:
        args.append('--no-camera')
    if not use_gpu:
        args.append('--simple-render')
    if lc('spawn_ball', context).lower() == 'true':
        args += ['--ball-x', lc('ball_x', context), '--ball-y', lc('ball_y', context),
                 '--ball-radius', lc('ball_radius', context),
                 '--ball-mass', lc('ball_mass', context)]
    else:
        args.append('--no-ball')

    return [
        *sensor_launch.nodes(camera=camera),
        Node(package='pinky_soccer_sim', executable='mujoco_soccer_bridge', name='pinky_mujoco',
             remappings=[('/cmd_vel', '/drive/cmd_vel')],
             arguments=args, output='screen', additional_env=env,
             parameters=[{'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        *sensor_launch.arguments(),
        *profile_arguments(simulation=True),
        DeclareLaunchArgument('use_gpu', default_value='false'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument(
            'world_name', default_value='empty.world',
            description='File in pinky_gz_sim/worlds or *.yaml in pinky_mujoco/worlds'),
        DeclareLaunchArgument('world', default_value='', description='Absolute world path'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='8'),
        DeclareLaunchArgument('screen_tilt_deg', default_value='-25'),
        DeclareLaunchArgument('bumper_enabled', default_value='true'),
        DeclareLaunchArgument('bumper_params_file', default_value='',
                              description='Empty: pinky_soccer_description default yaml'),
        DeclareLaunchArgument('spawn_ball', default_value='true'),
        DeclareLaunchArgument('ball_x', default_value='0.3'),
        DeclareLaunchArgument('ball_y', default_value='0.0'),
        DeclareLaunchArgument('ball_radius', default_value='0.021335'),
        DeclareLaunchArgument('ball_mass', default_value='0.0459'),
        DeclareLaunchArgument('model_dir', default_value=os.environ.get(
            'PINKY_SOCCER_MODEL_DIR', str(Path.home() / '.cache' / 'pinky_soccer_mujoco'))),
        OpaqueFunction(function=setup),
    ])
