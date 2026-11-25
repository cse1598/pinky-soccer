"""
MuJoCo simulation of Pinky Pro with the soccer bumper.

Mirrors pinky_mujoco/launch/launch_sim.launch.py (official, unmodified), which always
expands pinky_description/urdf/robot.urdf.xacro. Here the official expand_urdf() helper
expands pinky_soccer_description/urdf/soccer_robot.urdf.xacro instead, and the URDF is
handed to the official bridge with --urdf-file through mujoco_soccer_bridge. The wrapper
adds a dynamic ball and native IMU/ray sensors, configures the camera and normalizes
sensor interfaces to the physical robot. Official wheel physics/odometry/lidar remain.

Several robots: namespace puts robot 1 under /<namespace>/ (own /tf), opponent_name adds a
second robot in the same world (pinky_soccer_sim.mujoco_multi) under /<opponent_name>/;
opponent_sensors:=true also simulates its LiDAR/camera/IMU (needed when it plays).

Run it, for instance, as
    ros2 launch pinky_soccer_sim mujoco_soccer.launch.py
    ros2 launch pinky_soccer_sim mujoco_soccer.launch.py viewer:=false camera:=false
    ros2 launch pinky_soccer_sim mujoco_soccer.launch.py world_name:=pinky_factory.world
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pinky_soccer_bringup.calibration_profile import (arguments as profile_arguments,
                                                      configure_urdf, drive_constants)
from pinky_soccer_sim import multi_robot, sensor_launch
from pinky_soccer_sim.render_backend import MODES, renderer_environment
from pinky_soccer_sim.field_selection import selected_world
from pinky_soccer_sim.sensor_model import load_config
import yaml


def lc(name, context):
    return LaunchConfiguration(name).perform(context)


def resolve_world(context):
    """Return the world path; same rules as the official MuJoCo launch."""
    explicit = lc('world', context)
    selected = selected_world(LaunchConfiguration('field', default='').perform(context),
                              explicit, lc('world_name', context))
    if selected is not None:
        return selected
    if explicit:
        return Path(explicit).expanduser()
    name = lc('world_name', context)
    own = Path(get_package_share_directory('pinky_soccer_sim')) / 'worlds' / name
    if own.is_file():
        return own
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
    profile = lc('calibration_profile', context)
    urdf = configure_urdf(urdf, profile, True)
    wheel_radius, wheel_separation = drive_constants(profile, simulation=True)

    # Separate cache from the official launch so both can be used side by side.
    model_dir = Path(lc('model_dir', context)).expanduser()
    model_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = model_dir / 'robot.urdf'
    urdf_path.write_text(urdf)
    world = resolve_world(context)
    if not world.is_file():
        raise FileNotFoundError(f'World file not found: {world}')

    renderer, env, simple_render = renderer_environment(lc('use_gpu', context))
    args = ['--urdf-file', str(urdf_path), '--output-dir', str(model_dir), '--world', str(world)]
    # Simulator odometry calibration: drive constants only, physics geometry unchanged.
    if wheel_radius is not None:
        args += ['--wheel-radius', str(wheel_radius)]
    if wheel_separation is not None:
        args += ['--wheel-separation', str(wheel_separation)]
    sensor_path = lc('sensor_config', context)
    cfg = load_config(sensor_path)
    physics = yaml.safe_load((Path(get_package_share_directory('pinky_mujoco'))
                              / 'config/mujoco.yaml').read_text())
    from pinky_soccer_sim.motor_limits import configure_mujoco
    configure_mujoco(physics)
    physics['camera'].update({key: cfg['camera'][key] for key in
                              ('width', 'height', 'fps', 'horizontal_fov')})
    from pinky_soccer_sim.materials import load_materials
    material_path = lc('material_config', context)
    materials = load_materials(material_path)
    physics['physics']['timestep'] = materials['timestep']
    if lc('real_driver_limits', context).lower() == 'true':
        # Real robot: no separate v/w clamp, only the driver's 100 rpm ratio-preserving scaling
        # (pinky_bringup bringup.py). Lift the bridge clamps; the bridge wrapper scales wheels.
        physics['control']['max_linear'] = 10.0
        physics['control']['max_angular'] = 100.0
        args += ['--real-driver-max-rpm', '100']
    args += ['--material-config', material_path]
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
    if simple_render:
        args.append('--simple-render')
    if lc('spawn_ball', context).lower() == 'true':
        args += ['--ball-x', lc('ball_x', context), '--ball-y', lc('ball_y', context),
                 '--ball-radius', lc('ball_radius', context),
                 '--ball-mass', lc('ball_mass', context)]
    else:
        args.append('--no-ball')

    name = multi_robot.check_name(lc('namespace', context))
    opponent = multi_robot.check_name(lc('opponent_name', context))
    if opponent and opponent == name:
        raise ValueError('namespace and opponent_name must differ')
    spawn = [lc(k, context) for k in ('x', 'y', 'yaw')]
    if any(spawn):
        args += ['--spawn', *[v or '0' for v in spawn]]
    opp_sensors = lc('opponent_sensors', context).lower() == 'true'
    if opponent:
        args += ['--opponent-name', opponent, '--opponent-spawn',
                 *[lc(f'opponent_{k}', context) for k in ('x', 'y', 'yaw')]]
        if opp_sensors:
            args.append('--opponent-sensors')
        elif lc('opponent_camera', context).lower() == 'true':
            args.append('--opponent-camera')
    # The official bridge uses absolute topics; move robot 1's into its namespace.
    remaps = [('/cmd_vel', 'drive/cmd_vel'), ('/scan', 'sim/lidar/scan')]
    if name:
        remaps += [('/odom', 'odom'), ('/joint_states', 'joint_states'),
                   ('/ground_truth', 'ground_truth'), ('/cmd_vel_stamped', 'cmd_vel_stamped')]
    else:
        remaps = [('/cmd_vel', '/drive/cmd_vel'), ('/scan', '/sim/lidar/scan')]
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    out = multi_robot.group(name, [
        *sensor_launch.nodes(camera=camera),
        Node(package=lc('bridge_package', context), executable=lc('bridge_executable', context),
             name='pinky_mujoco', remappings=remaps, arguments=args, output='screen',
             additional_env=env, parameters=[{'use_sim_time': True}]),
        state])
    if opponent:
        robot2 = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                       name='robot_state_publisher', output='screen',
                       parameters=[{'robot_description': urdf, 'use_sim_time': True}])]
        robot2 += (sensor_launch.nodes(camera=camera) if opp_sensors else
                   [Node(package='pinky_soccer_bringup', executable='command_watchdog')])
        out += multi_robot.group(opponent, robot2)
    return [LogInfo(msg=f'MuJoCo renderer selection: {renderer}')] + out


def default_profile(simulator):
    """Return this simulator's own odometry calibration profile (or '' when absent)."""
    path = (Path(get_package_share_directory('pinky_soccer_sim')) / 'config' / 'calibration'
            / f'{simulator}_profile.yaml')
    return str(path) if path.is_file() else ''


def generate_launch_description():
    return LaunchDescription([
        *sensor_launch.arguments(),
        *profile_arguments(simulation=True, default=default_profile('mujoco')),
        DeclareLaunchArgument('use_gpu', default_value='auto', choices=MODES,
                              description='auto/true: tested NVIDIA PRIME or system; '
                              'nvidia: force PRIME; system: system OpenGL; false: CPU'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('real_driver_limits', default_value='true',
                              description='true: wheel limits like the real driver (100 rpm '
                              'scaling); false: official bridge clamps 0.25 m/s, 1.5 rad/s'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('material_config', default_value=str(
            Path(get_package_share_directory('pinky_soccer_sim'))/'config/materials.yaml')),
        DeclareLaunchArgument('field', default_value='', choices=['', 'rcj', 'hardboard'],
                              description='rcj|hardboard; empty preserves world/world_name'),
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
        DeclareLaunchArgument('ball_radius', default_value='0.02135'),
        DeclareLaunchArgument('ball_mass', default_value='0.0459'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='namespace of robot 1 (empty: none, global /tf)'),
        DeclareLaunchArgument('x', default_value='', description='robot 1 spawn x (empty: 0)'),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('opponent_name', default_value='',
                              description='second robot namespace (empty: one robot)'),
        DeclareLaunchArgument('opponent_x', default_value='0.4'),
        DeclareLaunchArgument('opponent_y', default_value='0.0'),
        DeclareLaunchArgument('opponent_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('opponent_camera', default_value='false',
                              description='obstacle opponent (opponent_sensors false): front '
                              'camera only, /<opponent>/sim/camera/image_raw, rendered on demand'),
        DeclareLaunchArgument('opponent_sensors', default_value='false',
                              description='simulate LiDAR/camera/IMU of the second robot'),
        DeclareLaunchArgument('bridge_package', default_value='pinky_soccer_sim'),
        DeclareLaunchArgument('bridge_executable', default_value='mujoco_soccer_bridge',
                              description='pinky_soccer_viz mujoco_eval_bridge adds ball truth'),
        DeclareLaunchArgument('model_dir', default_value=os.environ.get(
            'PINKY_SOCCER_MODEL_DIR', str(Path.home() / '.cache' / 'pinky_soccer_mujoco'))),
        OpaqueFunction(function=setup),
    ])
