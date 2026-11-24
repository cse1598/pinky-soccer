"""
Gazebo simulation of Pinky Pro with the soccer bumper.

Rebuilds pinky_gz_sim/launch/launch_sim.launch.xml (official, unmodified) because that
file includes pinky_description/launch/upload_robot.launch.py, which hard-codes
pinky_description/urdf/robot.urdf.xacro and would spawn the robot without the bumper.
Reused from the official packages by path: worlds, the robot xacro macro (through
pinky_soccer_description) and Gazebo plugins. Our bridge and runtime sensor model
normalize camera/IMU/ADC interfaces to the physical robot.

Differences from the official launch:
  - robot_description comes from pinky_soccer_description/urdf/soccer_robot.urdf.xacro
  - cam_tilt_deg defaults to 8 (same as the xacro and pinky_mujoco; official Gazebo uses 0)
  - screen_tilt_deg is passed through
  - native camera/IMU/ray inputs are bridged under /sim and normalized to public topics
  - optional test ball spawned next to the robot (the world file is not changed)
  - namespace:=<name> puts the robot's ROS nodes under /<name>/ (own /tf) and its gz topics
    under /<name>/; opponent_name:=<name> spawns a second robot the same way

Run it, for instance, as
    ros2 launch pinky_soccer_sim gazebo_soccer.launch.py
    ros2 launch pinky_soccer_sim gazebo_soccer.launch.py world_name:=pinky_factory.world
    ros2 launch pinky_soccer_sim gazebo_soccer.launch.py spawn_ball:=false gui:=false
"""
import os
from pathlib import Path
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from pinky_soccer_bringup.calibration_profile import arguments as profile_arguments, configure_urdf
from pinky_soccer_sim import multi_robot, sensor_launch
from pinky_soccer_sim.sensor_model import configure_gazebo, load_config
import xacro
from pinky_soccer_sim import materials
import yaml

BALL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="ball">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>
          <bounce><restitution_coefficient>0.6</restitution_coefficient></bounce>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <!-- Photo-based vivid orange approximation; fluorescence is not simulated. -->
          <ambient>1 0.30 0 1</ambient><diffuse>1 0.30 0 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular><shininess>64</shininess>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def lc(name):
    return LaunchConfiguration(name)


def spawn_ball(context):
    """Spawn the ball: a 42.7 mm golf ball by default (solid sphere)."""
    if lc('spawn_ball').perform(context).lower() != 'true':
        return []
    radius = float(lc('ball_radius').perform(context))
    mass = float(lc('ball_mass').perform(context))
    if radius <= 0 or mass <= 0:
        raise ValueError('ball_radius and ball_mass must be > 0')
    sdf = BALL_SDF.format(name='soccer_ball', radius=radius, mass=mass,
                          inertia=2.0 * mass * radius * radius / 5.0)  # solid sphere
    sdf = materials.configure_gazebo_ball(
        sdf, materials.load_materials(lc('material_config').perform(context)))
    return [Node(
        package='ros_gz_sim', executable='create', name='create_ball', output='screen',
        arguments=['-name', 'soccer_ball', '-string', sdf,
                   '-x', lc('ball_x').perform(context),
                   '-y', lc('ball_y').perform(context),
                   '-z', str(radius + 0.005)],
        parameters=[{'use_sim_time': True}])]


def robot_xml(context, sensors=True):
    """Expand official macros then add sensors only in the runtime model."""
    path = os.path.join(get_package_share_directory('pinky_soccer_description'),
                        'urdf', 'soccer_robot.urdf.xacro')
    mappings = {name: lc(name).perform(context) for name in
                ('cam_tilt_deg', 'screen_tilt_deg', 'bumper_enabled', 'bumper_params_file')}
    mappings['is_sim'] = 'true'
    urdf = xacro.process_file(path, mappings=mappings).toxml()
    cfg = load_config(lc('sensor_config').perform(context))
    flags = [sensors and lc(name).perform(context).lower() == 'true' for name in
             ('camera_enabled', 'imu_enabled', 'adc_enabled')]
    urdf = configure_gazebo(urdf, cfg, *flags)
    urdf = configure_urdf(urdf, lc('calibration_profile').perform(context), True)
    return urdf


def robots(context):
    """[(namespace, model name, (x, y, yaw), sensors)] for robot 1 and the optional opponent."""
    name = multi_robot.check_name(lc('namespace').perform(context))
    opponent = multi_robot.check_name(lc('opponent_name').perform(context))
    if opponent and opponent == name:
        raise ValueError('namespace and opponent_name must differ')
    out = [(name, name or lc('robot_name').perform(context),
            tuple(lc(k).perform(context) for k in ('x', 'y', 'yaw')), True)]
    if opponent:
        out.append((opponent, opponent,
                    tuple(lc(f'opponent_{k}').perform(context) for k in ('x', 'y', 'yaw')),
                    lc('opponent_sensors').perform(context).lower() == 'true'))
    return out


def setup_robot(context):
    """Per robot: model SDF, state publishers, spawn, gz bridge and sensor nodes."""
    with open(lc('bridge_params').perform(context)) as f:
        bridge_entries = yaml.safe_load(f)
    cfg = materials.load_materials(lc('material_config').perform(context))
    actions = []
    shared_bridge = False
    for ns, model, pose, sensors in robots(context):
        urdf = robot_xml(context, sensors)
        directory = Path(tempfile.mkdtemp(prefix='pinky-material-robot-'))
        path = directory/'robot.urdf'
        path.write_text(urdf)          # ROS description: plain (topics are a gz matter)
        gz_path = directory/'robot_gz.urdf'
        gz_path.write_text(multi_robot.namespace_gz_topics(urdf, ns))
        converted = subprocess.run(['gz', 'sdf', '-p', str(gz_path)],
                                   capture_output=True, text=True, check=True)
        sdf = directory/'robot.sdf'
        sdf.write_text(materials.configure_gazebo_robot(converted.stdout, urdf, cfg))
        if ns:
            entries, shared = multi_robot.namespace_bridge_config(bridge_entries, ns)
            bridge_file = directory/'bridge.yaml'
            bridge_file.write_text(yaml.safe_dump(entries))
            if not shared_bridge:
                clock_file = directory/'clock_bridge.yaml'
                clock_file.write_text(yaml.safe_dump(shared))
                actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                                    name='clock_bridge', output='screen', parameters=[{
                                        'config_file': str(clock_file), 'use_sim_time': True}]))
                shared_bridge = True
        else:
            bridge_file = lc('bridge_params').perform(context)
        nodes = [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher',
                 parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
            # upload_robot.launch.py equivalent
            Node(package='joint_state_publisher', executable='joint_state_publisher',
                 name='joint_state_publisher', output='screen',
                 parameters=[{'source_list': ['joint_states'], 'rate': 20.0,
                              'use_sim_time': True}]),
            Node(package='ros_gz_sim', executable='create', name='create_robot', output='screen',
                 arguments=['-name', model, '-file', str(sdf), '-x', pose[0], '-y', pose[1],
                            '-z', '0.1', '-Y', pose[2]],
                 parameters=[{'use_sim_time': True}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='parameter_bridge',
                 output='screen',
                 parameters=[{'config_file': str(bridge_file), 'use_sim_time': True}]),
            *(sensor_launch.nodes() if sensors else
              [Node(package='pinky_soccer_bringup', executable='command_watchdog')]),
        ]
        actions += multi_robot.group(ns, nodes)
    return actions


def setup_world(context):
    """Resolve own soccer worlds first while retaining official worlds and overrides."""
    from pathlib import Path
    from pinky_soccer_sim.field_selection import selected_world
    explicit = lc('world').perform(context)
    name = lc('world_name').perform(context)
    selected = selected_world(LaunchConfiguration('field', default='').perform(context),
                              explicit, name)
    candidates = [selected] if selected is not None else ([Path(explicit).expanduser()] if explicit else [
        Path(get_package_share_directory(package)) / 'worlds' / name
        for package in ('pinky_soccer_sim', 'pinky_gz_sim')])
    for path in candidates:
        if path.is_file():
            cfg = materials.load_materials(lc('material_config').perform(context))
            contacts = materials.sphere_contacts(robot_xml(context), cfg)
            directory = Path(tempfile.mkdtemp(prefix='pinky-material-world-'))
            ball = ((float(lc('ball_radius').perform(context)),
                     float(lc('ball_mass').perform(context)))
                    if lc('spawn_ball').perform(context).lower() == 'true' else None)
            output = materials.gazebo_world(path, directory/'world.sdf', cfg,
                                            [model for _, model, _, _ in robots(context)],
                                            contacts, ball)
            return [SetLaunchConfiguration('soccer_world', str(output))]
    raise FileNotFoundError(f'World file not found: {explicit or name}')


def default_profile(simulator):
    """Return this simulator's own odometry calibration profile (or '' when absent)."""
    path = (Path(get_package_share_directory('pinky_soccer_sim')) / 'config' / 'calibration'
            / f'{simulator}_profile.yaml')
    return str(path) if path.is_file() else ''


def generate_launch_description():
    soccer_description = get_package_share_directory('pinky_soccer_description')
    pinky_description = get_package_share_directory('pinky_description')
    pinky_gz_sim = get_package_share_directory('pinky_gz_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Same as the official launch, plus this workspace's description share so that
    # package://pinky_soccer_description/... meshes resolve once the CAD bumper exists.
    resource_path = os.pathsep.join(filter(None, [
        os.path.dirname(pinky_description),
        os.path.dirname(soccer_description),
        os.path.join(pinky_gz_sim, 'models'),
        os.path.join(os.path.expanduser('~'), '.gazebo', 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    return LaunchDescription([
        *sensor_launch.arguments(),
        *profile_arguments(simulation=True, default=default_profile('gazebo')),
        DeclareLaunchArgument('material_config', default_value=str(
            Path(get_package_share_directory('pinky_soccer_sim'))/'config/materials.yaml')),
        DeclareLaunchArgument('field', default_value='', choices=['', 'rcj', 'hardboard'],
                              description='rcj|hardboard; empty preserves world/world_name'),
        DeclareLaunchArgument('world_name', default_value='empty.world',
                              description='File in pinky_gz_sim/worlds'),
        DeclareLaunchArgument('world', default_value='', description='Explicit world file path'),
        DeclareLaunchArgument('bridge_params', default_value=PathJoinSubstitution(
            [get_package_share_directory('pinky_soccer_sim'), 'config', 'gazebo_bridge.yaml'])),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('robot_name', default_value='pinky',
                              description='Gazebo model name when namespace is empty'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='robot ROS namespace and model name (empty: none)'),
        DeclareLaunchArgument('opponent_name', default_value='',
                              description='second robot namespace/model (empty: one robot)'),
        DeclareLaunchArgument('opponent_x', default_value='0.4'),
        DeclareLaunchArgument('opponent_y', default_value='0.0'),
        DeclareLaunchArgument('opponent_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('opponent_sensors', default_value='false',
                              description='camera/IMU/ADC sensors on the second robot'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='8'),
        DeclareLaunchArgument('screen_tilt_deg', default_value='-25'),
        DeclareLaunchArgument('bumper_enabled', default_value='true'),
        DeclareLaunchArgument('bumper_params_file', default_value=PathJoinSubstitution(
            [soccer_description, 'config', 'bumper_v1_params.yaml'])),
        DeclareLaunchArgument('spawn_ball', default_value='true'),
        DeclareLaunchArgument('ball_x', default_value='0.3'),
        DeclareLaunchArgument('ball_y', default_value='0.0'),
        DeclareLaunchArgument('ball_radius', default_value='0.02135',
                              description='Golf ball (42.7 mm)'),
        DeclareLaunchArgument('ball_mass', default_value='0.0459',
                              description='Golf ball (45.9 g)'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        OpaqueFunction(function=setup_world),

        IncludeLaunchDescription(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r -s -v4 ', lc('soccer_world')],
                              'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': '-g -v4'}.items(),
            condition=IfCondition(lc('gui'))),

        OpaqueFunction(function=setup_robot),
        OpaqueFunction(function=spawn_ball),
    ])
