"""
Gazebo simulation of Pinky Pro with the soccer bumper.

Rebuilds pinky_gz_sim/launch/launch_sim.launch.xml (official, unmodified) because that
file includes pinky_description/launch/upload_robot.launch.py, which hard-codes
pinky_description/urdf/robot.urdf.xacro and would spawn the robot without the bumper.
Reused from the official packages by path: worlds, pinky_bridge.yaml, the robot xacro
macro (through pinky_soccer_description) and all Gazebo plugins.

Differences from the official launch:
  - robot_description comes from pinky_soccer_description/urdf/soccer_robot.urdf.xacro
  - cam_tilt_deg defaults to 8 (same as the xacro and pinky_mujoco; official Gazebo uses 0)
  - screen_tilt_deg is passed through
  - one image_bridge for camera/image_raw (official starts two nodes with the same name)
  - optional test ball spawned next to the robot (the world file is not changed)
  - namespace is not supported yet (official bridge topics are not namespaced either)

Run it, for instance, as
    ros2 launch pinky_soccer_sim gazebo_soccer.launch.py
    ros2 launch pinky_soccer_sim gazebo_soccer.launch.py world_name:=pinky_factory.world
    ros2 launch pinky_soccer_sim gazebo_soccer.launch.py spawn_ball:=false gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
          <bounce><restitution_coefficient>0.3</restitution_coefficient></bounce>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>1 0.45 0 1</ambient><diffuse>1 0.45 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def lc(name):
    return LaunchConfiguration(name)


def spawn_ball(context):
    """Spawn a solid test sphere (placeholder, not the real ball)."""
    if lc('spawn_ball').perform(context).lower() != 'true':
        return []
    radius = float(lc('ball_radius').perform(context))
    mass = float(lc('ball_mass').perform(context))
    if radius <= 0 or mass <= 0:
        raise ValueError('ball_radius and ball_mass must be > 0')
    sdf = BALL_SDF.format(name='soccer_ball', radius=radius, mass=mass,
                          inertia=2.0 * mass * radius * radius / 5.0)
    return [Node(
        package='ros_gz_sim', executable='create', name='create_ball', output='screen',
        arguments=['-name', 'soccer_ball', '-string', sdf,
                   '-x', lc('ball_x').perform(context),
                   '-y', lc('ball_y').perform(context),
                   '-z', str(radius + 0.005)],
        parameters=[{'use_sim_time': True}])]


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

    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([soccer_description, 'urdf', 'soccer_robot.urdf.xacro']),
        ' is_sim:=true',
        ' cam_tilt_deg:=', lc('cam_tilt_deg'),
        ' screen_tilt_deg:=', lc('screen_tilt_deg'),
        ' bumper_enabled:=', lc('bumper_enabled'),
        ' bumper_params_file:=', lc('bumper_params_file'),
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='empty.world',
                              description='File in pinky_gz_sim/worlds'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [pinky_gz_sim, 'worlds', lc('world_name')])),
        DeclareLaunchArgument('bridge_params', default_value=PathJoinSubstitution(
            [pinky_gz_sim, 'params', 'pinky_bridge.yaml'])),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('robot_name', default_value='pinky'),
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
        DeclareLaunchArgument('ball_radius', default_value='0.02',
                              description='Placeholder; keep the centre within the bumper height'),
        DeclareLaunchArgument('ball_mass', default_value='0.03'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),

        IncludeLaunchDescription(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r -s -v4 ', lc('world')],
                              'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': '-g -v4'}.items(),
            condition=IfCondition(lc('gui'))),

        # upload_robot.launch.py equivalent (robot_state_publisher + joint_state_publisher)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True,
                          'ignore_timestamp': False}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             parameters=[{'source_list': ['joint_states'], 'rate': 20.0,
                          'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create', name='create_robot', output='screen',
             arguments=['-name', lc('robot_name'), '-topic', 'robot_description',
                        '-x', lc('x'), '-y', lc('y'), '-z', '0.1', '-Y', lc('yaw')],
             parameters=[{'use_sim_time': True}]),
        OpaqueFunction(function=spawn_ball),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='parameter_bridge',
             output='screen',
             parameters=[{'config_file': lc('bridge_params'), 'use_sim_time': True}]),
        Node(package='ros_gz_image', executable='image_bridge', name='image_bridge',
             output='screen', arguments=['camera/image_raw'],
             parameters=[{'use_sim_time': True}]),
    ])
