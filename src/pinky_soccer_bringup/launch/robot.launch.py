"""
Real Pinky Pro with the soccer bumper (run on the robot).

Rebuilds pinky_bringup/launch/bringup_robot.launch.xml (official, unmodified). That file
includes pinky_description/launch/upload_robot.launch.py, which hard-codes
pinky_description/urdf/robot.urdf.xacro, so only that part is replaced: robot_description
comes from pinky_soccer_description/urdf/soccer_robot.urdf.xacro. The lidar driver, the
motor driver (pinky_bringup bringup) and battery_publisher are started exactly as in the
official file.

Wheel joint names: the official driver publishes left_wheel_joint / right_wheel_joint but
the URDF uses l_wheel_joint / r_wheel_joint, so the official setup never moves the wheels
in TF. Here the driver output is remapped to wheel_joint_states_raw,
wheel_joint_state_relay renames the joints, and joint_state_publisher merges
wheel_joint_states into /joint_states.

cam_tilt_deg defaults to 0 like upload_robot.launch.py, so the real robot keeps the TF it
had before. The xacro, Gazebo and MuJoCo launches use 8; measure the real tilt and align.

hardware:=false starts only robot_state_publisher / joint_state_publisher, which is useful
for checking the description on a PC without the robot's drivers installed.

Run it, for instance, as
    ros2 launch pinky_soccer_bringup robot.launch.py
    ros2 launch pinky_soccer_bringup robot.launch.py hardware:=false
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def lc(name):
    return LaunchConfiguration(name)


def generate_launch_description():
    soccer_description = get_package_share_directory('pinky_soccer_description')
    hardware = IfCondition(lc('hardware'))

    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([soccer_description, 'urdf', 'soccer_robot.urdf.xacro']),
        ' is_sim:=false',
        ' cam_tilt_deg:=', lc('cam_tilt_deg'),
        ' screen_tilt_deg:=', lc('screen_tilt_deg'),
        ' bumper_enabled:=', lc('bumper_enabled'),
        ' bumper_params_file:=', lc('bumper_params_file'),
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='true',
                              description='false: description only (no lidar/motor/battery)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Same defaults as bringup_robot.launch.xml / upload_robot.launch.py
        DeclareLaunchArgument('wheel_radius', default_value='0.027'),
        DeclareLaunchArgument('wheel_separation', default_value='0.0961'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='0'),
        DeclareLaunchArgument('screen_tilt_deg', default_value='-25'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('bumper_enabled', default_value='true'),
        DeclareLaunchArgument('bumper_params_file', default_value=PathJoinSubstitution(
            [soccer_description, 'config', 'bumper_params.yaml'])),

        # upload_robot.launch.py equivalent
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': lc('use_sim_time'),
                          'ignore_timestamp': False}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             parameters=[{'source_list': ['wheel_joint_states'], 'rate': 20.0,
                          'use_sim_time': lc('use_sim_time')}]),
        Node(package='pinky_soccer_bringup', executable='wheel_joint_state_relay',
             name='wheel_joint_state_relay', output='screen',
             parameters=[{'use_sim_time': lc('use_sim_time')}]),

        # Unchanged from bringup_robot.launch.xml
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('sllidar_ros2'),
                                  'launch', 'sllidar_c1_launch.py']),
            launch_arguments={'serial_port': lc('lidar_serial_port'),
                              'frame_id': 'rplidar_link',
                              'inverted': 'false',
                              'angle_compensate': 'true',
                              'scan_mode': 'DenseBoost'}.items(),
            condition=hardware),
        Node(package='pinky_bringup', executable='bringup',
             parameters=[PathJoinSubstitution([FindPackageShare('pinky_bringup'),
                                               'config', 'pinky_params.yaml']),
                         {'wheel_radius': ParameterValue(lc('wheel_radius'), value_type=float),
                          'wheel_separation': ParameterValue(lc('wheel_separation'),
                                                             value_type=float)}],
             remappings=[('joint_states', 'wheel_joint_states_raw')],
             condition=hardware),
        Node(package='pinky_bringup', executable='battery_publisher', condition=hardware),
    ])
