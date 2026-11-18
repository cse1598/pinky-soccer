"""
Show Pinky Pro with the soccer bumper in RViz2.

Run it, for instance, as
    ros2 launch pinky_soccer_description view_robot.launch.py
    ros2 launch pinky_soccer_description view_robot.launch.py bumper_enabled:=false
    ros2 launch pinky_soccer_description view_robot.launch.py gui:=false
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('pinky_soccer_description')

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([share, 'urdf', 'soccer_robot.urdf.xacro']),
            ' bumper_enabled:=', LaunchConfiguration('bumper_enabled'),
            ' bumper_params_file:=', LaunchConfiguration('bumper_params_file'),
            ' cam_tilt_deg:=', LaunchConfiguration('cam_tilt_deg'),
            ' screen_tilt_deg:=', LaunchConfiguration('screen_tilt_deg'),
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('bumper_enabled', default_value='true'),
        DeclareLaunchArgument(
            'bumper_params_file',
            default_value=PathJoinSubstitution([share, 'config', 'bumper_params.yaml'])),
        DeclareLaunchArgument('cam_tilt_deg', default_value='8'),
        DeclareLaunchArgument('screen_tilt_deg', default_value='-25'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_config',
            # Reuse the official RViz config (fixed frame base_footprint, /robot_description).
            default_value=PathJoinSubstitution([
                get_package_share_directory('pinky_description'), 'rviz', 'view_robot.rviz'])),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
