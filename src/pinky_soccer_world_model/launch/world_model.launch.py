"""Run ball_detector and world_model_node with the package configs.

Example: ros2 launch pinky_soccer_world_model world_model.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('pinky_soccer_world_model'), 'config')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    overrides = {'localization': LaunchConfiguration('localization'),
                 'our_goal_color': LaunchConfiguration('our_goal_color'),
                 'field_spec': LaunchConfiguration('field_spec')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='odom_only'),
        DeclareLaunchArgument('our_goal_color', default_value='yellow'),
        DeclareLaunchArgument('field_spec', default_value=''),
        Node(package='pinky_soccer_world_model', executable='ball_detector',
             name='ball_detector', output='screen',
             parameters=[os.path.join(cfg, 'ball_detector.yaml'), sim_time]),
        Node(package='pinky_soccer_world_model', executable='world_model_node',
             name='world_model_node', output='screen',
             parameters=[os.path.join(cfg, 'world_model.yaml'),
                         os.path.join(cfg, 'ball_filter.yaml'), sim_time, overrides]),
    ])
