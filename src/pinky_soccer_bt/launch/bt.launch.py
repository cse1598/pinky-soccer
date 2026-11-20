"""Run bt_node alone: ros2 launch pinky_soccer_bt bt.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pinky_soccer_bt')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'bt_node.yaml')),
        Node(package='pinky_soccer_bt', executable='bt_node', name='bt_node', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
