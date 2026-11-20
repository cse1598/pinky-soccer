"""Robot- and simulator-free loop: fake_world_model -> bt_node -> reactive_controller + UIs.

ros2 launch pinky_soccer_viz sim_viz.launch.py            # with RViz2 and rqt_image_view
ros2 launch pinky_soccer_viz sim_viz.launch.py gui:=false # headless (topics only)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    viz = get_package_share_directory('pinky_soccer_viz')
    wm_cfg = os.path.join(get_package_share_directory('pinky_soccer_world_model'), 'config')
    bt_cfg = os.path.join(get_package_share_directory('pinky_soccer_bt'), 'config')
    ctl_cfg = os.path.join(get_package_share_directory('pinky_soccer'), 'config')
    gui = IfCondition(LaunchConfiguration('gui'))
    common = {'our_goal_color': LaunchConfiguration('our_goal_color')}
    ui = {**common, 'snapshot_dir': LaunchConfiguration('snapshot_dir'),
          'snapshot_period': LaunchConfiguration('snapshot_period')}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('our_goal_color', default_value='yellow'),
        DeclareLaunchArgument('snapshot_dir', default_value=''),
        DeclareLaunchArgument('snapshot_period', default_value='0.0'),
        Node(package='pinky_soccer_world_model', executable='fake_world_model',
             name='fake_world_model', output='screen',
             parameters=[os.path.join(wm_cfg, 'fake_world_model.yaml'), common]),
        Node(package='pinky_soccer_bt', executable='bt_node', name='bt_node', output='screen',
             parameters=[os.path.join(bt_cfg, 'bt_node.yaml')]),
        Node(package='pinky_soccer', executable='reactive_controller',
             name='reactive_controller', output='screen',
             parameters=[os.path.join(ctl_cfg, 'reactive_controller.yaml')]),
        Node(package='pinky_soccer_viz', executable='world_model_viz', name='world_model_viz',
             output='screen', parameters=[common]),
        Node(package='pinky_soccer_viz', executable='world_model_topview',
             name='world_model_topview', output='screen', parameters=[ui]),
        Node(package='rviz2', executable='rviz2', name='rviz2', condition=gui,
             arguments=['-d', os.path.join(viz, 'rviz', 'soccer.rviz')]),
        Node(package='rqt_image_view', executable='rqt_image_view', name='topview_viewer',
             condition=gui, arguments=['/world_model/topview']),
    ])
