"""One PC entry point: mode real/gazebo/mujoco, RViz, and a separate start signal."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    """Route to the existing real or simulated complete game stack."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mode = arg('mode')
    if mode not in ('real', 'gazebo', 'mujoco'):
        raise ValueError('mode must be real, gazebo or mujoco')
    max_lin, max_ang = float(arg('max_lin')), float(arg('max_ang'))
    if not (math.isfinite(max_lin) and math.isfinite(max_ang)
            and 0 < max_lin <= 0.25 and 0 < max_ang <= 2.0):
        raise ValueError('Require 0 < max_lin <= 0.25 m/s and 0 < max_ang <= 2 rad/s')
    common = {name: arg(name) for name in
              ('field', 'our_goal_color', 'headless', 'max_lin', 'max_ang',
               'localization', 'heading_record', 'initial_x', 'initial_y', 'initial_yaw')}
    if mode == 'real':
        filename = 'real_game.launch.py'
        common.update({name: arg(name) for name in (
            'allow_uncalibrated', 'approximate_hfov_deg', 'camera_yaml',
            'image_rotated_180', 'initial_x', 'initial_y', 'initial_yaw', 'command_topic')})
    else:
        filename = 'sim_field.launch.py'
        common.update({name: arg(name) for name in
                       ('ball_x', 'ball_y', 'imu_enabled', 'adc_enabled')})
        common.update(simulator=mode, publish_debug_image='true')
    path = os.path.join(get_package_share_directory('pinky_soccer_viz'), 'launch', filename)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                     launch_arguments=common.items())]


def generate_launch_description():
    """Every mode starts paused; no game start publisher is launched here."""
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='real', choices=['real', 'gazebo', 'mujoco']),
        DeclareLaunchArgument('field', default_value='hardboard', choices=['hardboard', 'rcj']),
        DeclareLaunchArgument('our_goal_color', default_value='yellow',
                              choices=['yellow', 'blue']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('localization', default_value='lidar', choices=['lidar', 'odom_only']),
        DeclareLaunchArgument('heading_record', default_value=''),
        DeclareLaunchArgument('max_lin', default_value='0.08'),
        DeclareLaunchArgument('max_ang', default_value='0.5'),
        DeclareLaunchArgument('allow_uncalibrated', default_value='false'),
        DeclareLaunchArgument('approximate_hfov_deg', default_value='66.0'),
        DeclareLaunchArgument('camera_yaml', default_value=''),
        DeclareLaunchArgument('image_rotated_180', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('command_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('ball_x', default_value='0.3'),
        DeclareLaunchArgument('ball_y', default_value='0.0'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('adc_enabled', default_value='true'),
        OpaqueFunction(function=setup),
    ])
