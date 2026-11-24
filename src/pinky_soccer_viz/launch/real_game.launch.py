"""PC perception, world model, BT and low-speed control for an already running robot."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pinky_soccer_sim.field_selection import field_files
from pinky_soccer_world_model.localization_launch import localization_nodes


def setup(context):
    """Resolve typed arguments and keep one field specification for every consumer."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        value = arg(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    def number(name):
        value = float(arg(name))
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def config(package, filename):
        return os.path.join(get_package_share_directory(package), 'config', filename)

    _, field_spec = field_files(arg('field'))
    team = {'field_spec': str(field_spec), 'our_goal_color': arg('our_goal_color'),
            'use_sim_time': False}
    max_lin, max_ang = number('max_lin'), number('max_ang')
    hfov = number('approximate_hfov_deg')
    if not (0 < max_lin <= 0.30 and 0 < max_ang <= 6.0):
        raise ValueError('Require 0 < max_lin <= 0.30 m/s and 0 < max_ang <= 6 rad/s')
    if not 1 < hfov < 179:
        raise ValueError('approximate_hfov_deg must be between 1 and 179')
    lidar = arg('localization') == 'lidar'
    initial = [number('initial_x'), number('initial_y'), number('initial_yaw')]
    nodes = localization_nodes(
        field_spec, False, initial, flag('allow_uncalibrated'), hfov, arg('camera_yaml'),
        arg('heading_record'), flag('image_rotated_180')) if lidar else []
    nodes += [
        LogInfo(msg='REAL ROBOT: starts paused. Place robot at kickoff pose BEFORE launch. '
                'Publish /game/playing=true to move; false to stop.'),
        Node(package='pinky_soccer_world_model', executable='ball_detector',
             name='ball_detector', output='screen', parameters=[
                 config('pinky_soccer_world_model', 'ball_detector.yaml'),
                 {'use_sim_time': False, 'allow_uncalibrated': flag('allow_uncalibrated'),
                  'approximate_hfov_deg': hfov, 'camera_yaml': arg('camera_yaml'),
                  'image_rotated_180': flag('image_rotated_180'), 'publish_debug_image': True}]),
        Node(package='pinky_soccer_world_model', executable='world_model_node',
             name='world_model_node', output='screen', parameters=[
                 config('pinky_soccer_world_model', 'world_model.yaml'),
                 config('pinky_soccer_world_model', 'ball_filter.yaml'), team,
                 {'localization': 'tf' if lidar else 'odom_only',
                  'localization_pose_timeout': 2.0 if lidar else 0.0,
                  'align_first_odom': True, 'odom_timeout': 0.5,
                  'playing_default': False, 'grid.camera_hfov_deg': hfov,
                  'initial_pose': [number('initial_x'), number('initial_y'),
                                   number('initial_yaw')]}]),
        Node(package='pinky_soccer_bt', executable='bt_node', name='bt_node', output='screen',
             parameters=[config('pinky_soccer_bt', 'bt_node.yaml'),
                         {'use_sim_time': False, 'groot2_enabled': False,
                          'verify_mode': flag('verify')}]),
        Node(package='pinky_soccer_viz', executable='world_model_viz',
             name='world_model_viz', output='screen', parameters=[team]),
        Node(package='pinky_soccer_viz', executable='world_model_topview',
             name='world_model_topview', output='screen',
             parameters=[team, {'show_ground_truth': False, 'camera_hfov_deg': hfov}]),
    ]
    if arg('controller') == 'mpc':
        nodes.append(Node(
            package='pinky_soccer_mpc', executable='mpc_controller', name='mpc_controller',
            output='screen', remappings=[('cmd_vel', arg('command_topic'))], parameters=[{
                'use_sim_time': False, 'target': 'real',
                'calibration_profile': arg('calibration_profile'),
                'max_lin': max_lin, 'max_ang': max_ang,
                'wheel_feedback': flag('wheel_feedback')}]))
    else:
        nodes.append(Node(
            package='pinky_soccer', executable='reactive_controller',
            name='reactive_controller', output='screen', parameters=[
                config('pinky_soccer', 'reactive_controller.yaml'),
                {'use_sim_time': False, 'max_lin': min(max_lin, 0.25),
                 'max_ang': min(max_ang, 2.0)}],
            remappings=[('cmd_vel', arg('command_topic'))]))
    if flag('verify'):
        nodes.append(LogInfo(msg='VERIFY MODE: the game tree is off; behaviours run only from '
                             'the verifier window. Keep a hand on the robot power switch.'))
        nodes.append(Node(
            package='pinky_soccer_viz', executable='behavior_verifier',
            name='behavior_verifier', output='screen', parameters=[team, {
                'simulation': False, 'mode_label': 'real',
                'controller_config': config('pinky_soccer', 'reactive_controller.yaml'),
                'max_lin': max_lin, 'max_ang': max_ang, 'controller_kind': arg('controller'),
                'command_topic': arg('command_topic')}]))
    if not flag('headless'):
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                          arguments=['-d', os.path.join(
                              get_package_share_directory('pinky_soccer_viz'),
                              'rviz', 'soccer.rviz')], parameters=[{'use_sim_time': False}]))
        nodes.append(Node(package='pinky_soccer_viz', executable='game_camera_view',
                          name='game_camera_view', output='screen',
                          parameters=[{'use_sim_time': False}]))
    return nodes


def generate_launch_description():
    """Never launch a simulator, sensor driver or automatic start command."""
    return LaunchDescription([
        DeclareLaunchArgument('field', default_value='hardboard', choices=['rcj', 'hardboard']),
        DeclareLaunchArgument('our_goal_color', default_value='yellow',
                              choices=['yellow', 'blue']),
        DeclareLaunchArgument('allow_uncalibrated', default_value='false'),
        DeclareLaunchArgument('approximate_hfov_deg', default_value='66.0'),
        DeclareLaunchArgument('camera_yaml', default_value=''),
        DeclareLaunchArgument('image_rotated_180', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('max_lin', default_value='0.08'),
        DeclareLaunchArgument('max_ang', default_value='0.5'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('localization', default_value='lidar', choices=['lidar', 'odom_only']),
        DeclareLaunchArgument('heading_record', default_value=''),
        DeclareLaunchArgument('controller', default_value='mpc', choices=['mpc', 'reactive']),
        DeclareLaunchArgument('wheel_feedback', default_value='false',
                              description='NMPC uses /joint_states wheel speeds (needs the '
                              'right wheel sign -1 confirmed; open-loop estimate otherwise)'),
        DeclareLaunchArgument('calibration_profile', default_value='',
                              description='robot profile.yaml for the NMPC model (empty: '
                              'official defaults, marked unmeasured)'),
        DeclareLaunchArgument('verify', default_value='false',
                              description='behaviour verification window + bt_node verify_mode'),
        DeclareLaunchArgument('command_topic', default_value='/cmd_vel',
                              description='Override for a non-driving dry run'),
        OpaqueFunction(function=setup),
    ])
