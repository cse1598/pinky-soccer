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
    # auto: the real robot keeps the low safety caps until the user raises them; simulators
    # drive at the wheel limits (90 % of the motor rating) but turn at most 2 rad/s: faster
    # spins (up to 5.4 rad/s) broke LiDAR/heading localization in MuJoCo (180 deg reseeds,
    # 2.3 m errors; none at 2 rad/s). max_ang:=0 removes the turn cap.
    auto = {'real': ('0.08', '0.5')}.get(mode, ('0', '2.0'))
    lin_text = auto[0] if arg('max_lin') == 'auto' else arg('max_lin')
    ang_text = auto[1] if arg('max_ang') == 'auto' else arg('max_ang')
    max_lin, max_ang = float(lin_text), float(ang_text)
    low = 0.0 if mode != 'real' else 1e-9
    if not (math.isfinite(max_lin) and math.isfinite(max_ang)
            and low <= max_lin <= 0.30 and low <= max_ang <= 6.0):
        raise ValueError('Require max_lin <= 0.30 m/s and max_ang <= 6 rad/s '
                         '(> 0 on the real robot; 0 = wheel limits only in simulation)')
    common = {name: arg(name) for name in
              ('field', 'our_goal_color', 'headless', 'verify', 'controller',
               'localization', 'heading_record', 'initial_x', 'initial_y', 'initial_yaw')}
    common.update(max_lin=str(max_lin), max_ang=str(max_ang))
    if mode == 'real':
        filename = 'real_game.launch.py'
        common['calibration_profile'] = arg('calibration_profile')
        common['wheel_feedback'] = arg('wheel_feedback')
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
        DeclareLaunchArgument('verify', default_value='false', choices=['true', 'false'],
                              description='Behaviour verification mode: verifier window, '
                              'bt_node runs only requested behaviours'),
        DeclareLaunchArgument('localization', default_value='lidar', choices=['lidar', 'odom_only']),
        DeclareLaunchArgument('heading_record', default_value=''),
        DeclareLaunchArgument('max_lin', default_value='auto',
                              description='auto: real 0.08 m/s, simulation 0 (= wheel limits)'),
        DeclareLaunchArgument('max_ang', default_value='auto',
                              description='auto: real 0.5 rad/s, simulation 2.0 '
                              '(localization limit); 0 = none'),
        DeclareLaunchArgument('controller', default_value='mpc', choices=['mpc', 'reactive'],
                              description='mpc: NMPC (pinky_soccer_mpc); reactive: old P control'),
        DeclareLaunchArgument('wheel_feedback', default_value='false',
                              description='real: NMPC wheel speed feedback from /joint_states'),
        DeclareLaunchArgument('calibration_profile', default_value='',
                              description='real robot profile.yaml for the NMPC model'),
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
