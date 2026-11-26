"""
One PC entry point: mode real/gazebo/mujoco, RViz, and a separate start signal.

robot_name:=<name> puts our robot's topics under /<name>/ (several robots in one ROS graph),
team:=Y|B selects the goal we defend (Y yellow -X, B blue +X). Simulation only: robots:=2
adds an opponent (opponent_mode game|static|lateral|longitudinal); see sim_field.launch.py.
"""

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
              ('field', 'our_goal_color', 'team', 'robot_name', 'headless', 'verify',
               'controller', 'mpc_backend', 'mpc_damping_scale', 'mpc_observer_mode', 'localization', 'heading_record', 'initial_x', 'initial_y',
               'initial_yaw')}
    common.update(max_lin=str(max_lin), max_ang=str(max_ang),
                  identification=arg('identification'))
    if mode == 'real':
        filename = 'real_game.launch.py'
        profile = arg('calibration_profile')
        common['calibration_profile'] = '' if profile in ('auto', 'none') else profile
        common['wheel_feedback'] = arg('wheel_feedback')
        common.update({name: arg(name) for name in (
            'allow_uncalibrated', 'approximate_hfov_deg', 'camera_yaml',
            'image_rotated_180', 'command_topic')})
        if arg('robots') != '1':
            raise ValueError('robots:=2 is a simulation option; start each real robot with '
                             'its own robot_name and team')
    else:
        filename = 'sim_field.launch.py'
        common.update({name: arg(name) for name in
                       ('ball_x', 'ball_y', 'imu_enabled', 'adc_enabled', 'robots',
                        'opponent_name', 'opponent_mode', 'opponent_speed', 'opponent_travel',
                        'opponent_x', 'opponent_y', 'opponent_yaw', 'opponent_verify')})
        if mode == 'mujoco':
            common['use_gpu'] = arg('use_gpu')
        common.update(simulator=mode, publish_debug_image='true',
                      calibration_profile=arg('calibration_profile'))
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
        DeclareLaunchArgument('team', default_value='', choices=['', 'Y', 'B'],
                              description='goal we defend: Y yellow (-X) | B blue (+X); '
                              'empty: our_goal_color'),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='ROS namespace of our robot (topics /<name>/...); '
                              'empty: none (simulation with robots:=2: team colour)'),
        DeclareLaunchArgument('robots', default_value='1', choices=['1', '2'],
                              description='simulation: 2 adds an opponent robot'),
        DeclareLaunchArgument('opponent_name', default_value=''),
        DeclareLaunchArgument('opponent_mode', default_value='game',
                              choices=['game', 'static', 'lateral', 'longitudinal']),
        DeclareLaunchArgument('opponent_verify', default_value='false', choices=['true', 'false'],
                              description='robots:=2 game opponent: bt_node verify_mode'),
        DeclareLaunchArgument('opponent_speed', default_value='0.10',
                              description='obstacle modes: back-and-forth speed [m/s]'),
        DeclareLaunchArgument('opponent_travel', default_value='0.40',
                              description='obstacle modes: back-and-forth distance [m]'),
        DeclareLaunchArgument('opponent_x', default_value='auto'),
        DeclareLaunchArgument('opponent_y', default_value='auto'),
        DeclareLaunchArgument('opponent_yaw', default_value='auto'),
        DeclareLaunchArgument('use_gpu', default_value='auto',
                              choices=['auto', 'true', 'false', 'nvidia', 'system'],
                              description='MuJoCo renderer only'),
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
        DeclareLaunchArgument('mpc_observer_mode', default_value='off', choices=['off','align','ndob']),
        DeclareLaunchArgument('mpc_damping_scale', default_value='0.0',
                              description='acados GoToPose damping multiplier; 0 disables'),
        DeclareLaunchArgument('mpc_backend', default_value='acados', choices=['acados', 'slsqp']),
        DeclareLaunchArgument('controller', default_value='mpc', choices=['mpc', 'nav2', 'reactive'],
                              description='mpc: NMPC; nav2: native NavFn + MPPI (simulation verify); reactive: old P control'),
        DeclareLaunchArgument('wheel_feedback', default_value='false',
                              description='real: NMPC wheel speed feedback from /joint_states'),
        # Passed explicitly in both modes: included launches inherit this configuration, and
        # an empty value here once switched off the simulators' own calibration profiles.
        DeclareLaunchArgument('identification', default_value='',
                              description='validated target-specific wheel response YAML'),
        DeclareLaunchArgument('calibration_profile', default_value='auto',
                              description='auto: simulators use their own odometry profile, '
                              'real none; or a profile.yaml path; none disables'),
        DeclareLaunchArgument('allow_uncalibrated', default_value='false'),
        DeclareLaunchArgument('approximate_hfov_deg', default_value='66.0'),
        DeclareLaunchArgument('camera_yaml', default_value=''),
        DeclareLaunchArgument('image_rotated_180', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='auto',
                              description='start = kickoff pose; auto: own half facing the '
                              'opponent goal (one robot: centre spot)'),
        DeclareLaunchArgument('initial_y', default_value='auto'),
        DeclareLaunchArgument('initial_yaw', default_value='auto'),
        DeclareLaunchArgument('command_topic', default_value='cmd_vel',
                              description='relative: follows robot_name'),
        DeclareLaunchArgument('ball_x', default_value='auto'),
        DeclareLaunchArgument('ball_y', default_value='auto'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('adc_enabled', default_value='true'),
        OpaqueFunction(function=setup),
    ])
