"""PC perception, world model, BT and low-speed control for an already running robot.

robot_name:=<name> must match the robot's bringup (pinky_soccer_bringup robot.launch.py
robot_name:=<name>): all topics are then /<name>/..., with its own /tf. team:=Y|B is the goal
we defend. The referee (referee:=true) judges goals from this robot's world model ball estimate
and runs the kickoff cycle; with two robots on one network start it on one PC only.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pinky_soccer_sim import multi_robot
from pinky_soccer_sim.field_selection import field_files
from pinky_soccer_viz.robot_stack import decision_stack, RobotConfig


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

    _, field_spec = field_files(arg('field'))
    color = multi_robot.team_color(arg('team'), arg('our_goal_color'))
    name = multi_robot.check_name(arg('robot_name'))
    max_lin, max_ang = number('max_lin'), number('max_ang')
    hfov = number('approximate_hfov_deg')
    if not (0 <= max_lin <= 0.30 and 0 <= max_ang <= 6.0):
        raise ValueError('Require 0 <= max_lin <= 0.30 m/s and 0 <= max_ang <= 6 rad/s; '
                         '0 uses MPC internal limits only')
    if not 1 < hfov < 179:
        raise ValueError('approximate_hfov_deg must be between 1 and 179')
    default = multi_robot.kickoff_pose(color, False)
    initial = tuple(default[i] if arg(k) == 'auto' else number(k)
                    for i, k in enumerate(('initial_x', 'initial_y', 'initial_yaw')))
    nodes = multi_robot.dds_environment() + [LogInfo(
        msg=f'REAL ROBOT {name or "(no namespace)"} team {color}: starts paused. '+
        ('Receive sensors until preparation/prepare (Trigger) or R; '
         'calibrate, GoToPose to team start, then wait READY. preparation/stop stops motion. '
         if arg('localization') == 'rectangle' else
         'Place robot at its kickoff pose BEFORE launch. ')+
        'preparation/start (or /game/playing=true) is accepted after READY; preparation/stop stops.')]
    if arg('robot_host'):
        if arg('robot_host') == 'local':
            from launch.actions import IncludeLaunchDescription
            from launch.launch_description_sources import PythonLaunchDescriptionSource
            from pinky_soccer_viz.robot_stack import share
            nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                share('pinky_soccer_bringup', 'launch', 'robot.launch.py')),
                launch_arguments={'robot_name': name, 'imu_enabled': 'true',
                                  'led_enabled': 'true', 'calibration_profile': arg('robot_profile')}.items()))
        else:
            nodes.append(Node(package='pinky_soccer_viz', executable='remote_robot',
                name='remote_robot', output='screen', arguments=['--host', arg('robot_host'),
                    '--workspace', arg('robot_workspace'), '--name', name,
                    '--profile', arg('robot_profile')]))
    nodes += decision_stack(RobotConfig(
        preparation=flag('preparation'), preparation_config=arg('preparation_config'),
        name=name, color=color, initial=initial, simulation=False, target='real',
        field_spec=str(field_spec), localization=arg('localization'),
        localization_config=arg('localization_config'),
        localization_debug=flag('localization_debug'),
        controller=arg('controller'), attack_strategy=arg('strategy'), mpc_backend=arg('mpc_backend'),
        mpc_damping_scale=float(arg('mpc_damping_scale')),
        mpc_observer_mode=arg('mpc_observer_mode'), max_lin=max_lin, max_ang=max_ang,
        calibration_profile=arg('calibration_profile'),
        ball_offset_source=arg('ball_offset_source'),
        identification=arg('identification'),
        wheel_feedback=flag('wheel_feedback'), verify=flag('verify'), ui=True,
        headless=flag('headless'), groot2=False, heading_record=arg('heading_record'),
        command_topic=arg('command_topic'),
        real={'allow_uncalibrated': flag('allow_uncalibrated'), 'approximate_hfov_deg': hfov,
              'camera_yaml': arg('camera_yaml'),
              'image_rotated_180': flag('image_rotated_180')}))
    if flag('verify'):
        nodes.append(LogInfo(msg='VERIFY MODE: the game tree is off; behaviours run only from '
                             'the verifier window. Keep a hand on the robot power switch.'))
    if flag('referee'):
        nodes.append(Node(package='pinky_soccer_viz', executable='game_referee',
                          name='game_referee', output='screen', parameters=[{
                              'use_sim_time': False, 'field_spec': str(field_spec),
                              'source': 'world_model', 'reset_ball': False,
                              'playing_input_topic': multi_robot.topic(name, 'preparation/playing')
                                  if flag('preparation') and arg('localization') == 'rectangle' else '/game/playing',
                              'world_model_topics': [multi_robot.topic(name, 'world_model')]}]))
    return nodes


def generate_launch_description():
    """No simulator/sensor drivers; rectangle startup owns the initial slow spin."""
    return LaunchDescription([
        DeclareLaunchArgument('preparation', default_value='true'),
        DeclareLaunchArgument('preparation_config', default_value=''),
        DeclareLaunchArgument('field', default_value='hardboard', choices=['rcj', 'hardboard']),
        DeclareLaunchArgument('robot_host', default_value='', description='SSH user@host; local for on-robot stack; empty for existing external bringup'),
        DeclareLaunchArgument('robot_workspace', default_value='pinky-soccer'),
        DeclareLaunchArgument('robot_profile', default_value=''),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='namespace; must match the robot bringup'),
        DeclareLaunchArgument('team', default_value='', choices=['', 'Y', 'B'],
                              description='goal we defend: Y yellow (-X) | B blue (+X)'),
        DeclareLaunchArgument('our_goal_color', default_value='yellow',
                              choices=['yellow', 'blue']),
        DeclareLaunchArgument('allow_uncalibrated', default_value='false'),
        DeclareLaunchArgument('approximate_hfov_deg', default_value='66.0'),
        DeclareLaunchArgument('camera_yaml', default_value=''),
        DeclareLaunchArgument('image_rotated_180', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='auto',
                              description='kickoff pose (auto: centre spot facing the '
                              'opponent goal)'),
        DeclareLaunchArgument('initial_y', default_value='auto'),
        DeclareLaunchArgument('initial_yaw', default_value='auto'),
        DeclareLaunchArgument('max_lin', default_value='0'),
        DeclareLaunchArgument('max_ang', default_value='0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('localization', default_value='rectangle',
                              choices=['rectangle', 'lidar', 'odom_only']),
        DeclareLaunchArgument('localization_config', default_value='auto'),
        DeclareLaunchArgument('localization_debug', default_value='false'),
        DeclareLaunchArgument('ball_offset_source', default_value='auto'),
        DeclareLaunchArgument('heading_record', default_value=''),
        DeclareLaunchArgument('mpc_observer_mode', default_value='off', choices=['off','align','ndob']),
        DeclareLaunchArgument('mpc_damping_scale', default_value='0.0',
                              description='acados GoToPose damping multiplier; 0 disables'),
        DeclareLaunchArgument('mpc_backend', default_value='acados', choices=['acados', 'slsqp']),
        DeclareLaunchArgument('strategy', default_value='legacy', choices=['legacy','path_mpc']),
        DeclareLaunchArgument('controller', default_value='mpc', choices=['mpc', 'reactive', 'attack']),
        DeclareLaunchArgument('wheel_feedback', default_value='false',
                              description='NMPC uses /joint_states wheel speeds (needs the '
                              'right wheel sign -1 confirmed; open-loop estimate otherwise)'),
        DeclareLaunchArgument('identification', default_value='',
                              description='empty/auto: pinky-pro measured tau; none: original tau; '
                              'or mpc_motor_response_schema YAML'),
        DeclareLaunchArgument('calibration_profile', default_value='',
                              description='robot profile.yaml for the NMPC model (empty: '
                              'official defaults, marked unmeasured)'),
        DeclareLaunchArgument('verify', default_value='false',
                              description='behaviour verification window + bt_node verify_mode'),
        DeclareLaunchArgument('referee', default_value='true',
                              description='goal detection + kickoff cycle; one per game'),
        DeclareLaunchArgument('command_topic', default_value='cmd_vel',
                              description='Override for a non-driving dry run (relative: '
                              'inside robot_name)'),
        OpaqueFunction(function=setup),
    ])
