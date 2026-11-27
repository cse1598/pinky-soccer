"""Attack-only game with the existing overhead referee and calibrated sensors."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pinky_soccer_field.field_geometry import Field
from launch_ros.actions import Node
import math


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    sim = arg('mode') == 'mujoco'
    rolling = float(arg('ball_rolling_deceleration'))
    if not math.isfinite(rolling) or rolling < 0:
        raise ValueError('ball_rolling_deceleration must be finite and nonnegative')
    field = Field.from_file(os.path.join(share('pinky_soccer_sim'), 'config', arg('field')+'_field.yaml'))
    def include(package, name, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share(package), 'launch', name)), launch_arguments=args.items())
    common = {k: arg(k) for k in ('mode', 'field', 'team', 'robots', 'robot_name', 'headless',
        'calibration_profile', 'camera_yaml', 'identification', 'robot_host', 'robot_workspace', 'robot_profile',
        'opponent_mode', 'opponent_x', 'opponent_y', 'initial_x', 'initial_y', 'initial_yaw')}
    common.update(controller='attack', strategy=arg('strategy'), opponent_strategy=arg('opponent_strategy'),
                  referee='false', preparation='true', verify='false',
                  wheel_feedback='true', localization='rectangle', imu_enabled='true')
    referee = dict(mode='topic' if sim else 'realsense', use_sim_time='true' if sim else 'false',
                   show_ui='false' if arg('headless') == 'true' else 'true',
                   match_controls='true', stop_on_reacquired_exit='true', robot_namespaces=arg('robot_name'),
                   yellow_goal_y_min=str(-field.spec['goal_width']/2),
                   yellow_goal_y_max=str(field.spec['goal_width']/2),
                   blue_goal_y_min=str(-field.spec['goal_width']/2),
                   blue_goal_y_max=str(field.spec['goal_width']/2))
    if sim:
        referee.update(image_topic='/referee_camera/image_raw',
            camera_info_topic='/referee_camera/camera_info', warmup_frames='3', calibration_frames='3',
            field_length_m=str(field.spec['playing_length']), field_width_m=str(field.spec['playing_width']),
            robot_black_max_channel='100')
        if arg('robots') == '2':
            own = arg('robot_name') or ('blue' if arg('team') == 'B' else 'yellow')
            other = 'yellow' if arg('team') == 'B' else 'blue'
            referee['robot_namespaces'] = own+','+other if arg('opponent_mode') == 'game' else own
    elif arg('robots') != '1':
        raise ValueError('Real robots run one stack per namespace; use referee robot_namespaces for two robots')
    names = referee['robot_namespaces'].split(',')
    actions = [SetEnvironmentVariable('PINKY_SOCCER_BALL_SOURCE', arg('ball_source')),
               SetEnvironmentVariable('PINKY_SOCCER_REFEREE_BALL_ASSIST', arg('referee_ball_assist')),
               SetEnvironmentVariable('PINKY_SOCCER_EXTRA_VIEWS', ','.join(
                   name for name in ('rviz', 'camera', 'localization', 'preparation')
                   if arg('show_'+name) == 'true')),
               SetEnvironmentVariable('PINKY_SOCCER_BALL_ROLLING_DECELERATION', str(rolling) if sim else ''),
               SetEnvironmentVariable('PINKY_SOCCER_OVERHEAD_CAMERA', '1' if sim else '0'),
               include('pinky_soccer_viz', 'game.launch.py', common)]
    if arg('referee_enabled') == 'true':
        actions.append(include('pinky_soccer_referee', 'referee.launch.py', referee))
    if sim and arg('auto_reset_ball') == 'true':
        actions.append(Node(package='pinky_soccer_viz', executable='sim_ball_reset',
            output='screen', parameters=[{'use_sim_time': True, 'robots': names}]))
    return actions


def generate_launch_description():
    defaults = dict(mode='mujoco', field='hardboard', team='Y', robots='1', robot_name='',
        strategy='legacy', opponent_strategy='legacy',
        headless='false', show_rviz='true', show_camera='false',
        show_localization='false', show_preparation='false', auto_reset_ball='true',
        ball_rolling_deceleration='0.10', referee_ball_assist='false', ball_source='auto', opponent_mode='game', opponent_x='auto', opponent_y='auto',
        initial_x='auto', initial_y='auto', initial_yaw='auto', calibration_profile='auto', camera_yaml='', identification='',
        robot_host='', robot_workspace='pinky_soccer_ws', robot_profile='', referee_enabled='true')
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v,
        **({'choices': ['real', 'mujoco']} if k == 'mode' else
           {'choices': ['auto','robot','assist','referee']} if k == 'ball_source' else
           {'choices': ['legacy','path_mpc']} if k in ('strategy','opponent_strategy') else {})) for k,v in defaults.items()),
        OpaqueFunction(function=setup)])
