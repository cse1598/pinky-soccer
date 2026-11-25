"""Simulator + full decision stack, one or two robots, with a referee.

Includes the pinky_soccer_sim MuJoCo/Gazebo launch and adds, per playing robot, ball_detector,
AMCL + goal/compass heading, world_model_node, bt_node and the controller (NMPC by default),
plus the UIs for our robot and the referee (goal -> kickoff positions -> ball back to the
centre spot). All use simulation time.

ros2 launch pinky_soccer_viz sim_field.launch.py                       # MuJoCo, one robot
ros2 launch pinky_soccer_viz sim_field.launch.py robots:=2             # + opponent playing
ros2 launch pinky_soccer_viz sim_field.launch.py robots:=2 opponent_mode:=lateral \
    opponent_speed:=0.15 opponent_travel:=0.5                         # moving obstacle
ros2 launch pinky_soccer_viz sim_field.launch.py simulator:=gazebo team:=B
ros2 topic pub --once /game/playing std_msgs/msg/Bool "{data: true}"

Names: robot_name / opponent_name are the ROS namespaces (topics /<name>/..., own /tf).
With one robot the default is no namespace (as before); with two robots the defaults are
the team colours (yellow, blue). team:=Y|B is the goal we defend (Y = yellow = -X).

opponent_mode (robots:=2): game (plays with its own full stack), static (stands still),
lateral (placed sideways, drives left-right), longitudinal (faces us, drives back and
forth); opponent_speed [m/s] and opponent_travel [m] set the back-and-forth motion.

verify:=true is the behaviour verification mode for our robot (pinky_soccer_viz VERIFY.md).
Ground truth (/<name>/ground_truth, /sim/ball_ground_truth) is published for the referee and
the UI overlays only, never as a decision input.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pinky_soccer_sim import multi_robot
from pinky_soccer_sim.field_selection import field_files
from pinky_soccer_viz.robot_stack import decision_stack, RobotConfig

OPPONENT_MODES = ('game', 'static', 'lateral', 'longitudinal')


def lc(name, context):
    return LaunchConfiguration(name).perform(context)


def flag(name, context):
    value = lc(name, context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')
    return value == 'true'


def pose_arg(prefix, context, default):
    """(x, y, yaw) from <prefix>x/y/yaw arguments; 'auto' entries use the default."""
    values = [lc(prefix + k, context) for k in ('x', 'y', 'yaw')]
    return tuple(default[i] if v == 'auto' else float(v) for i, v in enumerate(values))


def caps(context):
    """Return (max_lin, max_ang); 0 means no cap beyond the wheel limits."""
    max_lin, max_ang = float(lc('max_lin', context)), float(lc('max_ang', context))
    if not (0 <= max_lin <= 0.30 and 0 <= max_ang <= 6.0):
        raise ValueError('Require 0 <= max_lin <= 0.30 m/s and 0 <= max_ang <= 6 rad/s')
    return max_lin, max_ang


def setup(context):
    sim = lc('simulator', context)
    if sim not in ('mujoco', 'gazebo'):
        raise ValueError('simulator must be mujoco or gazebo')
    robots = int(lc('robots', context))
    if robots not in (1, 2):
        raise ValueError('robots must be 1 or 2')
    mode = lc('opponent_mode', context)
    if mode not in OPPONENT_MODES:
        raise ValueError(f'opponent_mode must be one of {OPPONENT_MODES}')
    world, spec = field_files(lc('field', context))
    color = multi_robot.team_color(lc('team', context), lc('our_goal_color', context))
    two = robots == 2
    name = multi_robot.check_name(lc('robot_name', context) or (color if two else ''))
    opponent = multi_robot.check_name(lc('opponent_name', context)
                                      or multi_robot.other(color)) if two else ''
    if two and name == opponent:
        raise ValueError('robot_name and opponent_name must differ')
    # Kickoff layout also with one robot: 0.4 m behind the centre spot, where the referee puts
    # the ball back after a goal (a robot kicking off from the centre drove into the ball).
    ours = pose_arg('initial_', context, multi_robot.kickoff_pose(color, True))
    other = multi_robot.other(color)
    opp_pose = pose_arg('opponent_', context, multi_robot.kickoff_pose(other, True)
                        if mode == 'game' else multi_robot.obstacle_pose(color, mode))
    ball = (0.0, 0.0)
    ball = tuple(ball[i] if lc(k, context) == 'auto' else float(lc(k, context))
                 for i, k in enumerate(('ball_x', 'ball_y')))
    headless = flag('headless', context)
    max_lin, max_ang = caps(context)

    # ---------------------------------------------------------------- simulator
    common = {'field': lc('field', context), 'ball_x': str(ball[0]), 'ball_y': str(ball[1]),
              'imu_enabled': lc('imu_enabled', context),
              'adc_enabled': lc('adc_enabled', context),
              'namespace': name, 'x': str(ours[0]), 'y': str(ours[1]), 'yaw': str(ours[2])}
    if two:
        common.update({'opponent_name': opponent, 'opponent_x': str(opp_pose[0]),
                       'opponent_y': str(opp_pose[1]), 'opponent_yaw': str(opp_pose[2]),
                       'opponent_sensors': 'true' if mode == 'game' else 'false'})
    package = get_package_share_directory('pinky_soccer_sim')
    # Own odometry profile of this simulator by default. Always passed explicitly: an included
    # launch inherits the parent's configurations, and an inherited empty value disabled it.
    own = os.path.join(package, 'config', 'calibration', f'{sim}_profile.yaml')
    profile = lc('calibration_profile', context)
    if profile in ('', 'auto'):
        profile = own if os.path.exists(own) else 'none'
    common['calibration_profile'] = profile
    if sim == 'mujoco':
        # The eval bridge adds /sim/ball_ground_truth and /sim/set_ball (referee reset).
        common.update(viewer='false' if headless else 'true',
                      use_gpu=lc('use_gpu', context),
                      bridge_package='pinky_soccer_viz', bridge_executable='mujoco_eval_bridge')
    else:
        common.update(gui='false' if headless else 'true')
    out = multi_robot.dds_environment() + [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package, 'launch', f'{sim}_soccer.launch.py')),
        launch_arguments=common.items())]
    if sim == 'gazebo':
        models = [name or 'pinky'] + ([opponent] if two else [])
        topics = [multi_robot.topic(n, 'ground_truth')
                  for n in [name] + ([opponent] if two else [])]
        out.append(Node(package='pinky_soccer_viz', executable='gazebo_truth_relay',
                        name='gazebo_truth_relay', output='screen', parameters=[{
                            'use_sim_time': True, 'world_name': world.stem,
                            'models': models, 'topics': topics}]))

    # ---------------------------------------------------------------- robots
    base = {'simulation': True, 'target': sim, 'field_spec': str(spec),
            'localization': lc('localization', context),
            'controller': lc('controller', context), 'mpc_backend': lc('mpc_backend', context),
             'mpc_damping_scale': float(lc('mpc_damping_scale', context)),
             'mpc_observer_mode': lc('mpc_observer_mode', context), 'max_lin': max_lin, 'max_ang': max_ang,
            'calibration_profile': '' if profile == 'none' else profile,
            'identification': lc('identification', context),
            'wheel_feedback': True, 'headless': headless,
            'publish_debug_image': flag('publish_debug_image', context)}
    out += decision_stack(RobotConfig(
        name=name, color=color, initial=ours, verify=flag('verify', context), ui=True,
        heading_record=lc('heading_record', context),
        opponent_truth=multi_robot.topic(opponent, 'ground_truth') if two else '',
        # Obstacle opponents run without simulated sensors: nothing to show.
        camera_robots=(name, opponent) if two and mode == 'game' else (), **base))
    if two and mode == 'game':
        out += decision_stack(RobotConfig(
            name=opponent, color=other, initial=opp_pose, ui=False, groot2=False, **base))
    elif two:
        out += multi_robot.group(opponent, [Node(
            package='pinky_soccer_viz', executable='obstacle_driver', name='obstacle_driver',
            output='screen', parameters=[{
                'use_sim_time': True, 'mode': mode,
                'speed': float(lc('opponent_speed', context)),
                'travel': float(lc('opponent_travel', context)),
                'pose_topic': multi_robot.topic(opponent, 'ground_truth')}])])

    # ---------------------------------------------------------------- referee
    out.append(Node(package='pinky_soccer_viz', executable='game_referee', name='game_referee',
                    output='screen', parameters=[{
                        'use_sim_time': True, 'field_spec': str(spec), 'source': 'truth',
                        'reset_ball': True}]))
    return out


def generate_launch_description():
    auto = 'auto'
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='mujoco', description='mujoco|gazebo'),
        DeclareLaunchArgument('field', default_value='rcj', choices=['rcj', 'hardboard']),
        DeclareLaunchArgument('robots', default_value='1', choices=['1', '2']),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='our namespace (default: none for 1 robot, team '
                              'colour for 2)'),
        DeclareLaunchArgument('team', default_value='', choices=['', 'Y', 'B'],
                              description='goal we defend: Y yellow (-X) or B blue (+X); '
                              'empty: our_goal_color'),
        DeclareLaunchArgument('our_goal_color', default_value='yellow',
                              choices=['yellow', 'blue']),
        DeclareLaunchArgument('opponent_name', default_value='',
                              description='opponent namespace (default: the other colour)'),
        DeclareLaunchArgument('opponent_mode', default_value='game',
                              choices=list(OPPONENT_MODES)),
        DeclareLaunchArgument('opponent_speed', default_value='0.10',
                              description='obstacle back-and-forth speed [m/s]'),
        DeclareLaunchArgument('opponent_travel', default_value='0.40',
                              description='obstacle back-and-forth distance [m]'),
        DeclareLaunchArgument('opponent_x', default_value=auto),
        DeclareLaunchArgument('opponent_y', default_value=auto),
        DeclareLaunchArgument('opponent_yaw', default_value=auto),
        DeclareLaunchArgument('use_gpu', default_value='auto',
                              choices=['auto', 'true', 'false', 'nvidia', 'system']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('localization', default_value='lidar',
                              choices=['lidar', 'odom_only']),
        DeclareLaunchArgument('heading_record', default_value=''),
        DeclareLaunchArgument('initial_x', default_value=auto,
                              description='our start = kickoff pose (auto: own half, 0.4 m '
                              'behind the centre spot, facing the opponent goal)'),
        DeclareLaunchArgument('initial_y', default_value=auto),
        DeclareLaunchArgument('initial_yaw', default_value=auto),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('adc_enabled', default_value='false'),
        DeclareLaunchArgument('max_lin', default_value='0',
                              description='extra linear cap [m/s]; 0 = wheel limits only'),
        DeclareLaunchArgument('max_ang', default_value='2.0',
                              description='angular cap [rad/s]; faster spins broke localization '
                              'in MuJoCo; 0 = wheel limits only'),
        DeclareLaunchArgument('mpc_observer_mode', default_value='off', choices=['off','align','ndob']),
        DeclareLaunchArgument('mpc_damping_scale', default_value='0.0',
                              description='acados GoToPose damping multiplier; 0 disables'),
        DeclareLaunchArgument('mpc_backend', default_value='acados', choices=['acados', 'slsqp']),
        DeclareLaunchArgument('controller', default_value='mpc', choices=['mpc', 'reactive']),
        DeclareLaunchArgument('identification', default_value='',
                              description='validated simulator wheel response YAML'),
        DeclareLaunchArgument('calibration_profile', default_value='auto',
                              description="auto: the simulator's own odometry profile "
                              "(pinky_soccer_sim config/calibration); 'none' disables"),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('eval_ball_truth', default_value='true',
                              description='kept for compatibility: ball truth is always on'),
        DeclareLaunchArgument('verify', default_value='false',
                              description='behaviour verification window + bt_node verify_mode'),
        DeclareLaunchArgument('ball_x', default_value=auto, description='auto: centre spot'),
        DeclareLaunchArgument('ball_y', default_value=auto),
        OpaqueFunction(function=setup),
    ])
