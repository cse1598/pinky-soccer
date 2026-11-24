"""Simulator + full decision stack on the RCJ field.

Includes the unchanged pinky_soccer_sim launch (world_name:=rcj_soccer.world) and adds
ball_detector, AMCL + goal/compass heading, world_model_node (TF), bt_node, the controller
(controller:=mpc NMPC by default, or reactive)
and both UIs. All use simulation time.

ros2 launch pinky_soccer_viz sim_field.launch.py                       # MuJoCo, viewers
ros2 launch pinky_soccer_viz sim_field.launch.py headless:=true        # no GUI at all
ros2 launch pinky_soccer_viz sim_field.launch.py simulator:=gazebo
ros2 topic pub --once /game/playing std_msgs/msg/Bool "{data: true}"

eval_ball_truth:=true (MuJoCo only) swaps in mujoco_eval_bridge, which also publishes
/sim/ball_ground_truth for scoring and the grey UI overlay (never a decision input).

verify:=true is the behaviour verification mode: bt_node runs single behaviours requested by
the behavior_verifier window instead of the game tree, and ground truth is published for its
overlay (MuJoCo: eval bridge, Gazebo: gazebo_truth_relay on the world pose stream).
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pinky_soccer_sim.field_selection import field_files
from pinky_soccer_world_model.localization_launch import localization_nodes


def lc(name, context):
    return LaunchConfiguration(name).perform(context)


def text(context, value):
    """Resolve a str, Substitution or list of them to a plain string."""
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def load_mujoco_launch(eval_truth):
    """Load the sim package's MuJoCo launch; optionally swap its bridge for the eval one."""
    path = os.path.join(get_package_share_directory('pinky_soccer_sim'), 'launch',
                        'mujoco_soccer.launch.py')
    spec = importlib.util.spec_from_file_location('pinky_soccer_sim_mujoco_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if eval_truth:
        original = module.setup

        def setup(context):
            out = []
            for action in original(context):
                if isinstance(action, Node) and action.node_executable == 'mujoco_soccer_bridge':
                    try:
                        args = [text(context, a) for a in action._Node__arguments]
                        remaps = [(text(context, a), text(context, b))
                                  for a, b in action._Node__remappings]
                        env = {text(context, k): text(context, v)
                               for k, v in action.process_description
                               ._Executable__additional_env or []}
                    except AttributeError as exc:
                        raise RuntimeError('launch_ros internals changed; run with '
                                           'eval_ball_truth:=false') from exc
                    if remaps != [('/cmd_vel', '/drive/cmd_vel'), ('/scan', '/sim/lidar/scan')]:
                        raise RuntimeError(f'Unexpected MuJoCo bridge remaps {remaps}; '
                                           'update sim_field.launch.py')
                    # Parameters mirror mujoco_soccer.launch.py ({'use_sim_time': True}).
                    action = Node(package='pinky_soccer_viz', executable='mujoco_eval_bridge',
                                  name='pinky_mujoco', remappings=remaps, arguments=args,
                                  output='screen', parameters=[{'use_sim_time': True}],
                                  additional_env=env)
                out.append(action)
            return out
        module.setup = setup
    return module.generate_launch_description()


def simulator(context):
    sim = lc('simulator', context)
    headless = lc('headless', context).lower() == 'true'
    world, spec = field_files(lc('field', context))
    common = {'field': lc('field', context), 'ball_x': lc('ball_x', context),
              'ball_y': lc('ball_y', context), 'imu_enabled': lc('imu_enabled', context),
              'adc_enabled': lc('adc_enabled', context)}
    verify = lc('verify', context).lower() == 'true'
    if sim == 'mujoco':
        ld = load_mujoco_launch(lc('eval_ball_truth', context).lower() == 'true' or verify)
        args = dict(common, viewer='false' if headless else 'true')
        return [SetLaunchConfiguration('active_field_spec', str(spec)),
                IncludeLaunchDescription(LaunchDescriptionSource(ld),
                                         launch_arguments=args.items())]
    if sim == 'gazebo':
        path = os.path.join(get_package_share_directory('pinky_soccer_sim'), 'launch',
                            'gazebo_soccer.launch.py')
        args = dict(common, gui='false' if headless else 'true')
        out = [SetLaunchConfiguration('active_field_spec', str(spec)),
               IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=args.items())]
        if verify:
            out += gazebo_truth(world.stem)
        return out
    raise ValueError('simulator must be mujoco or gazebo')


def gazebo_truth(world_name):
    """Robot/ball poses from the Gazebo world pose stream (display/evaluation only)."""
    return [Node(package='pinky_soccer_viz', executable='gazebo_truth_relay',
                 name='gazebo_truth_relay', output='screen',
                 parameters=[{'use_sim_time': True, 'world_name': world_name}])]


def generate_launch_description():
    viz = get_package_share_directory('pinky_soccer_viz')
    wm_cfg = os.path.join(get_package_share_directory('pinky_soccer_world_model'), 'config')
    bt_cfg = os.path.join(get_package_share_directory('pinky_soccer_bt'), 'config')
    ctl_cfg = os.path.join(get_package_share_directory('pinky_soccer'), 'config')
    sim_time = {'use_sim_time': True}
    team = {'our_goal_color': LaunchConfiguration('our_goal_color'),
            'field_spec': LaunchConfiguration('active_field_spec')}
    no_gui = UnlessCondition(LaunchConfiguration('headless'))
    ui = {**team, **sim_time, 'snapshot_dir': LaunchConfiguration('snapshot_dir'),
          'snapshot_period': LaunchConfiguration('snapshot_period')}
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='mujoco', description='mujoco|gazebo'),
        DeclareLaunchArgument('field', default_value='rcj', choices=['rcj', 'hardboard']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('localization', default_value='lidar', choices=['lidar', 'odom_only']),
        DeclareLaunchArgument('heading_record', default_value=''),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('adc_enabled', default_value='false'),
        DeclareLaunchArgument('max_lin', default_value='0',
                              description='extra linear cap [m/s]; 0 = wheel limits only'),
        DeclareLaunchArgument('max_ang', default_value='2.0',
                              description='angular cap [rad/s]; faster spins broke localization '
                              'in MuJoCo; 0 = wheel limits only'),
        DeclareLaunchArgument('controller', default_value='mpc', choices=['mpc', 'reactive']),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('eval_ball_truth', default_value='false'),
        DeclareLaunchArgument('verify', default_value='false',
                              description='behaviour verification window + bt_node verify_mode'),
        DeclareLaunchArgument('ball_x', default_value='0.3'),
        DeclareLaunchArgument('ball_y', default_value='0.0'),
        DeclareLaunchArgument('our_goal_color', default_value='yellow'),
        DeclareLaunchArgument('snapshot_dir', default_value=''),
        DeclareLaunchArgument('snapshot_period', default_value='0.0'),
        OpaqueFunction(function=simulator),
        Node(package='pinky_soccer_world_model', executable='ball_detector',
             name='ball_detector', output='screen',
             parameters=[os.path.join(wm_cfg, 'ball_detector.yaml'), sim_time,
                         {'publish_debug_image': ParameterValue(
                             LaunchConfiguration('publish_debug_image'), value_type=bool)}]),
        OpaqueFunction(function=localization),
        Node(package='pinky_soccer_bt', executable='bt_node', name='bt_node', output='screen',
             parameters=[os.path.join(bt_cfg, 'bt_node.yaml'), sim_time,
                         {'verify_mode': ParameterValue(LaunchConfiguration('verify'),
                                                        value_type=bool)}]),
        Node(package='pinky_soccer_viz', executable='behavior_verifier',
             name='behavior_verifier', output='screen',
             condition=IfCondition(LaunchConfiguration('verify')),
             parameters=[team, sim_time, {
                 'simulation': True, 'mode_label': LaunchConfiguration('simulator'),
                 'controller_config': os.path.join(ctl_cfg, 'reactive_controller.yaml'),
                 'max_lin': ParameterValue(LaunchConfiguration('max_lin'), value_type=float),
                 'max_ang': ParameterValue(LaunchConfiguration('max_ang'), value_type=float),
                 'controller_kind': LaunchConfiguration('controller')}]),
        OpaqueFunction(function=controller),
        Node(package='pinky_soccer_viz', executable='world_model_viz', name='world_model_viz',
             output='screen', parameters=[team, sim_time]),
        Node(package='pinky_soccer_viz', executable='world_model_topview',
             name='world_model_topview', output='screen', parameters=[ui]),
        Node(package='rviz2', executable='rviz2', name='rviz2', condition=no_gui,
             arguments=['-d', os.path.join(viz, 'rviz', 'soccer.rviz')],
             parameters=[sim_time]),
        Node(package='pinky_soccer_viz', executable='game_camera_view',
             name='game_camera_view', condition=no_gui, output='screen', parameters=[sim_time]),
    ])


def caps(context):
    """Return (max_lin, max_ang); 0 means no cap beyond the wheel limits."""
    max_lin, max_ang = float(lc('max_lin', context)), float(lc('max_ang', context))
    if not (0 <= max_lin <= 0.30 and 0 <= max_ang <= 6.0):
        raise ValueError('Require 0 <= max_lin <= 0.30 m/s and 0 <= max_ang <= 6 rad/s')
    return max_lin, max_ang


def controller(context):
    """NMPC (default) or the old proportional controller, both BehaviorGoal -> /cmd_vel."""
    max_lin, max_ang = caps(context)
    kind = lc('controller', context)
    sim = lc('simulator', context)
    if kind == 'mpc':
        profile = os.path.join(get_package_share_directory('pinky_soccer_sim'), 'config',
                               'calibration', f'{sim}_profile.yaml')
        return [Node(package='pinky_soccer_mpc', executable='mpc_controller',
                     name='mpc_controller', output='screen', parameters=[{
                         'use_sim_time': True, 'target': sim,
                         'calibration_profile': profile if os.path.exists(profile) else '',
                         'max_lin': max_lin, 'max_ang': max_ang,
                         'wheel_feedback': True}])]
    if kind != 'reactive':
        raise ValueError('controller must be mpc or reactive')
    cfg = os.path.join(get_package_share_directory('pinky_soccer'), 'config',
                       'reactive_controller.yaml')
    return [Node(package='pinky_soccer', executable='reactive_controller',
                 name='reactive_controller', output='screen',
                 parameters=[cfg, {'use_sim_time': True, 'max_lin': max_lin or 0.25,
                                   'max_ang': max_ang or 2.0}])]


def localization(context):
    """Exactly one map->odom authority in either localization mode."""
    _, spec = field_files(lc('field', context))
    mode = lc('localization', context)
    initial = tuple(float(lc(k, context)) for k in ('initial_x', 'initial_y', 'initial_yaw'))
    out = localization_nodes(spec, True, initial, heading_record=lc('heading_record', context)) \
        if mode == 'lidar' else []
    cfg = os.path.join(get_package_share_directory('pinky_soccer_world_model'), 'config')
    out.append(Node(package='pinky_soccer_world_model', executable='world_model_node',
                    name='world_model_node', output='screen', parameters=[
                        os.path.join(cfg, 'world_model.yaml'), os.path.join(cfg, 'ball_filter.yaml'),
                        {'use_sim_time': True, 'field_spec': str(spec),
                         'our_goal_color': lc('our_goal_color', context),
                         'localization': 'tf' if mode == 'lidar' else 'odom_only',
                         'localization_pose_timeout': 2.0 if mode == 'lidar' else 0.0,
                         'initial_pose': list(initial), 'playing_default': False}]))
    return out
