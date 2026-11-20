"""Simulator + full decision stack on the RCJ field.

Includes the unchanged pinky_soccer_sim launch (world_name:=rcj_soccer.world) and adds
ball_detector, world_model_node (localization:=odom_only), bt_node, reactive_controller
and both UIs. All use simulation time.

ros2 launch pinky_soccer_viz sim_field.launch.py                       # MuJoCo, viewers
ros2 launch pinky_soccer_viz sim_field.launch.py headless:=true        # no GUI at all
ros2 launch pinky_soccer_viz sim_field.launch.py simulator:=gazebo
ros2 topic pub --once /game/playing std_msgs/msg/Bool "{data: true}"

eval_ball_truth:=true (MuJoCo only) swaps in mujoco_eval_bridge, which also publishes
/sim/ball_ground_truth for scoring and the grey UI overlay (never a decision input).
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def lc(name, context):
    return LaunchConfiguration(name).perform(context)


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
                        args = action._Node__arguments
                        remaps = action._Node__remappings
                        params = action._Node__parameters
                        env = action.process_description._Executable__additional_env
                    except AttributeError as exc:
                        raise RuntimeError('launch_ros internals changed; run with '
                                           'eval_ball_truth:=false') from exc
                    action = Node(package='pinky_soccer_viz', executable='mujoco_eval_bridge',
                                  name='pinky_mujoco', remappings=remaps, arguments=args,
                                  output='screen', parameters=params,
                                  additional_env=dict(env or {}))
                out.append(action)
            return out
        module.setup = setup
    return module.generate_launch_description()


def simulator(context):
    sim = lc('simulator', context)
    headless = lc('headless', context).lower() == 'true'
    common = {'world_name': 'rcj_soccer.world', 'ball_x': lc('ball_x', context),
              'ball_y': lc('ball_y', context)}
    if sim == 'mujoco':
        ld = load_mujoco_launch(lc('eval_ball_truth', context).lower() == 'true')
        args = dict(common, viewer='false' if headless else 'true')
        return [IncludeLaunchDescription(LaunchDescriptionSource(ld),
                                         launch_arguments=args.items())]
    if sim == 'gazebo':
        path = os.path.join(get_package_share_directory('pinky_soccer_sim'), 'launch',
                            'gazebo_soccer.launch.py')
        args = dict(common, gui='false' if headless else 'true')
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                         launch_arguments=args.items())]
    raise ValueError('simulator must be mujoco or gazebo')


def generate_launch_description():
    viz = get_package_share_directory('pinky_soccer_viz')
    wm_cfg = os.path.join(get_package_share_directory('pinky_soccer_world_model'), 'config')
    bt_cfg = os.path.join(get_package_share_directory('pinky_soccer_bt'), 'config')
    ctl_cfg = os.path.join(get_package_share_directory('pinky_soccer'), 'config')
    sim_time = {'use_sim_time': True}
    team = {'our_goal_color': LaunchConfiguration('our_goal_color')}
    no_gui = UnlessCondition(LaunchConfiguration('headless'))
    ui = {**team, **sim_time, 'snapshot_dir': LaunchConfiguration('snapshot_dir'),
          'snapshot_period': LaunchConfiguration('snapshot_period')}
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='mujoco', description='mujoco|gazebo'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('eval_ball_truth', default_value='false'),
        DeclareLaunchArgument('ball_x', default_value='0.3'),
        DeclareLaunchArgument('ball_y', default_value='0.0'),
        DeclareLaunchArgument('our_goal_color', default_value='yellow'),
        DeclareLaunchArgument('snapshot_dir', default_value=''),
        DeclareLaunchArgument('snapshot_period', default_value='0.0'),
        OpaqueFunction(function=simulator),
        Node(package='pinky_soccer_world_model', executable='ball_detector',
             name='ball_detector', output='screen',
             parameters=[os.path.join(wm_cfg, 'ball_detector.yaml'), sim_time]),
        Node(package='pinky_soccer_world_model', executable='world_model_node',
             name='world_model_node', output='screen',
             parameters=[os.path.join(wm_cfg, 'world_model.yaml'),
                         os.path.join(wm_cfg, 'ball_filter.yaml'), sim_time, team,
                         {'localization': 'odom_only'}]),
        Node(package='pinky_soccer_bt', executable='bt_node', name='bt_node', output='screen',
             parameters=[os.path.join(bt_cfg, 'bt_node.yaml'), sim_time]),
        Node(package='pinky_soccer', executable='reactive_controller',
             name='reactive_controller', output='screen',
             parameters=[os.path.join(ctl_cfg, 'reactive_controller.yaml'), sim_time]),
        Node(package='pinky_soccer_viz', executable='world_model_viz', name='world_model_viz',
             output='screen', parameters=[team, sim_time]),
        Node(package='pinky_soccer_viz', executable='world_model_topview',
             name='world_model_topview', output='screen', parameters=[ui]),
        Node(package='rviz2', executable='rviz2', name='rviz2', condition=no_gui,
             arguments=['-d', os.path.join(viz, 'rviz', 'soccer.rviz')],
             parameters=[sim_time]),
    ])
