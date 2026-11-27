"""
PC decision stack of one robot, optionally in its own namespace (launch helper).

Nodes: ball_detector, localization (map_server, AMCL, goal_heading) or odom-only,
world_model_node, bt_node, the controller (NMPC or reactive) and, for the robot shown on
this PC, the UIs (RViz, top view, markers, camera view, behaviour verifier).

Namespaced robots (pinky_soccer_sim.multi_robot.group): relative topics, own /tf. Nodes
that use absolute names get remaps here: goal_heading (camera/IMU/AMCL topics), RViz
(/world_model/* displays); /game/playing and /sim/ball_ground_truth stay global.
"""

from dataclasses import dataclass, field, replace
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from pinky_soccer_sim import multi_robot
from pinky_soccer_world_model.localization_launch import localization_nodes

# goal_heading_node and RViz use absolute names; inside a namespace they must follow it.
GOAL_HEADING_TOPICS = ('initialpose', 'request_nomotion_update',
                       'localization/goal_image/compressed', 'localization/status',
                       'camera/camera_info', 'camera/image_raw/compressed', 'imu_raw',
                       'imu/mag', 'imu/status', 'amcl_pose')
RVIZ_TOPICS = ('world_model/search_grid', 'world_model/search_grid_markers',
               'world_model/field_markers', 'world_model/markers', 'world_model/topview')
GLOBAL_TOPICS = ('game/playing', 'sim/ball_ground_truth')


def share(package, *parts):
    """Path inside a package share directory."""
    return os.path.join(get_package_share_directory(package), *parts)


def params_file(path, name):
    """
    Return a parameter file usable inside namespace `name`.

    Keys like `bt_node:` match only the node in the root namespace, so in a namespace the
    file would be silently ignored. Returns a copy with keys `/<name>/<node>:` (or the
    original path without a namespace).
    """
    if not name:
        return path
    import yaml
    data = yaml.safe_load(Path(path).read_text()) or {}
    rewritten = {(k if str(k).startswith('/') else f'/{name}/{k}'): v for k, v in data.items()}
    out = Path(tempfile.mkdtemp(prefix=f'pinky_params_{name}_')) / Path(path).name
    out.write_text(yaml.safe_dump(rewritten))
    return str(out)


@dataclass
class RobotConfig:
    """One robot's decision stack settings."""

    preparation: bool = True
    preparation_config: str = ''
    name: str = ''                       # namespace ('' = none)
    color: str = 'yellow'                # goal we defend
    initial: tuple = (0.0, 0.0, 0.0)     # start = kickoff pose (map frame)
    simulation: bool = True
    target: str = 'mujoco'               # dynamics target: real | mujoco | gazebo
    field_spec: str = ''
    localization: str = 'rectangle'
    localization_config: str = 'auto'
    localization_debug: bool = False
    controller: str = 'mpc'
    mpc_backend: str = 'acados'
    mpc_damping_scale: float = 0.0
    mpc_observer_mode: str = 'off'
    max_lin: float = 0.0
    max_ang: float = 0.0
    calibration_profile: str = ''
    ball_offset_source: str = 'auto'
    identification: str = ''
    wheel_feedback: bool = True
    verify: bool = False
    bt_tree_file: str = ''              # optional tree, otherwise existing match strategy
    ui: bool = True                      # RViz/top view/camera/verifier for this robot
    headless: bool = False
    groot2: bool = True
    heading_record: str = ''
    publish_debug_image: bool = True
    camera_robots: tuple = ()          # other robots selectable in the camera window
    camera_raw_only: tuple = ()        # ... of which these have only sim/camera/image_raw
    command_topic: str = 'cmd_vel'
    opponent_truth: str = ''             # simulation: other robot's ground truth topic
    real: dict = field(default_factory=dict)   # real-robot camera/uncalibrated options


def remaps(cfg):
    """Group remaps for absolute names (only needed in a namespace)."""
    if not cfg.name:
        return []
    out = [SetRemap(f'/{t}', t) for t in GOAL_HEADING_TOPICS + RVIZ_TOPICS]
    out += [SetRemap(t, f'/{t}') for t in GLOBAL_TOPICS]
    return out


def controller_node(cfg):
    """NMPC (default) or the old proportional controller."""
    if cfg.controller == 'mpc':
        return Node(package='pinky_soccer_mpc', executable='mpc_controller',
                    name='mpc_controller', output='screen',
                    remappings=[('cmd_vel', cfg.command_topic)], parameters=[{
                        'use_sim_time': cfg.simulation, 'target': cfg.target,
                        'calibration_profile': cfg.calibration_profile,
                        'identification': cfg.identification, 'mpc_backend': cfg.mpc_backend,
                        'mpc_damping_scale': float(cfg.mpc_damping_scale),
                        'mpc_observer_mode': cfg.mpc_observer_mode,
                        'max_lin': cfg.max_lin, 'max_ang': cfg.max_ang,
                        'wheel_feedback': cfg.wheel_feedback,
                        'motion_management': cfg.target != 'gazebo',
                        # GoToPose avoid_walls uses the same field as the world model.
                        'field_spec': cfg.field_spec}])
    if cfg.controller != 'reactive':
        raise ValueError('controller must be mpc or reactive')
    return Node(package='pinky_soccer', executable='reactive_controller',
                name='reactive_controller', output='screen',
                remappings=[('cmd_vel', cfg.command_topic)],
                parameters=[params_file(share('pinky_soccer', 'config',
                                              'reactive_controller.yaml'), cfg.name),
                            {'use_sim_time': cfg.simulation,
                             'max_lin': min(cfg.max_lin or 0.25, 0.25),
                             'max_ang': min(cfg.max_ang or 2.0, 2.0)}])


def decision_stack(cfg):
    """Return the launch actions of one robot's decision stack."""
    clock = {'use_sim_time': cfg.simulation}
    team = {'our_goal_color': cfg.color, 'field_spec': cfg.field_spec}
    wm = share('pinky_soccer_world_model', 'config')
    real = cfg.real
    rectangle = cfg.localization == 'rectangle'
    preparation = cfg.preparation and rectangle
    lidar = cfg.localization in ('lidar', 'rectangle')
    nodes = [
        Node(package='pinky_soccer_world_model', executable='ball_detector',
             name='ball_detector', output='screen', parameters=[
                 params_file(os.path.join(wm, 'ball_detector.yaml'), cfg.name), clock,
                 {'publish_debug_image': cfg.publish_debug_image,
                  'ball_offset_source': ((cfg.calibration_profile or real.get('camera_yaml', ''))
                      if cfg.ball_offset_source == 'auto' else cfg.ball_offset_source),
                  **{k: real[k] for k in ('allow_uncalibrated', 'approximate_hfov_deg',
                                          'camera_yaml', 'image_rotated_180') if k in real}}]),
    ]
    if rectangle:
        if cfg.controller == 'nav2':
            raise ValueError('Nav2 comparison uses localization:=lidar; rectangle is the startup MPC path')
        localization_params = [os.path.join(wm, 'rectangle_localization.yaml')]
        if cfg.localization_config == 'auto':
            if cfg.simulation:
                localization_params.append(os.path.join(wm, 'rectangle_sim_hls.yaml'))
        elif cfg.localization_config not in ('', 'none'):
            localization_params.append(cfg.localization_config)
        nodes.append(Node(package='pinky_soccer_world_model', executable='rectangle_localization',
            name='rectangle_localization', output='screen',
            remappings=[('cmd_vel', 'preparation/localized_cmd_vel' if preparation else cfg.command_topic)], parameters=[*localization_params, clock, team, {
                'heading_record': cfg.heading_record,
                'debug_enabled': cfg.localization_debug,
                'led_enabled': not cfg.simulation and not preparation,
                'calibration_turn_cap': min(.5, cfg.max_ang or .5),
                **{k: real[k] for k in ('allow_uncalibrated', 'approximate_hfov_deg',
                                      'camera_yaml', 'image_rotated_180') if k in real}}]))
    elif lidar:
        nodes += localization_nodes(
            cfg.field_spec, cfg.simulation, cfg.initial,
            real.get('allow_uncalibrated', False), real.get('approximate_hfov_deg', 66.0),
            real.get('camera_yaml', ''), cfg.heading_record,
            real.get('image_rotated_180', False))
    world = {'use_sim_time': cfg.simulation, **team,
             'localization': 'tf' if lidar else 'odom_only',
             'localization_pose_timeout': 2.0 if lidar else 0.0,
             'initial_pose': list(cfg.initial), 'playing_default': False}
    if not cfg.simulation:
        world.update({'align_first_odom': True, 'odom_timeout': 0.5,
                      'grid.camera_hfov_deg': real.get('approximate_hfov_deg', 66.0)})
    nodes += [
        Node(package='pinky_soccer_world_model', executable='world_model_node',
             name='world_model_node', output='screen', parameters=[
                 params_file(os.path.join(wm, 'world_model.yaml'), cfg.name),
                 params_file(os.path.join(wm, 'ball_filter.yaml'), cfg.name), world],
             remappings=([('amcl_pose', 'localization/pose')] if rectangle else []) +
                        ([('game/playing', 'preparation/playing')] if preparation else [])),
        Node(package='pinky_soccer_bt', executable='bt_node', name='bt_node', output='screen',
             remappings=[('behavior_goal', 'preparation/strategy_goal')] if preparation else [],
             parameters=[params_file(share('pinky_soccer_bt', 'config', 'bt_node.yaml'),
                                     cfg.name), clock, {
                 'verify_mode': cfg.verify, 'groot2_enabled': cfg.groot2,
                 'kickoff_pose': [float(v) for v in cfg.initial]},
                 *([{'tree_file': cfg.bt_tree_file}] if cfg.bt_tree_file else [])]),
    ]
    if preparation:
        nodes.append(Node(package='pinky_soccer_viz', executable='match_preparation',
            name='match_preparation', output='screen',
            remappings=[('cmd_vel', cfg.command_topic)], parameters=[
                *([cfg.preparation_config] if cfg.preparation_config else []), clock, team]))
    if cfg.controller == 'nav2':
        from .nav2_stack import nav2_nodes
        nodes += nav2_nodes(cfg)
    else:
        nodes.append(controller_node(replace(cfg, command_topic='localization/controller_cmd_vel')
                                     if rectangle else cfg))
    if cfg.ui:
        nodes += ui_nodes(cfg, clock, team)
    return multi_robot.group(cfg.name, remaps(cfg) + nodes)


def ui_nodes(cfg, clock, team):
    """Viewers of the robot shown on this PC."""
    gui = IfCondition('false' if cfg.headless else 'true')
    out = [
        Node(package='pinky_soccer_viz', executable='world_model_viz', name='world_model_viz',
             output='screen', parameters=[team, clock]),
        Node(package='pinky_soccer_viz', executable='world_model_topview',
             name='world_model_topview', output='screen',
             parameters=[team, clock, {'show_ground_truth': cfg.simulation}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', condition=gui, output='screen',
             arguments=['-d', share('pinky_soccer_viz', 'rviz', 'soccer.rviz')],
             parameters=[clock]),
        Node(package='pinky_soccer_viz', executable='game_camera_view',
             name='game_camera_view', condition=gui, output='screen', parameters=[clock, {
                 # Absolute namespaces; this robot first, then the others (keys 1-9, Tab).
                 'robots': [cfg.name] + [r for r in cfg.camera_robots if r != cfg.name],
                 'raw_only_robots': list(cfg.camera_raw_only) or ['']}]),
    ]
    if cfg.verify:
        out.append(Node(
            package='pinky_soccer_viz', executable='behavior_verifier', name='behavior_verifier',
            output='screen', parameters=[team, clock, {
                'simulation': cfg.simulation,
                'mode_label': cfg.target + (f' / {cfg.name}' if cfg.name else ''),
                'controller_config': share('pinky_soccer', 'config', 'reactive_controller.yaml'),
                'max_lin': cfg.max_lin, 'max_ang': cfg.max_ang,
                'controller_kind': cfg.controller,
                'command_topic': ('localization/controller_cmd_vel' if cfg.localization == 'rectangle'
                                  else cfg.command_topic),
                'opponent_truth_topic': cfg.opponent_truth}]))
    if cfg.localization == 'rectangle':
        out.append(Node(package='pinky_soccer_viz', executable='localization_view',
                        name='localization_view', condition=gui, parameters=[clock, {'preparation_enabled': cfg.preparation}]))
        if cfg.preparation:
            out.append(Node(package='pinky_soccer_viz', executable='preparation_view',
                name='preparation_view', condition=gui, parameters=[clock]))
    return out
