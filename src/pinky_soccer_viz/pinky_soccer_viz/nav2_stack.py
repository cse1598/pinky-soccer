"""Native Nav2 Jazzy navigation servers for the simulator skill comparison.

Use the existing field_map_server, AMCL and TF; never start a second localization
stack. Nav2 velocity outputs are private to nav2_verifier.
"""
import math
from pathlib import Path
import tempfile

import yaml
from launch_ros.actions import Node
from pinky_soccer_dynamics import params as dyn_params


def nav2_settings(cfg):
    p = dyn_params.load(cfg.target, profile=cfg.calibration_profile or None)
    radius = p.d('wheel_radius')
    track = p.d('wheel_separation')
    wheel_max = p.wheel_cmd_max()
    v = min(cfg.max_lin or .25, radius*wheel_max)
    w = min(cfg.max_ang or 2., 2*radius*wheel_max/track)
    accel = min(1.5, radius*p.wheel_accel_cmd_max())
    ang_accel = min(10., 2*radius*p.wheel_accel_cmd_max()/track)
    return dict(max_lin=v, max_ang=w, wheel_radius=radius, wheel_separation=track,
                wheel_max=wheel_max, accel=accel, ang_accel=ang_accel)


def nav2_config(cfg, s):
    """Return a ROS parameter file dictionary; exact node FQNs include nested costmaps."""
    prefix = '/' + cfg.name.strip('/') if cfg.name else ''
    def topic(name):
        return prefix + '/' + name
    clock = {'use_sim_time': cfg.simulation}
    data = {}
    def add(name, values):
        data[topic(name)] = {'ros__parameters': {**clock, **values}}
    v, w = s['max_lin'], s['max_ang']
    add('planner_server', {
        'expected_planner_frequency': 5., 'costmap_update_timeout': 1.,
        'planner_plugins': ['GridBased'],
        'GridBased': {'plugin': 'nav2_navfn_planner::NavfnPlanner',
                      'tolerance': .005, 'use_astar': True, 'allow_unknown': False}})
    follow = {
        'plugin': 'nav2_mppi_controller::MPPIController',
        'motion_model': 'DiffDrive', 'time_steps': 56, 'model_dt': .05,
        'batch_size': 1000, 'iteration_count': 1, 'vx_max': v, 'vx_min': -v,
        'wz_max': w, 'ax_max': s['accel'], 'ax_min': -s['accel'],
        'az_max': s['ang_accel'], 'vx_std': min(.12, v*.5), 'wz_std': min(.8, w*.4),
        'temperature': .3, 'gamma': .015, 'prune_distance': 1.2,
        'transform_tolerance': .2, 'visualize': False, 'regenerate_noises': True,
        'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
                    'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic'],
        'ConstraintCritic': {'cost_weight': 4.},
        'CostCritic': {'cost_weight': 3.81, 'critical_cost': 300.,
                       'collision_cost': 1000000., 'consider_footprint': True,
                       'near_goal_distance': .12, 'trajectory_point_step': 1},
        'GoalCritic': {'cost_weight': 8., 'threshold_to_consider': .5},
        'GoalAngleCritic': {'cost_weight': 6., 'threshold_to_consider': .20},
        'PathAlignCritic': {'cost_weight': 10., 'threshold_to_consider': .10,
                            'offset_from_furthest': 10, 'trajectory_point_step': 3},
        'PathFollowCritic': {'cost_weight': 5., 'threshold_to_consider': .30,
                             'offset_from_furthest': 5},
        'PathAngleCritic': {'cost_weight': 2., 'threshold_to_consider': .12,
                            'offset_from_furthest': 4, 'mode': 1}}
    add('controller_server', {
        'controller_frequency': 20., 'costmap_update_timeout': .5,
        'enable_stamped_cmd_vel': False, 'odom_topic': topic('odom'),
        'min_x_velocity_threshold': .001, 'min_y_velocity_threshold': .001,
        'min_theta_velocity_threshold': .001, 'failure_tolerance': .5,
        'progress_checker_plugins': ['progress_checker'],
        'goal_checker_plugins': ['goal_checker'], 'controller_plugins': ['FollowPath'],
        'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                             'required_movement_radius': .025, 'movement_time_allowance': 8.},
        'goal_checker': {'plugin': 'nav2_controller::StoppedGoalChecker',
                         'xy_goal_tolerance': .015, 'yaw_goal_tolerance': math.radians(2.5),
                         'trans_stopped_velocity': .01, 'rot_stopped_velocity': .05,
                         'stateful': False}, 'FollowPath': follow})
    # Conservative rectangular hull of the existing MuJoCo collision geometry,
    # with 2 cm padding. This is not the two-disc NMPC approximation.
    for local in (False, True):
        name = 'local_costmap' if local else 'global_costmap'
        add(name+'/'+name, {
            'global_frame': 'odom' if local else 'map',
            'robot_base_frame': 'base_footprint', 'transform_tolerance': .3,
            'update_frequency': 10. if local else 5.,
            'publish_frequency': 5. if local else 2.,
            'rolling_window': local, 'width': 3, 'height': 3,
            'resolution': .01, 'track_unknown_space': not local,
            'footprint': '[[-0.076,-0.0766],[0.0421,-0.0766],[0.0421,0.0766],[-0.076,0.0766]]',
            'footprint_padding': .02, 'always_send_full_costmap': False,
            'plugins': (['obstacle_layer', 'inflation_layer'] if local else
                        ['static_layer', 'obstacle_layer', 'inflation_layer']),
            'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer',
                             'map_topic': topic('map'), 'map_subscribe_transient_local': True},
            'obstacle_layer': {'plugin': 'nav2_costmap_2d::ObstacleLayer',
                               'enabled': True, 'observation_sources': 'scan',
                               'scan': {'topic': topic('scan'), 'data_type': 'LaserScan',
                                        'marking': True, 'clearing': True, 'inf_is_valid': True,
                                        'obstacle_min_range': .035, 'obstacle_max_range': 3.,
                                        'raytrace_min_range': .035, 'raytrace_max_range': 3.5,
                                        'max_obstacle_height': 2.}},
            'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer',
                                'inflation_radius': .20, 'cost_scaling_factor': 12.}})
    add('bt_navigator', {'global_frame': 'map', 'robot_base_frame': 'base_footprint',
                         'odom_topic': topic('odom'), 'bt_loop_duration': 10,
                         'default_server_timeout': 20, 'wait_for_service_timeout': 1000,
                         'navigators': ['navigate_to_pose'],
                         'navigate_to_pose': {'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'},
                         'error_code_names': ['compute_path_error_code', 'follow_path_error_code']})
    add('behavior_server', {'local_costmap_topic': 'local_costmap/costmap_raw',
                            'global_costmap_topic': 'global_costmap/costmap_raw',
                            'local_footprint_topic': 'local_costmap/published_footprint',
                            'global_footprint_topic': 'global_costmap/published_footprint',
                            'local_frame': 'odom', 'global_frame': 'map',
                            'robot_base_frame': 'base_footprint', 'cycle_frequency': 20.,
                            'enable_stamped_cmd_vel': False, 'max_rotational_vel': w,
                            'min_rotational_vel': min(.3, w), 'rotational_acc_lim': s['ang_accel'],
                            'behavior_plugins': ['spin', 'backup', 'wait'],
                            'spin': {'plugin': 'nav2_behaviors::Spin'},
                            'backup': {'plugin': 'nav2_behaviors::BackUp'},
                            'wait': {'plugin': 'nav2_behaviors::Wait'}})
    return data


def nav2_nodes(cfg):
    if not cfg.simulation or not cfg.verify or cfg.localization != 'lidar':
        raise ValueError('Nav2 comparison requires simulation, verify:=true, localization:=lidar')
    s = nav2_settings(cfg)
    filename = Path(tempfile.mkdtemp(prefix='pinky_nav2_verify_')) / 'nav2.yaml'
    filename.write_text(yaml.safe_dump(nav2_config(cfg, s)), encoding='utf-8')
    nodes = []
    for package, executable in (('nav2_planner', 'planner_server'),
                                ('nav2_controller', 'controller_server'),
                                ('nav2_behaviors', 'behavior_server'),
                                ('nav2_bt_navigator', 'bt_navigator')):
        nodes.append(Node(package=package, executable=executable, name=executable,
                          output='screen', parameters=[str(filename)],
                          remappings=[('cmd_vel', 'nav2_verify/cmd_vel')]))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='navigation_lifecycle', output='screen', parameters=[{
                          'use_sim_time': cfg.simulation, 'autostart': True,
                          'node_names': ['planner_server', 'controller_server',
                                         'behavior_server', 'bt_navigator'],
                          'bond_timeout': 8.}]))
    nodes.append(Node(package='pinky_soccer_viz', executable='nav2_verifier',
                      name='nav2_verifier', output='screen',
                      remappings=[('cmd_vel', cfg.command_topic)],
                      parameters=[{'use_sim_time': cfg.simulation,
                                   **{k: s[k] for k in ('max_lin', 'max_ang', 'wheel_radius',
                                                        'wheel_separation', 'wheel_max')}}]))
    return nodes
