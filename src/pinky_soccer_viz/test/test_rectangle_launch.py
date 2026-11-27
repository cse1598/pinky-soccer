"""Exactly one localization TF and command authority in the startup path."""
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import importlib.util
from pathlib import Path

from pinky_soccer_viz.robot_stack import decision_stack, RobotConfig


def module(name):
    path = Path(__file__).parents[1]/'launch'/name
    spec = importlib.util.spec_from_file_location('rectangle_launch', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(mod):
    ctx = LaunchContext()
    for action in mod.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            ctx.launch_configurations[action.name] = perform_substitutions(ctx, action.default_value)
    return ctx


def test_real_rectangle_has_no_amcl_or_old_heading_node(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    nodes = [x for x in decision_stack(RobotConfig(localization='rectangle', simulation=False,
             target='real', ui=False)) if isinstance(x, Node)]
    names = [n.node_executable for n in nodes]
    assert names.count('rectangle_localization') == 1
    assert 'amcl' not in names and 'goal_heading' not in names
    controller = next(n for n in nodes if n.node_executable == 'mpc_controller')
    ctx = LaunchContext()
    remaps = [(perform_substitutions(ctx, source), perform_substitutions(ctx, target))
              for source, target in controller._Node__remappings]
    assert ('cmd_vel', 'localization/controller_cmd_vel') in remaps


def test_auto_routing_uses_same_algorithm_in_every_environment():
    mod = module('game.launch.py')
    for mode, expected in [('real', 'rectangle'), ('mujoco', 'rectangle'), ('gazebo', 'rectangle')]:
        ctx = context(mod)
        ctx.launch_configurations['mode'] = mode
        included = mod.setup(ctx)[0]
        args = dict(included.launch_arguments)
        assert args['localization'] == expected


def test_direct_real_default_and_simulator_default():
    main = module('game.launch.py')
    ctx = context(main)
    assert ctx.launch_configurations['mode'] == 'real'
    assert dict(main.setup(ctx)[0].launch_arguments)['localization'] == 'rectangle'
    assert context(module('real_game.launch.py')).launch_configurations['localization'] == 'rectangle'
    assert context(module('sim_field.launch.py')).launch_configurations['localization'] == 'rectangle'
