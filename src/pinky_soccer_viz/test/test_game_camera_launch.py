"""Both robot and simulation game launches must start the camera GUI with RViz."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest


@pytest.mark.parametrize('filename', ['real_game.launch.py', 'sim_field.launch.py'])
@pytest.mark.parametrize('headless', ['true', 'false'])
def test_camera_and_rviz_follow_headless_together(filename, headless, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    path = Path(__file__).parents[1]/'launch'/filename
    spec = importlib.util.spec_from_file_location('camera_game_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value)
    context.launch_configurations.update(headless=headless, localization='odom_only')
    actions = module.setup(context) if filename.startswith('real') else description.entities
    nodes = [action for action in actions if isinstance(action, Node)]
    active = [node.node_executable for node in nodes
              if node.condition is None or node.condition.evaluate(context)]
    assert active.count('game_camera_view') == (headless == 'false')
    assert active.count('rviz2') == (headless == 'false')
    if filename.startswith('sim'):
        assert context.launch_configurations['publish_debug_image'] == 'true'
