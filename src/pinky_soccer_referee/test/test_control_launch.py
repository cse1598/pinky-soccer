import importlib.util
from pathlib import Path

import pytest


@pytest.mark.parametrize('mode,expected', [('topic', True), ('video', False)])
def test_control_mode_defaults_and_explicit_override(mode, expected, tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    path = Path(__file__).parents[1]/'launch/referee.launch.py'
    spec = importlib.util.spec_from_file_location('referee_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ctx = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            ctx.launch_configurations[action.name] = perform_substitutions(ctx, action.default_value)
    video = tmp_path/'input.mp4'
    video.write_bytes(b'launch path validation only; never opened')
    ctx.launch_configurations.update(mode=mode, video_path=str(video), robot_namespaces='yellow,blue')
    node = module.setup(ctx)[-1]
    params = {perform_substitutions(ctx, key): value
              for key, value in node._Node__parameters[0].items()}
    assert params['match_controls'] == expected
    ctx.launch_configurations['match_controls'] = 'true'
    params = {perform_substitutions(ctx, key): value
              for key, value in module.setup(ctx)[-1]._Node__parameters[0].items()}
    assert params['match_controls'] is True
