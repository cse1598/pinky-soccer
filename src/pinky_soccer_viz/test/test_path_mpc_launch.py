import pytest
from pinky_soccer_viz.robot_stack import RobotConfig, decision_stack


@pytest.mark.parametrize('target', ['mujoco', 'real'])
def test_path_strategy_can_build_game_stack(target):
    assert decision_stack(RobotConfig(controller='attack', attack_strategy='path_mpc',
        target=target, simulation=target == 'mujoco', headless=True))


@pytest.mark.parametrize('changes', [dict(target='gazebo'), dict(verify=True),
                                    dict(controller='mpc'), dict(bt_tree_file='custom.xml')])
def test_path_strategy_keeps_other_restrictions(changes):
    cfg = RobotConfig(controller='attack', attack_strategy='path_mpc')
    for k, v in changes.items():
        setattr(cfg, k, v)
    with pytest.raises(ValueError, match='path_mpc'):
        decision_stack(cfg)
