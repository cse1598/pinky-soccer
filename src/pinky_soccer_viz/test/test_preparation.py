"""Preparation owns procedure only; GoToPose completion and sensor localization are inputs."""
from pathlib import Path
import math

import pytest
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_viz.preparation_core import Preparation, start_pose


@pytest.mark.parametrize('team', ['yellow', 'blue'])
def test_complete_sequence_and_led(team):
    p = Preparation(team)
    assert p.led == ((0, 0, 0), False)
    assert not p.command_allowed
    assert not p.start(True)[0]
    assert p.prepare()[0]
    assert p.led == ((0, 96, 0), True)
    assert not p.prepare()[0]
    assert not p.start(True)[0]
    p.stopped_at_start()
    assert p.state == 'CALIBRATING'
    p.calibrated()
    assert p.state == 'MOVING_TO_START' and p.led[1]
    p.stopped_at_start()
    assert p.state == 'MOVING_TO_START'
    p.arrival()
    assert not p.command_allowed
    p.stopped_at_start()
    assert p.state == 'READY' and p.prepared and not p.led[1]
    assert not p.start(False)[0]
    assert p.start(True)[0] and p.command_allowed
    p.stop()
    assert p.state == 'STOPPED' and not p.command_allowed


@pytest.mark.parametrize('stage', ['CALIBRATING', 'MOVING_TO_START'])
def test_failure_stop_retry_no_false_ready(stage):
    p = Preparation()
    p.prepare()
    if stage == 'MOVING_TO_START':
        p.calibrated()
    p.fail('test')
    assert p.led == ((96, 0, 0), True)
    assert not p.prepared and not p.start(True)[0]
    assert p.prepare()[0]
    p.stop()
    assert not p.start(True)[0]


def test_start_pose_reuses_geometry_and_rejects_wall():
    field = Field.from_file(Path(__file__).resolve().parents[2]/
                            'pinky_soccer_sim/config/hardboard_field.yaml')
    assert start_pose(field, 'yellow') == (-.4, 0., 0.)
    assert start_pose(field, 'blue') == (.4, 0., math.pi)
    assert start_pose(field, 'yellow', .3)[0] == pytest.approx(-field.half_length+.3)
    with pytest.raises(ValueError):
        start_pose(field, 'yellow', .01)
    with pytest.raises(ValueError):
        start_pose(field, 'yellow', explicit=[float('nan'), 0, 0])


def test_test_opponent_does_not_move_before_enable():
    from types import SimpleNamespace
    from pinky_soccer_viz.obstacle_driver import ObstacleDriver, Shuttle
    sent=[]
    fake=SimpleNamespace(mode='lateral',enabled=False,odom=(0.,0.,0.),
        pub=SimpleNamespace(publish=sent.append),shuttle=Shuttle(.1,.4))
    ObstacleDriver.tick(fake)
    assert sent[-1].linear.x == 0 and sent[-1].angular.z == 0
    fake.enabled=True
    ObstacleDriver.tick(fake)
    assert sent[-1].linear.x == .1
