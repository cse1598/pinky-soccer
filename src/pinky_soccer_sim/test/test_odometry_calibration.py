"""Simulator odometry calibration: helpers, MuJoCo drive constants and bundled profiles."""

import math
from pathlib import Path
import types

from pinky_soccer_bringup.calibration_profile import read_profile
from pinky_soccer_sim import calibrate_odometry as cal
from pinky_soccer_sim.mujoco_soccer_bridge import install_drive_constants
import pytest

PROFILES = Path(__file__).resolve().parents[1] / 'config' / 'calibration'


def test_bumper_chord_signed_and_with_heading():
    front = (0.0415, 0.0, 0.0)
    assert cal.bumper_chord((0, 0, 0), (1.0, 0, 0), front) == pytest.approx(1.0)
    assert cal.bumper_chord((0, 0, 0), (-1.0, 0, 0), front) == pytest.approx(-1.0)
    # Pure rotation moves the bumper point although the base does not translate.
    assert abs(cal.bumper_chord((0, 0, 0), (0, 0, math.pi / 2), front)) > 0.05


def test_true_turn_unwraps_with_odometry():
    assert math.degrees(cal.true_turn(2 * math.pi, 0.0, math.radians(-8.7))) == \
        pytest.approx(351.3)
    assert math.degrees(cal.true_turn(-2 * math.pi, 0.1, 0.1 + math.radians(9))) == \
        pytest.approx(-351.0)


def test_gazebo_pose_parse(monkeypatch):
    out = ('Model: [12]\n  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n'
           '    [0.500000 -0.010000 0.020000]\n    [0.000000 -0.000000 1.570000]\n')
    monkeypatch.setattr(cal.subprocess, 'run',
                        lambda *a, **k: types.SimpleNamespace(stdout=out))
    assert cal.gazebo_pose('pinky') == (0.5, -0.01, 1.57)


def test_mujoco_drive_constants_override_conversion_only():
    class Physics:
        def __init__(self):
            self.radius, self.track = 0.028, 0.0971
    bridge = types.SimpleNamespace(Physics=Physics)
    install_drive_constants(bridge, 0.0275, 0.0982)
    p = bridge.Physics()
    assert (p.radius, p.track) == (0.0275, 0.0982)
    assert (p.model_radius, p.model_track) == (0.028, 0.0971)
    untouched = types.SimpleNamespace(Physics=Physics)
    install_drive_constants(untouched)
    assert untouched.Physics is Physics


@pytest.mark.parametrize('simulator', ['mujoco', 'gazebo'])
def test_bundled_profiles_are_valid_simulation_profiles(simulator):
    path = PROFILES / f'{simulator}_profile.yaml'
    data = read_profile(path, simulation=True)
    assert data['environment'] == 'rehearsal' and data['robot_id'] == f'{simulator}-sim'
    assert data['measurement_source'] == 'simulator_ground_truth'
    w = data['results']['wheels']
    assert data['launch']['wheel_radius'] == pytest.approx(w['effective_radius_m'])
    assert data['launch']['wheel_separation'] == pytest.approx(w['separation_m'])
    assert w['trials'] == 12
    with pytest.raises(ValueError):
        read_profile(path, simulation=False)
