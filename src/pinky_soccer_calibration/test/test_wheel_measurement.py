"""Independent synthetic trajectories and UI save routing for wheel calibration."""

import math
from copy import deepcopy
import threading
from types import SimpleNamespace

import numpy as np

from pinky_soccer_calibration.suite_model import Session, wheel_fit
from pinky_soccer_calibration.wheel_measurement import (
    distance_measurement, rotation_measurement, straight_measurement)

import pytest

import yaml


REFERENCE = {'bumper_from_axle_xyz_m': [.055, .002, .01],
             'robot_description_sha256': 'model'}


def endpoint(distance, angle):
    """Integrate a constant-curvature axle path and a rigidly attached bumper."""
    # Exact SE(2) integration, independently of the inverse conversion under test.
    translation = np.array([distance*np.sinc(angle/np.pi),
                            distance*2*math.sin(angle/2)**2/angle if angle else 0])
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    return translation + (rotation-np.eye(2)) @ np.array(REFERENCE['bumper_from_axle_xyz_m'][:2])


@pytest.mark.parametrize('distance,angle', [
    (1., .08), (-1., -.08), (1., -.12), (-1., .12), (1., 0.), (-1., 0.), (1., 1e-10)])
def test_bumper_displacement_recovers_axle_arc_forward_and_backward(distance, angle):
    """Exercise signs, bumper lever arm, and straight-path numerical stability."""
    x, y = endpoint(distance, angle)
    result = straight_measurement('forward' if distance > 0 else 'backward', x, y, REFERENCE)
    assert result['distance_m'] == pytest.approx(distance)
    assert result['angle_rad'] == pytest.approx(angle)
    assert result['longitudinal_m'] == x
    assert result['lateral_m'] == y


def measurements():
    """Generate independent distance/turn runs with a known common radius."""
    trials = []
    for kind, distance, angle in [('forward', 1., .08), ('backward', -1., -.08),
                                  ('left', .015, 6.2), ('right', -.009, -6.35)]:
        # Rotation deliberately translates: it must not introduce an invented s=0 equation.
        record = {'kind': kind, 'left_rad': (distance-.0961*angle/2)/.028,
                  'right_rad': (distance+.0961*angle/2)/.028}
        record.update(distance_measurement(kind, distance, REFERENCE)
                      if kind in ('forward', 'backward') else
                      rotation_measurement(kind, math.degrees(angle)))
        trials.append(record)
    return trials


def test_fit_separated_trials_recovers_radii_without_assuming_rotation_translation_zero():
    """Do not turn unmeasured rotation translation into a zero-distance constraint."""
    report = wheel_fit(measurements())
    assert report['left_radius_m'] == pytest.approx(.028)
    assert report['right_radius_m'] == pytest.approx(.028)
    assert report['separation_m'] == pytest.approx(.0961)
    assert not report['individual_radii_identified']
    assert 'distance_m' not in measurements()[2]
    with pytest.raises(ValueError):
        wheel_fit(measurements()[:2]*2)


def test_raw_right_encoder_sign_is_normalized_without_modifying_records():
    """Real reverse-sign and sim forward-sign encoders yield the same calibration."""
    records = measurements()
    for row in records:
        row['right_rad'] *= -1
    original = deepcopy(records)
    result = wheel_fit(records)
    assert result['effective_radius_m'] == pytest.approx(.028)
    assert result['separation_m'] == pytest.approx(.0961)
    assert records == original
    assert all(r['encoder_multipliers'] == [1., -1.]
               for r in result['encoder_normalization'])


def test_legacy_lateral_displacement_does_not_become_a_yaw_measurement():
    """Rotating the initial measuring baseline cannot manufacture a wheel asymmetry."""
    records = measurements()
    for row in records[:2]:
        distance = row['distance_m']
        row.update(measurement_type='straight_displacement',
                   longitudinal_m=distance*math.cos(.15),
                   lateral_m=distance*math.sin(.15), angle_rad=123.)
    result = wheel_fit(records)
    assert result['effective_radius_m'] == pytest.approx(.028)
    assert result['separation_m'] == pytest.approx(.0961)
    assert result['left_radius_m'] == result['right_radius_m']


@pytest.mark.parametrize('kind,x,y', [
    ('forward', -1, 0), ('backward', 1, 0), ('forward', 1, float('nan')),
    ('forward', .1, 0), ('forward', 1, 1)])
def test_invalid_displacements_rejected(kind, x, y):
    """Reject wrong signs, nonfinite input, short paths, and excessive deflection."""
    with pytest.raises(ValueError):
        straight_measurement(kind, x, y, REFERENCE)


@pytest.mark.parametrize('kind,angle', [
    ('left', -360), ('right', 360), ('left', 0), ('left', float('nan'))])
def test_invalid_rotation_rejected(kind, angle):
    """Reject wrong direction, zero turns, and missing finite measurements."""
    with pytest.raises(ValueError):
        rotation_measurement(kind, angle)


def test_ui_saves_separate_types_and_invalidates_previous_fit(tmp_path):
    """Save raw measurements through UI handlers and reject mismatched buttons."""
    from pinky_soccer_calibration.suite_ui import App
    app = App.__new__(App)
    app.session = Session(tmp_path, 'rehearsal', 'wheel-test')
    app.node = SimpleNamespace(writer=True, lock=threading.RLock(),
                               measurement_geometry=lambda: SimpleNamespace(sha256='model'))
    app.inputs = {'distance': '1', 'angle': ''}
    app.node.last_motion = {'complete': True, 'kind': 'forward', 'left_rad': 36., 'right_rad': 36.,
                            'measurement_geometry': REFERENCE}
    with pytest.raises(ValueError, match='저장 버튼'):
        app.trial('rotation')
    assert app.node.last_motion is not None and not app.session.trials
    app.session.results['wheels'] = {'stale': True}
    app.trial('straight')
    saved = yaml.safe_load((app.session.root/'wheel_trials.yaml').read_text())
    assert saved[0]['distance_m'] == 1
    assert 'lateral_m' not in saved[0] and 'angle_rad' not in saved[0]
    assert saved[0]['measurement_type'] == 'straight_distance'
    assert (app.session.root/'wheel_straight_trials.yaml').exists()
    assert 'wheels' not in app.session.results
    assert app.node.last_motion is None
    app.inputs = {'distance': '', 'lateral': '', 'angle': '-362'}
    app.node.last_motion = {'complete': True, 'kind': 'right', 'left_rad': 12., 'right_rad': -12.}
    app.trial('rotation')
    saved = yaml.safe_load((app.session.root/'wheel_trials.yaml').read_text())
    assert saved[1]['measured_angle_deg'] == -362
    assert 'distance_m' not in saved[1]
    assert 'lateral_m' not in saved[1]
    rotation = yaml.safe_load((app.session.root/'wheel_rotation_trials.yaml').read_text())
    assert len(rotation) == 1 and rotation[0]['kind'] == 'right'


def test_geometry_uses_axle_midpoint_not_base_origin():
    """Respect nonzero wheel joint positions in the running robot description."""
    from pinky_soccer_calibration.measurement_geometry import MeasurementGeometry
    description = """<robot name="offset"><link name="base_link"/>
    <link name="bumper_link"><visual><geometry><box size="0.02 0.08 0.04"/>
    </geometry></visual></link>
    <joint name="bumper" type="fixed"><parent link="base_link"/><child link="bumper_link"/>
    <origin xyz=".05 .002 .01"/></joint>
    <joint name="l_wheel_joint" type="continuous"><parent link="base_link"/>
    <child link="left"/><origin xyz=".01 .05 0"/></joint>
    <joint name="r_wheel_joint" type="continuous"><parent link="base_link"/>
    <child link="right"/><origin xyz=".01 -.05 0"/></joint></robot>"""
    result = MeasurementGeometry(description).wheel_reference()
    assert result['bumper_from_axle_xyz_m'] == pytest.approx([.05, .002, .01])
    assert result['axle_center_base_xyz_m'] == pytest.approx([.01, 0, 0])
