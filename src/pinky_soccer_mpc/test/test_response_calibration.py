"""Measured real response, simulator identity and unchanged command limits."""
from copy import deepcopy

import numpy as np
import pytest
import yaml

from pinky_soccer_dynamics import params as P
from pinky_soccer_dynamics import profile_model
from pinky_soccer_mpc.ramp_predictor import RampPredictor
from pinky_soccer_mpc.response_calibration import apply_response, default_response_path


def snapshot():
    return yaml.safe_load(default_response_path().read_text())


def test_real_default_changes_only_time_constants():
    original = P.load('real')
    before = deepcopy(original.as_dict())
    p, info = apply_response(original)
    assert original.as_dict() == before
    assert info['applied']
    assert p.m('tau_left') == pytest.approx(.09031918634164283)
    assert p.m('tau_right') == pytest.approx(.08894523651011192)
    assert p.driver == original.driver and p.limits == original.limits
    assert p.joint_sign == original.joint_sign
    assert p.wheel_cmd_max() == original.wheel_cmd_max()
    assert p.wheel_accel_cmd_max() == original.wheel_accel_cmd_max()
    assert profile_model.profile_accel(p) == 74.9
    assert all(p.model[k] == original.model[k] for k in original.model
               if k not in ('tau_left', 'tau_right'))


@pytest.mark.parametrize('target', ['mujoco', 'gazebo'])
@pytest.mark.parametrize('selection', ['', 'auto', 'none'])
def test_simulation_default_is_identical(target, selection):
    original = P.load(target)
    result, info = apply_response(original, selection)
    assert result is original and info == {'applied': False}
    assert result.as_dict() == original.as_dict()


def test_explicit_simulator_application_and_legacy_file_are_rejected(tmp_path):
    with pytest.raises(ValueError, match='simulator'):
        apply_response(P.load('mujoco'), str(default_response_path()))
    legacy = tmp_path/'legacy.yaml'
    legacy.write_text('identification_schema: 1\naccepted: true\n')
    with pytest.raises(ValueError, match='legacy'):
        apply_response(P.load('real'), str(legacy))


def test_rollback_retains_original_response():
    original = P.load('real')
    restored, info = apply_response(original, 'none')
    assert restored is original and not info['applied']
    assert restored.m('tau_left') == restored.m('tau_right') == .05


@pytest.mark.parametrize('change', ['tau_nan', 'unidentified', 'units', 'ramp', 'overlap',
                                   'no_validation', 'worse_validation', 'extra_parameter'])
def test_incompatible_or_unvalidated_response_rejected(tmp_path, change):
    d = snapshot()
    if change == 'tau_nan':
        d['parameters']['tau_left'] = float('nan')
    elif change == 'unidentified':
        d['parameter_status']['tau_left'] = 'identification_insufficient'
    elif change == 'units':
        d['units']['tau_left'] = 'ms'
    elif change == 'ramp':
        d['fixed_profile_accel'] = 46.9
    elif change == 'overlap':
        d['validation_experiments'].append(d['identification_experiments'][0])
    elif change == 'no_validation':
        d['validation_experiments'] = []
    elif change == 'worse_validation':
        d['validation']['rmse_rad_s'] = 1.
    else:
        d['parameters']['delay'] = .01
    path = tmp_path/'bad.yaml'
    path.write_text(yaml.safe_dump(d))
    with pytest.raises(ValueError):
        apply_response(P.load('real'), str(path))


def test_calibrated_numeric_scalar_and_symbolic_predictions_match():
    import casadi as ca
    calibrated, _ = apply_response(P.load('real'))
    x, u, expression = profile_model.symbolic_step(calibrated, .1)
    symbolic = ca.Function('real_response', [x, u], [expression])
    predictor = RampPredictor(calibrated, .1)
    state = np.array([.2, -.1, .3, 1., 2., 1.4, 2.2, .5, .6])
    command = np.array([2., 3.])
    numeric = profile_model.step(calibrated, state, command, .1)
    assert np.allclose(predictor.step(state[:7], command), numeric[:7], atol=1e-12)
    assert np.allclose(np.asarray(symbolic(state, command)).ravel(), numeric, atol=1e-12)
