"""Real-robot, fixed-ramp MPC time constants; never change simulator defaults."""
from copy import deepcopy
import hashlib
import math
from pathlib import Path

import yaml

from pinky_soccer_dynamics.params import Value
from pinky_soccer_dynamics.profile_model import profile_accel


def default_response_path():
    """This robot's reviewed snapshot, installed with the MPC package."""
    path = Path(__file__).resolve().parents[1] / 'config' / 'real_motor_response.yaml'
    if path.is_file():
        return path
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('pinky_soccer_mpc')) / 'config' / path.name


def apply_response(params, selection=''):
    """Apply only tau L/R. Blank/auto uses real snapshot, none preserves the baseline.

    Legacy lag+delay identification is intentionally incompatible: its fitted ramp
    and transport delay cannot silently be dropped or applied a second time.
    """
    if selection == 'none' or (selection in ('', 'auto') and params.target != 'real'):
        return params, {'applied': False}
    if params.target != 'real':
        raise ValueError('Real motor response calibration cannot be applied to a simulator')
    path = default_response_path() if selection in ('', 'auto') else Path(selection).expanduser()
    raw = path.read_bytes()
    data = yaml.safe_load(raw)
    if not isinstance(data, dict) or data.get('mpc_motor_response_schema') != 1:
        raise ValueError('Expected fixed-ramp mpc_motor_response_schema: 1; legacy identification is not supported')
    if data.get('target') != 'real' or data.get('environment') != 'real' or not data.get('robot_id'):
        raise ValueError('Real target, environment and robot_id are required')
    if data.get('model') != 'fixed_ramp_first_order_no_delay' or data.get('approved_for_mpc') is not True:
        raise ValueError('Reviewed fixed-ramp first-order response required')
    if data.get('units') != {'tau_left': 's', 'tau_right': 's', 'fixed_profile_accel': 'rad/s^2'}:
        raise ValueError('Invalid motor response units')
    fixed = data.get('fixed_profile_accel')
    if (not isinstance(fixed, (int, float)) or not math.isfinite(fixed) or
            not math.isclose(float(fixed), profile_accel(params), rel_tol=1e-10)):
        raise ValueError('Response was fitted for a different motor profile ramp')
    if params.m('delay') != 0:
        raise ValueError('Fixed-ramp response does not implement transport delay')
    values = data.get('parameters') or {}
    if set(values) != {'tau_left', 'tau_right'}:
        raise ValueError('Only tau_left/right may be changed by this calibration')
    status = data.get('parameter_status') or {}
    for key, value in values.items():
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or not .003 < value < .6 or status.get(key) != 'estimated'):
            raise ValueError(f'{key}: finite, identified time constant in (0.003, 0.6) s required')
    validation = data.get('validation') or {}
    train = set(data.get('identification_experiments') or [])
    held = set(data.get('validation_experiments') or [])
    if not train or not held or train & held or validation.get('held_out') is not True:
        raise ValueError('Disjoint identification and held-out validation experiments required')
    candidate, baseline = (validation.get(k) for k in ('rmse_rad_s', 'baseline_rmse_rad_s'))
    if (not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
                for v in (candidate, baseline)) or candidate >= baseline):
        raise ValueError('Held-out velocity error must improve over the original response')
    result = deepcopy(params)
    provenance = dict(file=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest(),
                      robot_id=data['robot_id'], model=data['model'],
                      fixed_profile_accel=fixed, validation=validation)
    for key, value in values.items():
        result.model[key] = Value(float(value), 'measured', 'fixed-ramp effective tau fit', provenance)
    result.inputs.append(str(path.resolve()))
    return result, dict(applied=True, **provenance, parameters=values)
