"""
Parameter loading for the Pinky Pro wheel dynamics, with provenance.

Merge order (later wins), each value keeping its source:
  1. config/dynamics.yaml defaults for the target (official_default / simulation_model /
     assumption)
  2. calibration profile from pinky_soccer_calibration (``profile.yaml``):
     ``results.wheels.left_radius_m / right_radius_m / separation_m`` -> model geometry
     (source ``measured``), and ``launch.wheel_radius / wheel_separation`` -> the radius and
     separation the official driver is started with (driver group only)
  3. wheel response identification file written by this package (``identification.yaml``,
     schema below): tau_left/right, delay, wheel_speed_max, wheel_accel_max (``measured``)

The profile format is the one exported by pinky_soccer_calibration suite_model.Session.export
(schema_version 1, human_verified, environment real|rehearsal). Nothing is invented there;
response parameters live in our own identification file because the profile has no keys
for them.
"""

from copy import deepcopy
from dataclasses import dataclass, field
import math
from pathlib import Path

import yaml

MODEL_KEYS = ('left_radius', 'right_radius', 'track', 'tau_left', 'tau_right', 'delay',
              'wheel_speed_max', 'wheel_accel_max')
RESPONSE_KEYS = ('tau_left', 'tau_right', 'delay', 'wheel_speed_max', 'wheel_accel_max')
SOURCES = ('official_default', 'simulation_model', 'assumption', 'measured',
           'simulation_identified')
IDENTIFICATION_SCHEMA = 1


def default_config_path():
    """Installed (or source-tree) dynamics.yaml."""
    here = Path(__file__).resolve().parents[1] / 'config' / 'dynamics.yaml'
    if here.is_file():
        return here
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('pinky_soccer_dynamics')) / 'config' / 'dynamics.yaml'


@dataclass
class Value:
    """One parameter value with its provenance."""

    value: float
    source: str
    ref: str = ''
    meta: dict = field(default_factory=dict)

    def as_dict(self):
        """Return a YAML-friendly dict."""
        out = {'value': self.value, 'source': self.source, 'ref': self.ref}
        if self.meta:
            out['meta'] = self.meta
        return out


@dataclass
class DynamicsParams:
    """Model, driver and sign parameters for one target."""

    target: str
    model: dict
    driver: dict
    joint_sign: dict
    inputs: list = field(default_factory=list)       # files merged, in order

    def m(self, key):
        """Return a model parameter value."""
        return float(self.model[key].value)

    def d(self, key, default=None):
        """Return a driver parameter value (or default when absent)."""
        v = self.driver.get(key)
        return default if v is None else float(v.value)

    @property
    def sign_left(self):
        """Return the /joint_states sign of the left wheel."""
        return int(self.joint_sign['left'].value)

    @property
    def sign_right(self):
        """Return the /joint_states sign of the right wheel."""
        return int(self.joint_sign['right'].value)

    def unmeasured(self):
        """Return model keys whose value is not measured on this target."""
        return [k for k in MODEL_KEYS if self.model[k].source not in ('measured',
                                                                      'simulation_model',
                                                                      'simulation_identified')]

    def as_dict(self):
        """Return a YAML-friendly summary."""
        return {'target': self.target, 'inputs': self.inputs,
                'model': {k: v.as_dict() for k, v in self.model.items()},
                'driver': {k: v.as_dict() for k, v in self.driver.items()},
                'joint_sign': {k: v.as_dict() for k, v in self.joint_sign.items()},
                'unmeasured': self.unmeasured()}


def _values(group):
    out = {}
    for key, item in (group or {}).items():
        if not isinstance(item, dict) or 'value' not in item or 'source' not in item:
            raise ValueError(f'{key}: every entry needs value and source')
        if item['source'] not in SOURCES:
            raise ValueError(f'{key}: unknown source {item["source"]}')
        out[key] = Value(item['value'], item['source'], item.get('ref', ''),
                         dict(item.get('meta', {})))
    return out


def _check_positive(name, value, upper=None):
    value = float(value)
    if not math.isfinite(value) or value <= 0 or (upper is not None and value > upper):
        raise ValueError(f'{name}={value} out of range')
    return value


def validate(params):
    """Raise ValueError when a parameter is physically implausible."""
    m = params.model
    for key in MODEL_KEYS:
        if key not in m:
            raise ValueError(f'missing model parameter {key}')
    for key in ('left_radius', 'right_radius'):
        _check_positive(key, m[key].value, 0.1)
    _check_positive('track', m['track'].value, 0.5)
    for key in ('tau_left', 'tau_right'):
        _check_positive(key, m[key].value, 2.0)
    delay = float(m['delay'].value)
    if not (math.isfinite(delay) and 0.0 <= delay <= 0.5):
        raise ValueError(f'delay={delay} out of range')
    _check_positive('wheel_speed_max', m['wheel_speed_max'].value, 100.0)
    _check_positive('wheel_accel_max', m['wheel_accel_max'].value, 1000.0)
    for key in ('wheel_radius', 'wheel_separation'):
        if key not in params.driver:
            raise ValueError(f'missing driver parameter {key}')
        _check_positive(key, params.driver[key].value, 0.5)
    for side in ('left', 'right'):
        if int(params.joint_sign[side].value) not in (-1, 1):
            raise ValueError(f'joint_sign.{side} must be +1 or -1')
    return params


def load_defaults(target, config=None):
    """Load the target defaults from dynamics.yaml."""
    path = Path(config) if config else default_config_path()
    data = yaml.safe_load(path.read_text())
    if data.get('schema_version') != 1:
        raise ValueError(f'{path}: schema_version 1 required')
    if target not in data['targets']:
        raise ValueError(f'unknown target {target}; have {sorted(data["targets"])}')
    t = data['targets'][target]
    return DynamicsParams(target, _values(t['model']), _values(t['driver']),
                          _values(t['joint_sign']), [str(path)])


def apply_profile(params, path, *, apply_driver=True):
    """Merge a pinky_soccer_calibration profile.yaml (wheel geometry only)."""
    path = Path(path).expanduser()
    data = yaml.safe_load(path.read_text())
    if data.get('schema_version') != 1 or data.get('human_verified') is not True:
        raise ValueError(f'{path}: reviewed calibration profile (schema 1) required')
    env = data.get('environment')
    if env not in ('real', 'rehearsal'):
        raise ValueError(f'{path}: environment missing')
    if params.target == 'real' and env != 'real':
        raise ValueError('rehearsal profiles cannot be applied to the physical robot')
    out = deepcopy(params)
    meta = {'file': str(path), 'robot_id': data.get('robot_id'),
            'environment': env, 'exported_utc': data.get('exported_utc'),
            'project_revision': data.get('project_revision')}
    wheels = (data.get('results') or {}).get('wheels')
    if wheels:
        src = 'measured' if env == 'real' else 'simulation_identified'
        wmeta = dict(meta, trials=wheels.get('trials'),
                     equation_residual=wheels.get('equation_residual'),
                     condition_number=wheels.get('condition_number'))
        for key, pkey in (('left_radius', 'left_radius_m'), ('right_radius', 'right_radius_m'),
                          ('track', 'separation_m')):
            out.model[key] = Value(float(wheels[pkey]), src,
                                   f'profile results.wheels.{pkey}', wmeta)
    launch = data.get('launch') or {}
    if apply_driver:
        # The bringup launch passes these to the official driver, so the driver converts
        # /cmd_vel with them. They describe the driver, not the robot's true geometry.
        for key in ('wheel_radius', 'wheel_separation'):
            if key in launch:
                out.driver[key] = Value(float(launch[key]), 'measured' if env == 'real'
                                        else 'simulation_identified', f'profile launch.{key}',
                                        meta)
    out.inputs.append(str(path))
    return validate(out)


def apply_identification(params, path):
    """Merge a wheel response identification file written by identify tools."""
    path = Path(path).expanduser()
    data = yaml.safe_load(path.read_text())
    if data.get('identification_schema') != IDENTIFICATION_SCHEMA:
        raise ValueError(f'{path}: identification_schema {IDENTIFICATION_SCHEMA} required')
    if data.get('target') != params.target:
        raise ValueError(f'{path}: identification for {data.get("target")}, '
                         f'not {params.target}')
    if data.get('accepted') is not True:
        raise ValueError(f'{path}: not accepted (validation failed or not reviewed)')
    out = deepcopy(params)
    meta = {'file': str(path), 'robot_id': data.get('robot_id'),
            'experiment_utc': data.get('experiment_utc'),
            'validation': data.get('validation')}
    src = 'measured' if params.target == 'real' else 'simulation_identified'
    for key, value in (data.get('parameters') or {}).items():
        if key not in RESPONSE_KEYS:
            raise ValueError(f'{path}: {key} is not a response parameter')
        out.model[key] = Value(float(value), src, f'identification {path.name}', meta)
    out.inputs.append(str(path))
    return validate(out)


def load(target, config=None, profile=None, identification=None):
    """Return validated parameters for target with optional calibration inputs."""
    params = validate(load_defaults(target, config))
    if profile:
        params = apply_profile(params, profile)
    if identification:
        params = apply_identification(params, identification)
    return params
