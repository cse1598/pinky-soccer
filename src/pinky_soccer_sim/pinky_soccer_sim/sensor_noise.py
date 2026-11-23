"""Shared measured bias and Gaussian residual model for both simulator backends."""

from copy import deepcopy
from pathlib import Path

import numpy as np

from scipy.spatial.transform import Rotation

import yaml

STREAMS = {'lidar': 1, 'ultrasonic': 2, 'gyro': 3, 'accel': 4, 'orientation': 5}


def load_noise(path):
    """Validate a portable fitted model before accepting any simulated observations."""
    cfg = yaml.safe_load(Path(path).read_text())
    if not isinstance(cfg, dict) or cfg.get('schema_version') != 1:
        raise ValueError('Unsupported sensor noise schema')
    for name in ('lidar', 'ultrasonic'):
        model = cfg[name]
        if model.get('extrapolation') != 'hold_endpoint_parameters':
            raise ValueError('Range noise requires an explicit extrapolation policy')
        table = np.asarray(model['table'], dtype=float)
        if (table.ndim != 2 or table.shape[1] != 4 or len(table) < 1
                or not np.isfinite(table).all() or np.any(table[:, 0] <= 0)
                or np.any(np.diff(table[:, 0]) <= 0) or np.any(table[:, 2:] < 0)):
            raise ValueError('Noise table requires increasing distance, bias, independent/common sigma')
    for name in ('gyro', 'accel', 'orientation'):
        if (not np.isfinite(cfg[name].get('correlation_time_s', 0.))
                or cfg[name].get('correlation_time_s', 0.) < 0
                or not 0 < cfg[name].get('sample_period_s', .01) < np.inf):
            raise ValueError('Invalid IMU temporal correlation parameters')
        bias = np.asarray(cfg[name]['bias'], dtype=float)
        covariance = np.asarray(cfg[name]['covariance'], dtype=float)
        if (bias.shape != (3,) or covariance.shape != (3, 3)
                or not np.isfinite(bias).all() or not np.isfinite(covariance).all()
                or not np.allclose(covariance, covariance.T, atol=1e-12)
                or np.min(np.linalg.eigvalsh(covariance)) < -1e-12):
            raise ValueError('IMU noise requires a finite bias and positive semidefinite covariance')
    orientation = cfg['orientation']
    probability = orientation.get('outlier_probability', 0.)
    outliers = np.asarray(orientation.get('outlier_rotvecs', []), dtype=float)
    if (not np.isfinite(probability) or not 0 <= probability <= 1
            or (probability > 0 and (outliers.ndim != 2 or outliers.shape[1] != 3
                                     or not len(outliers) or not np.isfinite(outliers).all()))):
        raise ValueError('Invalid attitude outlier distribution')
    return cfg


class SensorNoise:
    """Add errors once at the public interface, with independent reproducible streams."""

    def __init__(self, cfg, seed=20261124, enabled=True):
        """Keep per-modality random streams independent of sensor enable flags."""
        if not isinstance(seed, int) or seed < 0:
            raise ValueError('noise_seed must be a nonnegative integer')
        self.cfg, self.seed, self.enabled = deepcopy(cfg), seed, enabled
        self.generators, self.last_stamp, self.vector_states = {}, {}, {}

    def rng(self, stream, stamp=None):
        """Restart a stream deterministically on a backwards simulation-time jump."""
        if stamp is not None:
            if stamp < self.last_stamp.get(stream, stamp):
                self.generators.pop(stream, None)
                self.vector_states.pop(stream, None)
            self.last_stamp[stream] = stamp
        if stream not in self.generators:
            self.generators[stream] = np.random.default_rng(
                np.random.SeedSequence([self.seed, STREAMS[stream]]))
        return self.generators[stream]

    def ranges(self, name, distances, minimum, maximum, stamp=None):
        """Preserve missing returns; extrapolate endpoint parameters without clamping distance."""
        values = np.asarray(distances, dtype=float).copy()
        if not self.enabled:
            return values
        valid = np.isfinite(values) & (values >= minimum) & (values <= maximum)
        if not valid.any():
            return values
        table = np.asarray(self.cfg[name]['table'], dtype=float)
        bias, independent, common = [np.interp(values[valid], table[:, 0], table[:, i])
                                     for i in (1, 2, 3)]
        rng = self.rng(name, stamp)
        values[valid] += bias + rng.normal(size=valid.sum())*independent + rng.normal()*common
        values[valid & (values < minimum)] = -np.inf
        values[valid & (values > maximum)] = np.inf
        return values

    def vector(self, name, values, stamp=None):
        """Add a stationary correlated residual with measured cross-axis covariance."""
        values = np.asarray(values, dtype=float)
        if not self.enabled:
            return values.copy()
        model = self.cfg[name]
        rng = self.rng(name, stamp)
        tau = model.get('correlation_time_s', 0.)
        previous = self.vector_states.get(name)
        alpha = 0.
        if tau > 0 and previous is not None:
            dt = ((stamp-previous[0])*1e-9 if stamp is not None and previous[0] is not None
                  else model.get('sample_period_s', .01))
            alpha = float(np.exp(-max(0., dt)/tau))
        if alpha == 1.:
            error = previous[1]
        else:
            innovation = rng.multivariate_normal([0., 0., 0.], model['covariance'])
            error = innovation if previous is None else alpha*previous[1]+np.sqrt(1-alpha**2)*innovation
        self.vector_states[name] = stamp, error
        return values + np.asarray(model['bias']) + error

    def quaternion(self, xyzw, stamp=None):
        """Apply zero-centred stationary orientation fluctuations in the sensor tangent frame."""
        if not self.enabled:
            return np.asarray(xyzw, dtype=float).copy()
        error = self.vector('orientation', [0., 0., 0.], stamp)
        model = self.cfg['orientation']
        if model.get('outlier_probability', 0.) > 0:
            rng = self.rng('orientation', stamp)
            if rng.random() < model['outlier_probability']:
                error = model['outlier_rotvecs'][rng.integers(len(model['outlier_rotvecs']))]
        return (Rotation.from_quat(xyzw)*Rotation.from_rotvec(error)).as_quat()

    def covariance(self, name):
        """Expose only the fitted residual variance, excluding constant bias uncertainty."""
        model = self.cfg[name]
        covariance = np.asarray(model['covariance'])
        probability = model.get('outlier_probability', 0.)
        if probability > 0:
            outliers = np.asarray(model['outlier_rotvecs'])
            mean = probability*outliers.mean(0)
            covariance = ((1-probability)*covariance + probability*(outliers.T@outliers)/len(outliers)
                          - np.outer(mean, mean))
        return covariance.reshape(-1).tolist()
