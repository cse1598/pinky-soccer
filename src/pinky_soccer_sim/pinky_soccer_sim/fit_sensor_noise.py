#!/usr/bin/env python3
"""Fit portable simulation error parameters from immutable calibration observations."""

import argparse
import hashlib
import sqlite3
from datetime import datetime
from pathlib import Path

import numpy as np

from scipy.spatial.transform import Rotation

import yaml


def vector_model(values):
    """Fit the sample mean and full three-axis covariance in SI units."""
    values = np.asarray(values, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 2 or not np.isfinite(values).all():
        raise ValueError('At least two finite vector samples are required')
    return {'bias': values.mean(0).tolist(), 'covariance': np.cov(values, rowvar=False).tolist(),
            'samples': len(values)}


def correlated_vector_model(groups, period):
    """Fit a stationary covariance and first-lag decay without joining separate runs."""
    model = vector_model(np.concatenate(groups))
    centered = [np.asarray(group)-np.mean(group, axis=0) for group in groups]
    denominator = sum(float(np.sum(group[:-1]**2)) for group in centered)
    numerator = sum(float(np.sum(group[:-1]*group[1:])) for group in centered)
    rho = np.clip(numerator/denominator, 0., .999999) if denominator > 1e-20 else 0.
    model.update(sample_period_s=period, correlation_time_s=float(-period/np.log(rho)) if rho > 0 else 0.,
                 lag_one_correlation=float(rho))
    return model


def lidar_ray_noise(bag, row):
    """Estimate common/independent frontal ray variation during the recorded stationary run."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    end = int(datetime.fromisoformat(row['recorded_utc']).timestamp()*1e9)
    start = end-10_000_000_000
    uri = Path(bag).resolve().as_uri()+'?mode=ro'
    with sqlite3.connect(uri, uri=True) as database:
        topic = database.execute("SELECT id FROM topics WHERE name='/scan'").fetchone()
        if topic is None:
            raise ValueError('Raw bag has no /scan topic')
        messages = [deserialize_message(data, LaserScan) for data, in database.execute(
            'SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
            (topic[0], start, end))]
    if len(messages) < 20:
        raise ValueError('Raw bag does not cover the stationary LiDAR measurement')
    first = messages[0]
    angles = first.angle_min + np.arange(len(first.ranges))*first.angle_increment
    front = np.abs((angles+2*np.pi) % (2*np.pi)-np.pi) < .035
    values = np.asarray([m.ranges for m in messages])[:, front]
    values[~np.isfinite(values)] = np.nan
    valid = np.isfinite(values).mean(0) >= .8
    values = values[:, valid]
    if values.shape[1] < 3:
        raise ValueError('Not enough continuously visible frontal rays')
    reference = row['reference']['sensor_reference_m']
    projected = np.nanmedian(values*(-np.cos(angles[front][valid])), axis=1)
    # Guard against the wrong time window or a moving robot/wall.
    reported = np.mean([v['distance'] for v in row['samples']])
    if abs(projected.mean()-reported) > .003 or projected.std(ddof=1) > .003:
        raise ValueError('Raw scan window does not match the stationary measurement')
    present = np.isfinite(values).astype(float)
    residual = np.nan_to_num(values-np.nanmean(values, axis=0))
    cov = (residual.T@residual)/(present.T@present-1.)
    common_var = max(0., float((cov.sum()-np.trace(cov))/(cov.size-len(cov))))
    total_var = float(np.diag(cov).mean())
    common_var = min(common_var, total_var)
    return [float(np.sqrt(total_var-common_var)), float(np.sqrt(common_var))], {
        'messages': len(messages), 'frontal_rays': values.shape[1],
        'window_start_ns': start, 'window_end_ns': end,
        'raw_projected_mean_m': float(projected.mean()), 'reference_m': reference,
        'aggregation': 'per_ray_temporal_covariance_after_removing_each_ray_mean'}


def fit_project(project, lidar_bag):
    """Use raw samples and gravity-referenced IMU residuals; never invent unknown channels."""
    project = Path(project)
    data = yaml.safe_load(project.read_text())
    if data['meta']['environment'] != 'real':
        raise ValueError('A physical robot calibration project is required')
    measurements = data['measurements']
    cfg = {'schema_version': 1, 'provenance': {
        'source_project': project.parent.name,
        'source_project_sha256': hashlib.sha256(project.read_bytes()).hexdigest(),
        'source_lidar_bag': Path(lidar_bag).parent.name,
        'method': ('Gaussian covariance; IMU first-lag exponential correlation; '
                   'range noise independent across scans'),
        'range_extension': ('Frontal flat-wall data applied to all rays; '
                            'endpoint parameters held outside measured distances'),
        'unmodelled': ['camera image noise and detection error', 'IR ADC noise', 'battery noise',
                       'encoder/odometry noise', 'transport latency', 'magnetometer error',
                       'dynamic IMU error and long-term drift'],
        'camera_ball_x_correction_m': data['meta'].get('ball_review', {}).get('offset_xy_m', [0, 0])[0],
        'camera_note': ('Review-only geometric correction, not image noise; '
                        'not injected into camera pixels'),
        'accel_reference': ('Recorded orientation inverse-rotated gravity 9.81 m/s^2; '
                            'attitude outliers use the regular run mean'),
        'orientation_reference': ('Robust per-run mean removed; '
                                  'large isolated residuals replayed separately; '
                                  'absolute attitude accuracy not measured')}}
    for name in ('lidar', 'ultrasonic'):
        table, runs = [], []
        for row in measurements[name]:
            samples = np.array([v['distance'] for v in row['samples']], dtype=float)
            if len(samples) < 2 or not np.isfinite(samples).all():
                raise ValueError('Finite raw range samples are required')
            reference = float(row['reference']['sensor_reference_m'])
            sigma, details = ([float(samples.std(ddof=1)), 0.], {})
            if name == 'lidar':
                sigma, details = lidar_ray_noise(lidar_bag, row)
            table.append([reference, float(samples.mean()-reference), *sigma])
            runs.append({'id': row['id'], 'samples': len(samples),
                         'recorded_utc': row['recorded_utc'], **details})
        cfg[name] = {'extrapolation': 'hold_endpoint_parameters', 'table': sorted(table), 'runs': runs}
    gyro, accel, orientation, outliers = [], [], [], []
    intervals = [delta for row in measurements.get('timing', [])
                 for delta in np.diff([v[0] for v in row.get('topics', {}).get('/imu_raw', [])])
                 if 0 < delta < .1]
    period = float(np.median(intervals)) if intervals else .01
    for row in measurements['imu_stationary']:
        samples = row['samples']
        rotations = Rotation.from_quat([v['orientation_xyzw'] for v in samples])
        preliminary = (rotations.mean().inv()*rotations).as_rotvec()
        distances = np.linalg.norm(preliminary, axis=1)
        median = np.median(distances)
        threshold = max(.02, float(median+10*np.median(abs(distances-median))))
        regular = distances <= threshold
        baseline = rotations[regular].mean()
        residual = (baseline.inv()*rotations).as_rotvec()
        outliers.extend(residual[~regular])
        # Corrupt attitude samples must not turn ordinary tilted gravity into acceleration error.
        gravity_rotations = rotations.as_quat()
        gravity_rotations[~regular] = baseline.as_quat()
        acc = np.array([[v['linear_acceleration_'+a] for a in 'xyz'] for v in samples])
        accel.append(acc-Rotation.from_quat(gravity_rotations).inv().apply(
            np.tile([0., 0., 9.81], (len(samples), 1))))
        gyro.append([[v['angular_velocity_'+a] for a in 'xyz'] for v in samples])
        edges = np.flatnonzero(np.diff(np.r_[False, regular, False]))
        orientation.extend(residual[start:end] for start, end in zip(edges[::2], edges[1::2]))
    for name, values in [('gyro', gyro), ('accel', accel), ('orientation', orientation)]:
        cfg[name] = correlated_vector_model(values, period)
    cfg['orientation']['bias'] = [0., 0., 0.]
    total = sum(len(row['samples']) for row in measurements['imu_stationary'])
    cfg['orientation'].update(outlier_probability=len(outliers)/total,
                              outlier_rotvecs=np.asarray(outliers).reshape(-1, 3).tolist(),
                              total_samples=total, outlier_samples=len(outliers),
                              outlier_policy=('Empirical stationary attitude jumps '
                                              'relative to robust per-run mean'))
    return cfg


def main():
    """Write a reviewable fitted YAML without modifying any calibration evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', type=Path)
    parser.add_argument('--lidar-bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    model = fit_project(args.project, args.lidar_bag)
    args.output.write_text(yaml.safe_dump(model, sort_keys=False))
    print(args.output)


if __name__ == '__main__':
    main()
