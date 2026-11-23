"""Measured-error moments, valid ROS observations, and shared simulator routing."""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from pinky_soccer_sim.sensor_adapter import SensorAdapter
from pinky_soccer_sim.sensor_noise import SensorNoise, load_noise

import pytest

from sensor_msgs.msg import Imu, LaserScan

import yaml

CONFIG = Path(__file__).parents[1]/'config/measured_sensor_noise.yaml'


def test_ultrasonic_reproduces_measured_distance_dependent_bias_and_spread():
    """All three independently measured distance anchors retain their SI moments."""
    cfg = load_noise(CONFIG)
    model = SensorNoise(cfg)
    for distance, bias, sigma, common in cfg['ultrasonic']['table']:
        errors = model.ranges('ultrasonic', np.full(50000, distance), .02, 3.)-distance
        assert errors.mean() == pytest.approx(bias, abs=sigma*.02)
        assert errors.std(ddof=1) == pytest.approx(sigma, rel=.02)
        assert common == 0


def test_lidar_reproduces_per_ray_and_common_scan_variance():
    """Shared scan noise prevents a wide median from becoming unrealistically ideal."""
    cfg = load_noise(CONFIG)
    model = SensorNoise(cfg)
    for distance, bias, independent, common in cfg['lidar']['table']:
        errors = np.array([model.ranges('lidar', np.full(100, distance), .05, 12.)-distance
                           for _ in range(3000)])
        assert errors.mean() == pytest.approx(bias, abs=.00003)
        assert np.mean(errors.var(axis=0, ddof=1)) == pytest.approx(independent**2+common**2, rel=.08)
        assert errors.mean(1).var(ddof=1) == pytest.approx(common**2+independent**2/100, rel=.08)


def test_invalid_returns_endpoint_extrapolation_and_seed_isolation():
    """No returns stay absent and unmeasured ranges never clamp to calibration distance."""
    cfg = load_noise(CONFIG)
    model = SensorNoise(cfg, seed=7)
    values = [np.nan, np.inf, -np.inf, .01, 20.]
    assert np.allclose(model.ranges('lidar', values, .05, 12.), values, equal_nan=True)
    for key in ('lidar', 'ultrasonic'):
        for row in cfg[key]['table']:
            row[2:] = [0., 0.]
    model, other = SensorNoise(cfg, 7), SensorNoise(cfg, 7)
    assert model.ranges('lidar', [4.], .05, 12.)[0] == pytest.approx(4.+cfg['lidar']['table'][-1][1])
    for _ in range(10):
        model.vector('accel', [0., 0., 9.81])
    assert model.vector('gyro', [0., 0., 0.]) == pytest.approx(other.vector('gyro', [0., 0., 0.]))
    model = SensorNoise(cfg, 7)
    initial = model.ranges('lidar', [.2], .05, 12., 100)
    model.ranges('lidar', [.2], .05, 12., 200)
    assert model.ranges('lidar', [.2], .05, 12., 1) == pytest.approx(initial)
    ideal = SensorNoise(cfg, enabled=False)
    assert ideal.ranges('ultrasonic', [.2], .02, 3.) == [.2]
    assert ideal.vector('accel', [0., 0., 9.81]) == pytest.approx([0., 0., 9.81])


def test_imu_full_covariance_and_unit_quaternion():
    """Stationary gravity is a physical input, never an additional constant noise vector."""
    cfg = load_noise(CONFIG)
    model = SensorNoise(cfg)
    for key, truth in [('gyro', [0., 0., 1.2]), ('accel', [0., 0., 9.81])]:
        cfg[key]['correlation_time_s'] = 0.
    model = SensorNoise(cfg)
    for key, truth in [('gyro', [0., 0., 1.2]), ('accel', [0., 0., 9.81])]:
        errors = np.array([model.vector(key, truth)-truth for _ in range(15000)])
        covariance = np.array(cfg[key]['covariance'])
        assert errors.mean(0) == pytest.approx(cfg[key]['bias'], abs=.003)
        assert np.diag(np.cov(errors, rowvar=False)) == pytest.approx(np.diag(covariance), rel=.05)
    for _ in range(100):
        assert np.linalg.norm(model.quaternion([0., 0., 0., 1.])) == pytest.approx(1.)


@pytest.mark.parametrize('mutation', ['sigma', 'covariance', 'schema'])
def test_corrupt_model_rejected(tmp_path, mutation):
    """Configuration failures stop startup instead of publishing invalid observations."""
    cfg = yaml.safe_load(CONFIG.read_text())
    if mutation == 'sigma':
        cfg['lidar']['table'][0][2] = -.1
    elif mutation == 'covariance':
        cfg['gyro']['covariance'][0][0] = -1.
    else:
        cfg['schema_version'] = 2
    path = tmp_path/'bad.yaml'
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):
        load_noise(path)


def test_public_callbacks_preserve_headers_inputs_and_native_imu_units():
    """One public source applies errors once while the simulator's ideal messages stay intact."""
    from sensor_msgs.msg import Range
    scan_out, imu_out, native_out, us_out = [], [], [], []
    silent = SimpleNamespace(publish=lambda value: None)
    fake = SimpleNamespace(noise=SensorNoise(load_noise(CONFIG)), key=SensorAdapter.key,
                           scan=SimpleNamespace(publish=scan_out.append),
                           imu=SimpleNamespace(publish=imu_out.append),
                           native=SimpleNamespace(publish=native_out.append),
                           mag=silent, mag_status=silent, world_field=[22e-6, 0., -42e-6],
                           us=SimpleNamespace(publish=us_out.append),
                           cfg={'ultrasonic': {'min_range': .02, 'max_range': 3., 'field_of_view': .26}})
    scan = LaserScan(range_min=.05, range_max=12., ranges=[.2, float('inf')])
    scan.header.stamp.sec, scan.header.frame_id = 3, 'rplidar_link'
    before = deepcopy(scan)
    SensorAdapter.scan_received(fake, scan)
    assert scan == before and scan_out[0].header == scan.header
    assert scan_out[0].ranges[0] != scan.ranges[0]
    assert scan_out[0].ranges[1] == float('inf')
    imu = Imu()
    imu.orientation.w, imu.linear_acceleration.z, imu.angular_velocity.z = 1., 9.81, 1.2
    original = deepcopy(imu)
    SensorAdapter.imu_received(fake, imu)
    assert imu == original
    assert native_out[0].angular_velocity.z == pytest.approx(np.degrees(imu_out[0].angular_velocity.z))
    assert np.diag(np.array(imu_out[0].linear_acceleration_covariance).reshape(3, 3)).min() > 0
    SensorAdapter.range_received(fake, 'ultrasonic_link', scan)
    assert us_out[0].radiation_type == Range.ULTRASOUND
    assert us_out[0].range != scan.ranges[0]


def test_fitting_removes_tilted_gravity_and_separates_stationary_attitudes(tmp_path, monkeypatch):
    """Changing between two fixed robot headings must not become attitude or gravity noise."""
    from pinky_soccer_sim.fit_sensor_noise import fit_project
    from scipy.spatial.transform import Rotation
    rows = []
    for yaw in (.2, 1.4):
        rot = Rotation.from_euler('xyz', [.18, -.05, yaw])
        gravity = rot.inv().apply([0., 0., 9.81])
        samples = []
        for jitter in (-.01, 0., .01):
            sample = {'orientation_xyzw': rot.as_quat().tolist()}
            sample.update({'linear_acceleration_'+axis: float(gravity[i]+jitter+.02)
                           for i, axis in enumerate('xyz')})
            sample.update({'angular_velocity_'+axis: jitter+.001 for axis in 'xyz'})
            samples.append(sample)
        rows.append({'samples': samples})
    ranges = [{'id': 'test', 'samples': [{'distance': .29}, {'distance': .31}],
               'reference': {'sensor_reference_m': .3}, 'recorded_utc': '2026-11-24T00:00:00+00:00'}]
    path = tmp_path/'project.yaml'
    path.write_text(yaml.safe_dump({'meta': {'environment': 'real'}, 'measurements': {
        'lidar': ranges, 'ultrasonic': ranges, 'imu_stationary': rows}}))
    original = path.read_bytes()
    monkeypatch.setattr('pinky_soccer_sim.fit_sensor_noise.lidar_ray_noise',
                        lambda bag, row: ([.001, .0002], {}))
    fitted = fit_project(path, tmp_path/'fake.db3')
    assert fitted['accel']['bias'] == pytest.approx([.02]*3)
    assert fitted['gyro']['bias'] == pytest.approx([.001]*3)
    assert np.max(np.abs(fitted['orientation']['covariance'])) < 1e-20
    assert path.read_bytes() == original


def test_temporal_correlation_uses_simulation_time_and_replays_after_reset():
    """Slow measured attitude variation must not turn into independent frame jitter."""
    cfg = load_noise(CONFIG)
    cfg['orientation'].update(bias=[0.]*3, covariance=np.eye(3).tolist(), correlation_time_s=.1)
    model = SensorNoise(cfg, seed=19)
    errors = np.array([model.vector('orientation', [0.]*3, index*10_000_000)
                       for index in range(20000)])
    assert np.corrcoef(errors[:-1, 0], errors[1:, 0])[0, 1] == pytest.approx(np.exp(-.1), abs=.015)
    assert errors[:, 0].var() == pytest.approx(1., rel=.1)
    same = model.vector('orientation', [0.]*3, 19999*10_000_000)
    assert same == pytest.approx(errors[-1])
    assert model.vector('orientation', [0.]*3, 0) == pytest.approx(errors[0])


def test_attitude_outliers_remain_rare_jumps_with_mixture_covariance():
    """Large recorded angle errors are replayed sparsely rather than widening every frame."""
    from scipy.spatial.transform import Rotation
    cfg = load_noise(CONFIG)
    cfg['orientation'].update(covariance=np.zeros((3, 3)).tolist(), bias=[0.]*3,
                              outlier_probability=.01, outlier_rotvecs=[[0., 0., .6]])
    model = SensorNoise(cfg, seed=23)
    quaternions = np.array([model.quaternion([0., 0., 0., 1.], i*10_000_000) for i in range(20000)])
    angles = Rotation.from_quat(quaternions).as_rotvec()[:, 2]
    assert np.mean(angles > .3) == pytest.approx(.01, abs=.002)
    assert np.max(abs(angles[angles <= .3])) < 1e-12
    assert np.array(model.covariance('orientation')).reshape(3, 3)[2, 2] == pytest.approx(.01*.99*.6**2)
