"""Offline checks of ROS camera_calibration results; no hardware measurements invented."""

import csv
import hashlib
from pathlib import Path
import shutil
import tarfile

import cv2
import numpy as np
import yaml


def load(path):
    with Path(path).open() as stream:
        return yaml.safe_load(stream)


def save(path, data):
    """Create a YAML file without overwriting earlier experiment results."""
    with Path(path).open('x') as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def positive(value):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError('Value must be finite and positive')
    return value


def stats(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0 or not np.isfinite(values).all():
        raise ValueError('Statistics require finite observations')
    return {'count': int(values.size), 'mean': float(values.mean()),
            'sample_std': float(values.std(ddof=1)) if values.size > 1 else None,
            'max': float(values.max())}


def object_points(square_mm):
    points = np.zeros((54, 3), np.float32)
    points[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * (positive(square_mm) / 1000)
    return points


def camera_model(path):
    data = load(path)
    if data['distortion_model'] != 'plumb_bob':
        raise ValueError('This experiment supports pinhole plumb_bob only, not fisheye')
    k = np.asarray(data['camera_matrix']['data'], dtype=float).reshape(3, 3)
    d = np.asarray(data['distortion_coefficients']['data'], dtype=float).reshape(-1)
    size = (int(data['image_width']), int(data['image_height']))
    if (not np.isfinite(k).all() or not np.isfinite(d).all() or len(d) != 5
            or min(size) <= 0 or k[0, 0] <= 0 or k[1, 1] <= 0
            or not 0 <= k[0, 2] < size[0] or not 0 <= k[1, 2] < size[1]
            or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0):
        raise ValueError('Invalid pinhole matrix, principal point, dimensions or coefficients')
    for key, shape in [('rectification_matrix', (3, 3)), ('projection_matrix', (3, 4))]:
        matrix = np.asarray(data[key]['data'], dtype=float).reshape(shape)
        if not np.isfinite(matrix).all():
            raise ValueError(f'Invalid {key}')
    return data, k, d, size


def import_archive(session, archive):
    """Read only expected regular members, never extract paths or links from tar."""
    session, archive = Path(session), Path(archive)
    if load(session / 'session.yaml')['purpose'] != 'calibration':
        raise ValueError('Import GUI results into a calibration session only')
    with tarfile.open(archive, 'r:gz') as source:
        members = source.getmembers()
        images = [m for m in members if m.name.startswith('left-') and m.name.endswith('.png')]
        selected = images + [m for m in members if m.name == 'ost.yaml']
        if not images or sum(m.name == 'ost.yaml' for m in selected) != 1:
            raise ValueError('Expected monocular ROS archive: ost.yaml and left-*.png')
        if len({m.name for m in selected}) != len(selected):
            raise ValueError('Duplicate archive members')
        if any(not m.isfile() or Path(m.name).name != m.name or m.size > 50_000_000
               for m in selected) or sum(m.size for m in selected) > 1_000_000_000:
            raise ValueError('Unsafe or oversized archive')
        destination = session / 'gui'
        destination.mkdir()
        shutil.copyfile(archive, destination / 'calibrationdata.tar.gz')
        for member in selected:
            name = 'camera.yaml' if member.name == 'ost.yaml' else member.name
            with source.extractfile(member) as stream:
                (destination / name).write_bytes(stream.read())
    camera_model(destination / 'camera.yaml')
    return destination / 'camera.yaml'


def detect(gray):
    found, corners = cv2.findChessboardCorners(
        gray, (9, 6), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))


def score_view(points, corners, k, d):
    """Fit only the board pose, keeping intrinsics frozen on held-out frames."""
    ok, rotation, translation = cv2.solvePnP(points, corners, k, d)
    if not ok:
        raise ValueError('Board pose estimation failed')
    projected, _ = cv2.projectPoints(points, rotation, translation, k, d)
    residual = (corners - projected).reshape(-1, 2)
    matrix, _ = cv2.Rodrigues(rotation)
    depths = (points @ matrix.T + translation.reshape(1, 3))[:, 2]
    if np.any(depths <= 0):
        raise ValueError('Estimated board lies behind camera')
    tilt = np.degrees(np.arccos(np.clip(abs(matrix[2, 2]), 0, 1)))
    return residual, float(tilt)


def check(session, calibration, max_rms=0.7, min_views=None):
    session, calibration = Path(session).resolve(), Path(calibration).resolve()
    meta = load(session / 'session.yaml')
    acquisition = load(session / 'acquisition.yaml')
    if not acquisition['completed']:
        raise ValueError('Recording incomplete; use a new session')
    meta.update(acquisition)
    model, k, d, size = camera_model(calibration)
    if model.get('camera_name') != meta['camera_id']:
        raise ValueError('Camera name differs from recorded camera identity')
    if list(size) != meta['image_size']:
        raise ValueError('Recorded resolution differs from calibration')
    positive(max_rms)
    training = meta['purpose'] == 'calibration'
    backend = meta.get('calibration_backend', 'ros_camera_calibration')
    folder = 'native' if backend == 'pinkylib' else 'gui'
    if training and calibration != session / folder / 'camera.yaml':
        raise ValueError('Training check must use this session\'s fitted result')
    minimum = min_views if min_views is not None else (15 if training else 8)
    if minimum < (15 if training else 8):
        raise ValueError('At least 15 training or 8 validation views required')
    files = sorted((session / (folder if training else 'frames')).glob('*.png'))
    output = session / 'check'
    output.mkdir()
    (output / 'previews').mkdir()
    points = object_points(meta['square_mm'])
    rows, errors, centers, tilts, areas, hashes, accepted = [], [], [], [], [], [], []
    with (output / 'corners.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['image', 'corner', 'u_px', 'v_px', 'du_px', 'dv_px'])
        for file in files:
            gray = cv2.imread(str(file), cv2.IMREAD_GRAYSCALE)
            if gray is None or gray.shape[::-1] != size:
                raise ValueError(f'Unreadable image or changed resolution: {file}')
            corners = detect(gray)
            if corners is None:
                rows.append([file.name, False, None, None])
                continue
            residual, tilt = score_view(points, corners, k, d)
            rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
            image_hash = digest(file)
            if image_hash in hashes:
                rows.append([file.name, False, None, 'duplicate'])
                continue
            hashes.append(image_hash)
            accepted.append(corners)
            rows.append([file.name, True, rms, tilt])
            errors.append(rms)
            centers.append(corners.reshape(-1, 2).mean(axis=0) / size)
            tilts.append(tilt)
            hull = cv2.convexHull(corners)
            areas.append(cv2.contourArea(hull) / (size[0] * size[1]))
            for index, (corner, delta) in enumerate(zip(corners.reshape(-1, 2), residual)):
                writer.writerow([file.name, index, *map(float, corner), *map(float, delta)])
            overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(overlay, (9, 6), corners, True)
            if not cv2.imwrite(str(output / 'previews' / file.name), overlay):
                raise OSError('Cannot save preview')
            # Full-sized undistorted image; original images remain untouched.
            if not cv2.imwrite(str(output / 'previews' / ('undistorted-' + file.name)),
                               cv2.undistort(gray, k, d)):
                raise OSError('Cannot save undistorted preview')
    with (output / 'views.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['image', 'used', 'rms_px', 'tilt_deg_or_rejection'])
        writer.writerows(rows)
    reasons = []
    if len(errors) < minimum:
        reasons.append(f'Need {minimum} distinct detected views; got {len(errors)}')
    span = np.ptp(centers, axis=0).tolist() if centers else [0.0, 0.0]
    area_ratio = max(areas) / min(areas) if areas else 0.0
    # Heuristics, not claims about measurement accuracy; reported for operator review.
    if min(span) < 0.25 or area_ratio < 1.5 or max(tilts, default=0) < 15:
        reasons.append('Insufficient position / distance / tilt diversity')
    if errors and max(errors) > max_rms:
        reasons.append('One or more views exceed the configured RMS limit')
    # Reject folding radial models over the image radius.
    radius = max(np.linalg.norm([(x - k[0, 2]) / k[0, 0],
                                 (y - k[1, 2]) / k[1, 1]])
                 for x in (0, size[0]) for y in (0, size[1]))
    r = np.linspace(0, radius, 200)
    derivative = 1 + 3*d[0]*r**2 + 5*d[1]*r**4 + 7*d[4]*r**6
    if np.any(derivative <= 0):
        reasons.append('Radial model folds within the image; recalibrate')
    uncertainty = None
    if training and len(accepted) >= 15:
        # Diagnostic re-fit only. The official GUI result remains the delivered model.
        fit = cv2.calibrateCameraExtended([points] * len(accepted), accepted, size, None, None)
        uncertainty = dict(zip(
            ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3'],
            map(float, fit[5].reshape(-1)[:9])))
    report = {
        'schema_version': 1, 'stage': 'camera_intrinsics', 'purpose': meta['purpose'],
        'session': str(session), 'camera_id': meta['camera_id'],
        'session_sha256': digest(session / 'session.yaml'),
        'acquisition_sha256': digest(session / 'acquisition.yaml'),
        'camera_settings': meta['camera_settings'], 'square_mm': meta['square_mm'],
        'image_size': list(size), 'frame_id': meta.get('frame_id'),
        'calibration': str(calibration), 'calibration_sha256': digest(calibration),
        'image_sha256': hashes, 'passed': not reasons, 'reasons': reasons,
        'source_image_sha256': meta.get('source_image_sha256', hashes),
        'calibration_backend': backend,
        'image_orientation': meta.get('image_orientation', 'driver image orientation'),
        'native_npz_sha256': (digest(calibration.with_name('camera_calibration.npz'))
                              if calibration.with_name('camera_calibration.npz').is_file() else None),
        'rms_px': stats(errors) if errors else None,
        'policy': {'max_per_view_rms_px': max_rms, 'min_views': minimum,
                   'min_center_span_xy': 0.25, 'min_area_ratio': 1.5, 'min_tilt_deg': 15},
        'coverage': {'center_span_xy': span, 'area_ratio': area_ratio,
                     'max_tilt_deg': max(tilts, default=0)},
        'diagnostic_refit_parameter_std': uncertainty,
        'note': 'Reprojection checks do not prove metric accuracy or base_link extrinsics.',
    }
    save(output / 'report.yaml', report)
    return report


def export(training, validation, visual_check):
    if not visual_check:
        raise ValueError('Inspect corners and undistorted previews, then pass --visual-check')
    train, valid = load(training), load(validation)
    for report in (train, valid):
        session = Path(report['session'])
        if (digest(session / 'session.yaml') != report['session_sha256']
                or digest(session / 'acquisition.yaml') != report['acquisition_sha256']):
            raise ValueError('Experiment metadata changed after checking')
    if train['purpose'] != 'calibration' or valid['purpose'] != 'validation':
        raise ValueError('Need training and independent validation reports')
    if not train['passed'] or not valid['passed']:
        raise ValueError('Both reports must pass before exporting')
    for field in ('camera_id', 'camera_settings', 'image_size', 'square_mm',
                  'frame_id', 'calibration_sha256'):
        if train[field] != valid[field]:
            raise ValueError(f'Incompatible experiments: {field}')
    reused = (set(train.get('source_image_sha256', train['image_sha256']))
              & set(valid.get('source_image_sha256', valid['image_sha256'])))
    if (train['session'] == valid['session'] or reused
            or set(train['image_sha256']) & set(valid['image_sha256'])):
        raise ValueError('Validation reuses training images')
    source = Path(train['calibration'])
    if digest(source) != train['calibration_sha256']:
        raise ValueError('Calibration changed after checking')
    native_npz = source.with_name('camera_calibration.npz')
    if train.get('native_npz_sha256') and digest(native_npz) != train['native_npz_sha256']:
        raise ValueError('Native calibration changed after checking')
    model, _, _, _ = camera_model(source)
    output = Path(train['session']) / 'approved'
    output.mkdir()
    save(output / 'camera.yaml', model)
    native_uri = None
    if train.get('native_npz_sha256'):
        shutil.copyfile(native_npz, output / 'camera_calibration.npz')
        native_uri = (output / 'camera_calibration.npz').as_uri()
    save(output / 'calibration.yaml', {
        'schema_version': 1, 'camera_id': train['camera_id'],
        'camera_intrinsics': {'status': 'validated',
                              'camera_info_url': (output / 'camera.yaml').as_uri(),
                              'sha256': digest(output / 'camera.yaml'),
                              'image_orientation': train.get('image_orientation'),
                              'calibration_backend': train.get('calibration_backend'),
                              'native_npz_uri': native_uri,
                              'native_npz_sha256': train.get('native_npz_sha256'),
                              'training_report': str(Path(training).resolve()),
                              'validation_report': str(Path(validation).resolve()),
                              'visual_check_confirmed': True},
        'camera_extrinsics': None, 'wheel_measurements': None,
        'odometry': None, 'lidar': None, 'imu_ultrasonic': None,
    })
    return output


def summarize(paths, output):
    reports = [load(path) for path in paths]
    if len(reports) < 2:
        raise ValueError('At least two independent experiments required')
    if len({r['session'] for r in reports}) != len(reports):
        raise ValueError('Repeated report from the same session')
    first = reports[0]
    samples = []
    for report in reports:
        if report['purpose'] != 'calibration' or not report['passed']:
            raise ValueError('Use passing calibration reports only')
        for key in ('camera_id', 'camera_settings', 'image_size', 'square_mm', 'frame_id'):
            if report[key] != first[key]:
                raise ValueError(f'Cannot pool different {key}')
        if digest(report['calibration']) != report['calibration_sha256']:
            raise ValueError('Calibration changed after checking')
        _, k, d, _ = camera_model(report['calibration'])
        samples.append([k[0, 0], k[1, 1], k[0, 2], k[1, 2], *d])
    for i, report in enumerate(reports):
        for previous in reports[:i]:
            if set(report['image_sha256']) & set(previous['image_sha256']):
                raise ValueError('Repeated experiment reuses images')
    result = {'schema_version': 1, 'camera_id': first['camera_id'],
              'reports': [str(Path(p).resolve()) for p in paths],
              'parameters': {name: stats(column) for name, column in zip(
                  ['fx_px', 'fy_px', 'cx_px', 'cy_px', 'k1', 'k2', 'p1', 'p2', 'k3'],
                  np.asarray(samples).T)},
              'note': 'Diagnostic repeat statistics only; mean is not an approved camera model.'}
    save(output, result)
    return result
