"""Synthetic fixtures exercise numerical checks; these are not robot measurements."""

import io
from pathlib import Path
import tarfile

import cv2
import numpy as np
import pytest

from pinky_soccer_calibration import core
from pinky_soccer_calibration.cli import gui_command, main


def model(k=None):
    k = np.array([[700., 0, 400], [0, 710., 300], [0, 0, 1]]) if k is None else k
    return {'image_width': 800, 'image_height': 600, 'camera_name': 'synthetic',
            'distortion_model': 'plumb_bob',
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': k.ravel().tolist()},
            'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': [0.] * 5},
            'rectification_matrix': {'rows': 3, 'cols': 3,
                                     'data': np.eye(3).ravel().tolist()},
            'projection_matrix': {'rows': 3, 'cols': 4,
                                  'data': np.c_[k, np.zeros(3)].ravel().tolist()}}


def fixture_session(root, purpose):
    root.mkdir()
    core.save(root / 'session.yaml', {'purpose': purpose, 'square_mm': 30.,
              'camera_id': 'synthetic', 'camera_settings': 'test-only',
              'image_topic': '/test/image'})
    core.save(root / 'acquisition.yaml', {'completed': True, 'image_size': [800, 600],
                                         'frame_id': 'test_optical'})
    return root


def draw_board(index, validation=False):
    """Render a planar board with known K and varied poses onto a gray background."""
    k = np.array(model()['camera_matrix']['data']).reshape(3, 3)
    rng = np.random.default_rng(index + (3000 if validation else 0))
    rotation = np.array([rng.uniform(-0.55, 0.55), rng.uniform(-0.55, 0.55),
                         rng.uniform(-0.2, 0.2)])
    depth = rng.uniform(0.6, 0.95)
    translation = np.array([rng.uniform(-0.3, 0.08), rng.uniform(-0.23, 0.08), depth])
    image = np.full((600, 800), 170, np.uint8)
    for row in range(7):
        for col in range(10):
            square = np.array([[col-1, row-1, 0], [col, row-1, 0],
                               [col, row, 0], [col-1, row, 0]], float) * .03
            projected, _ = cv2.projectPoints(square, rotation, translation, k, np.zeros(5))
            cv2.fillConvexPoly(image, np.round(projected).astype(np.int32),
                              255 if (row + col) % 2 else 0)
    return cv2.GaussianBlur(image, (3, 3), .6)


def make_dataset(root, purpose, count=35):
    fixture_session(root, purpose)
    folder = root / ('gui' if purpose == 'calibration' else 'frames')
    folder.mkdir()
    for index in range(count):
        cv2.imwrite(str(folder / f'{index:03d}.png'),
                    draw_board(index, purpose == 'validation'))
    return root


def test_known_model_residual_and_noise():
    k = np.array(model()['camera_matrix']['data']).reshape(3, 3)
    points = core.object_points(30)
    corners, _ = cv2.projectPoints(points, np.array([.2, -.3, .1]),
                                   np.array([-.1, -.1, .8]), k, np.zeros(5))
    residual, tilt = core.score_view(points, corners, k, np.zeros(5))
    assert np.sqrt(np.mean(residual**2)) < 1e-3
    assert tilt > 15
    noisy = corners + np.random.default_rng(42).normal(0, 1, corners.shape).astype(np.float32)
    residual, _ = core.score_view(points, noisy, k, np.zeros(5))
    assert np.sqrt(np.mean(residual**2)) > .5


def test_full_training_validation_export(tmp_path):
    train = make_dataset(tmp_path / 'train', 'calibration')
    valid = make_dataset(tmp_path / 'valid', 'validation')
    camera = train / 'gui/camera.yaml'
    core.save(camera, model())
    train_report = core.check(train, camera, max_rms=1.)
    valid_report = core.check(valid, camera, max_rms=1.)
    assert train_report['passed'], train_report['reasons']
    assert valid_report['passed'], valid_report['reasons']
    assert train_report['diagnostic_refit_parameter_std']['fx'] > 0
    assert (valid / 'check/corners.csv').is_file()
    training, validation = train / 'check/report.yaml', valid / 'check/report.yaml'
    with pytest.raises(ValueError, match='visual'):
        core.export(training, validation, False)
    output = core.export(training, validation, True)
    assert core.load(output / 'calibration.yaml')['camera_extrinsics'] is None
    core.camera_model(output / 'camera.yaml')
    # Changed inputs must invalidate the approval gate.
    camera.write_text(camera.read_text().replace('700.0', '701.0'))
    with pytest.raises(ValueError, match='changed'):
        core.export(training, validation, True)


def test_detection_empty_and_duplicate_fail(tmp_path):
    train = fixture_session(tmp_path / 'train', 'calibration')
    (train / 'gui').mkdir()
    core.save(train / 'gui/camera.yaml', model())
    image = draw_board(3)
    for i in range(16):
        cv2.imwrite(str(train / f'gui/{i}.png'), image)
    report = core.check(train, train / 'gui/camera.yaml')
    assert not report['passed']
    assert report['rms_px'] is None or report['rms_px']['count'] == 1
    with pytest.raises(FileExistsError):
        core.check(train, train / 'gui/camera.yaml')


def test_bad_calibration_and_square(tmp_path):
    path = tmp_path / 'camera.yaml'
    broken = model()
    broken['camera_matrix']['data'][0] = float('nan')
    core.save(path, broken)
    with pytest.raises(ValueError):
        core.camera_model(path)
    for value in [0, -1, float('inf'), float('nan')]:
        with pytest.raises(ValueError):
            core.object_points(value)
    assert core.stats([1, 2, 3])['sample_std'] == 1.
    assert core.stats([1])['sample_std'] is None


def test_secure_archive_and_gui(tmp_path):
    session = fixture_session(tmp_path / 'train', 'calibration')
    archive = tmp_path / 'gui.tar.gz'
    with tarfile.open(archive, 'w:gz') as target:
        import yaml
        for name, data in [('ost.yaml', yaml.safe_dump(model()).encode()),
                           ('left-0000.png', cv2.imencode('.png', draw_board(0))[1].tobytes())]:
            member = tarfile.TarInfo(name)
            member.size = len(data)
            target.addfile(member, io.BytesIO(data))
    assert core.import_archive(session, archive) == session / 'gui/camera.yaml'
    command = gui_command(session)
    assert command[command.index('--square') + 1] == '0.03'
    assert '--no-service-check' in command
    assert main(['check', '--session', str(session),
                 '--calibration', str(session / 'gui/camera.yaml')]) == 2
    bad_session = fixture_session(tmp_path / 'bad', 'calibration')
    with tarfile.open(archive, 'w:gz') as target:
        for name in ['ost.yaml', 'left-../../escape.png']:
            member = tarfile.TarInfo(name)
            member.size = 1
            target.addfile(member, io.BytesIO(b'x'))
    with pytest.raises(ValueError, match='Unsafe'):
        core.import_archive(bad_session, archive)


def test_repeat_summary_and_reuse_guard(tmp_path):
    reports = []
    for index in range(2):
        root = fixture_session(tmp_path / str(index), 'calibration')
        camera = root / 'camera.yaml'
        data = model()
        data['camera_matrix']['data'][0] += index * 2
        core.save(camera, data)
        report = root / 'report.yaml'
        core.save(report, {'purpose': 'calibration', 'passed': True,
                  'session': str(root), 'calibration': str(camera),
                  'calibration_sha256': core.digest(camera), 'camera_id': 'synthetic',
                  'camera_settings': 'test', 'square_mm': 30, 'image_size': [800, 600],
                  'frame_id': 'test', 'image_sha256': [str(index)]})
        reports.append(report)
    summary = core.summarize(reports, tmp_path / 'summary.yaml')
    assert summary['parameters']['fx_px']['mean'] == 701
    assert summary['parameters']['fx_px']['sample_std'] == pytest.approx(np.sqrt(2))
    with pytest.raises(ValueError, match='same session'):
        core.summarize([reports[0], reports[0]], tmp_path / 'bad.yaml')
