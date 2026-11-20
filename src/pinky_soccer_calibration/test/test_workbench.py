"""Offline workbench tests; all calibration fixtures here are explicitly synthetic."""

import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from pinky_soccer_calibration import core
from pinky_soccer_calibration.jupyter_camera import JupyterCamera
from pinky_soccer_calibration.workbench_model import Experiment, SampleBag
from test_intrinsics import draw_board, model


class FakeRemote:
    info = {'model': 'synthetic', 'orientation': 'test-only', 'configuration': '800x600'}

    def __init__(self):
        self.index = {'calibration': 0, 'validation': 0}

    def call(self, action):
        assert action == 'new'

    def frame(self, purpose, min_sharpness):
        while True:
            index = self.index[purpose]
            self.index[purpose] += 1
            gray = draw_board(index, purpose == 'validation')
            if core.detect(gray) is not None:
                break
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return {'accepted': True, 'image': image,
                'png_bytes': cv2.imencode('.png', image)[1].tobytes(),
                'jpeg_bytes': cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 100])[1].tobytes(),
                'observed_unix_ns': 1_000_000_000 + index,
                'sharpness': float(cv2.Laplacian(gray, cv2.CV_64F).var())}

    def fit(self, square_mm):
        assert square_mm == 30.
        stream = io.BytesIO()
        np.savez(stream, camera_matrix=np.array(model()['camera_matrix']['data']).reshape(3, 3),
                 distortion_coefficients=np.zeros(5))
        return stream.getvalue()


def test_native_workflow_and_exact_bag_samples(tmp_path):
    pytest.importorskip('rosbag2_py')
    exp = Experiment(FakeRemote(), tmp_path, 30., 'synthetic', (800, 600))
    try:
        with pytest.raises(ValueError, match='학습 보정'):
            exp.capture('validation')
        with pytest.raises(ValueError, match='15장'):
            exp.fit()
        for _ in range(30):
            exp.capture('calibration')
        train = exp.fit(max_rms=1.)
        assert train['passed'], train['reasons']
        assert train['calibration_backend'] == 'pinkylib'
        assert '중심 범위' in exp.coverage('calibration')
        with pytest.raises(ValueError, match='고정'):
            exp.capture('calibration')
        with pytest.raises(ValueError, match='모두 완료'):
            exp.export(True)
        for _ in range(20):
            exp.capture('validation')
        valid = exp.validate(max_rms=1.)
        assert valid['passed'], valid['reasons']
        with pytest.raises(ValueError, match='visual'):
            exp.export(False)
        output = exp.export(True)
        assert (output / 'camera_calibration.npz').is_file()
        assert core.load(output / 'calibration.yaml')['camera_intrinsics']['image_orientation'] == 'test-only'
        assert (exp.root / 'raw_bag/metadata.yaml').is_file()
        with pytest.raises(ValueError, match='종료'):
            exp.capture('validation')
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(exp.root / 'raw_bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        messages = []
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            messages.append((topic, deserialize_message(raw, Image), stamp))
        assert len(messages) == 50
        raw_png = cv2.imread(str(exp.samples['calibration'][0]))
        assert bytes(messages[0][1].data) == raw_png.tobytes()
        assert messages[0][1].header.stamp.sec == 1
        exp.camera_path.with_name('camera_calibration.npz').write_bytes(b'changed-after-validation')
        with pytest.raises(ValueError, match='Native calibration changed'):
            core.export(exp.train_path, exp.valid_path, True)
    finally:
        exp.close()


def test_failed_fit_keeps_snapshots_and_disallows_validation(tmp_path):
    class NoopBag:
        def __init__(self, path):
            pass
        def write(self, *args):
            pass
        def close(self):
            pass
    remote = FakeRemote()
    exp = Experiment(remote, tmp_path, 30, 'synthetic', (800, 600), bag_factory=NoopBag)
    try:
        for _ in range(15):
            exp.capture('calibration')
        first = exp.fit(max_rms=.0001)
        assert not first['passed']
        original = exp.train_path.read_bytes()
        with pytest.raises(ValueError, match='학습 보정'):
            exp.capture('validation')
        exp.capture('calibration')
        exp.fit(max_rms=.0001)
        assert (exp.root / 'attempt_001/training/check/report.yaml').read_bytes() == original
        assert (exp.root / 'attempt_002/training/check/report.yaml').is_file()
        assert len(list((exp.root / 'calibration').glob('*.png'))) == 16
    finally:
        exp.close()


def test_kernel_transport_owns_only_its_kernel(monkeypatch):
    class FakeSocket:
        def send(self, raw):
            self.message = json.loads(raw)
            self.step = 0
        def settimeout(self, value):
            pass
        def recv(self):
            self.step += 1
            parent = {'msg_id': self.message['header']['msg_id']}
            if self.step == 1:
                data = {'model': 'test', 'native_calibration': True} if "run('info'" in self.message['content']['code'] else {'answer': 42}
                return json.dumps({'msg_type': 'stream', 'parent_header': parent,
                                   'content': {'text': 'PINKY_JSON:' + json.dumps(data) + '\n'}})
            return json.dumps({'msg_type': 'status', 'parent_header': parent,
                               'content': {'execution_state': 'idle'}})
        def close(self):
            pass
    monkeypatch.setitem(sys.modules, 'websocket', SimpleNamespace(
        create_connection=lambda *args, **kwargs: FakeSocket(),
        WebSocketTimeoutException=TimeoutError))
    client = JupyterCamera('http://robot:8888')
    calls = []
    def request(path, method='GET', data=None):
        calls.append((path, method))
        return {'id': 'owned-kernel'} if method == 'POST' else None
    monkeypatch.setattr(client, 'request', request)
    assert client.connect()['model'] == 'test'
    assert client.call('anything', value="'\n") == {'answer': 42}
    client.close()
    assert calls == [('/tree', 'GET'), ('/api/kernels', 'POST'),
                     ('/api/kernels/owned-kernel', 'DELETE')]
    assert client.kernel is None
    for url in ['file:///tmp/foo', 'http://user:pass@robot/', 'http://robot/?token=secret']:
        with pytest.raises(ValueError):
            JupyterCamera(url)


def test_camera_service_releases_lease_without_hardware(monkeypatch):
    import importlib.util
    import time
    import threading
    # Load the exact code sent to Jupyter, supplying a synthetic camera implementation.
    class FakeCamera:
        def start(self, **kwargs):
            pass
        def close(self):
            self.closed = True
    monkeypatch.setitem(sys.modules, 'pinkylib', SimpleNamespace(Camera=FakeCamera))
    path = Path(__file__).parents[1] / 'pinky_soccer_calibration/robot_service.py'
    spec = importlib.util.spec_from_file_location('synthetic_robot_service', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Do not launch an actual thread or register a lasting test camera with atexit.
    monkeypatch.setattr(module.threading, 'Thread', lambda **kwargs: SimpleNamespace(start=lambda: None))
    monkeypatch.setattr(module.atexit, 'register', lambda fn: None)
    service = module.RobotCameraService(640, 480)
    camera = service.camera
    service.last_seen = time.monotonic() - 31
    monkeypatch.setattr(module.time, 'sleep', lambda seconds: None)
    service._watchdog()
    assert camera.closed and service.camera is None


def test_validation_cannot_reuse_originals_after_jpeg_conversion(tmp_path):
    camera = tmp_path / 'camera.yaml'
    core.save(camera, model())
    reports = []
    for purpose in ('calibration', 'validation'):
        session = tmp_path / purpose
        session.mkdir()
        core.save(session / 'session.yaml', {'synthetic_fixture': True})
        core.save(session / 'acquisition.yaml', {'completed': True})
        path = session / 'report.yaml'
        core.save(path, {
            'session': str(session), 'purpose': purpose, 'passed': True,
            'session_sha256': core.digest(session / 'session.yaml'),
            'acquisition_sha256': core.digest(session / 'acquisition.yaml'),
            'calibration': str(camera), 'calibration_sha256': core.digest(camera),
            'camera_id': 'synthetic', 'camera_settings': 'test', 'image_size': [800, 600],
            'square_mm': 30, 'frame_id': 'test',
            'image_sha256': [purpose + '-different-encoding'],
            'source_image_sha256': ['same-original-image'],
        })
        reports.append(path)
    with pytest.raises(ValueError, match='reuses training'):
        core.export(*reports, visual_check=True)
