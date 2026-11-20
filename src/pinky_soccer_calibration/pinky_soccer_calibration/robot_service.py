"""Source sent to a private robot kernel; depends only on the robot's existing stack."""

import atexit
import base64
import os
from pathlib import Path
import tempfile
import threading
import time

import cv2
from pinkylib import Camera


class RobotCameraService:
    """Own one camera with an idle lease; never touch another kernel's camera."""

    def __init__(self, width, height):
        self.camera = None
        self.lock = threading.RLock()
        self.last_seen = time.monotonic()
        self.root = None
        self.counts = {'calibration': 0, 'validation': 0}
        try:
            self.camera = Camera()
            self.camera.start(width=width, height=height)
        except Exception:
            self.close()
            raise
        atexit.register(self.close)
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _watchdog(self):
        while self.camera is not None:
            time.sleep(2)
            with self.lock:
                if time.monotonic() - self.last_seen > 30:
                    self.close()

    def close(self):
        with self.lock:
            camera, self.camera = self.camera, None
            if camera is not None:
                camera.close()

    @staticmethod
    def encode(extension, frame):
        options = [cv2.IMWRITE_JPEG_QUALITY, 100] if extension == '.jpg' else []
        ok, data = cv2.imencode(extension, frame, options)
        if not ok:
            raise RuntimeError('Camera image encoding failed')
        return data.tobytes()

    def run(self, action, params):
        with self.lock:
            self.last_seen = time.monotonic()
            if action == 'close':
                self.close()
                return {'closed': True}
            if self.camera is None:
                raise RuntimeError('카메라 연결이 만료되었습니다. 다시 연결해 주세요.')
            if action == 'info':
                return {'model': self.camera.picam2.camera_properties.get('Model'),
                        'configuration': repr(self.camera.picam2.camera_configuration()),
                        'orientation': 'pinkylib.get_frame:rotate180',
                        'native_calibration': hasattr(self.camera, 'calibration_camera')}
            if action == 'new':
                self.root = Path(tempfile.mkdtemp(prefix='pinky-calibration-'))
                self.counts = {'calibration': 0, 'validation': 0}
                for purpose in self.counts:
                    (self.root / purpose).mkdir()
                return {'remote_directory': str(self.root)}
            if action == 'frame':
                frame = self.camera.get_frame()
                stamp = time.time_ns()
                png = self.encode('.png', frame)
                result = {'png': base64.b64encode(png).decode(),
                          'observed_unix_ns': stamp, 'shape': list(frame.shape)}
                purpose = params.get('purpose')
                if purpose:
                    if self.root is None or purpose not in self.counts:
                        raise ValueError('Start an experiment first')
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    found, _ = cv2.findChessboardCorners(
                        gray, (9, 6), cv2.CALIB_CB_ADAPTIVE_THRESH |
                        cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE)
                    blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                    result.update(accepted=bool(found and blur >= params['min_sharpness']),
                                  sharpness=blur, detected=bool(found))
                    if result['accepted']:
                        self.counts[purpose] += 1
                        jpeg = self.encode('.jpg', frame)
                        name = f'{self.counts[purpose]:06d}.jpg'
                        (self.root / purpose / name).write_bytes(jpeg)
                        result.update(jpeg=base64.b64encode(jpeg).decode(), filename=name)
                return result
            if action == 'fit':
                if self.root is None or self.counts['calibration'] < 15:
                    raise ValueError('학습용 사진이 부족합니다. 최소 15장이 필요합니다.')
                previous = os.getcwd()
                display = self.camera.display_jupyter
                try:
                    os.chdir(self.root)
                    self.camera.display_jupyter = lambda frame: None
                    self.camera.calibration_camera(
                        str(self.root / 'calibration'), checkerboard_size=(9, 6),
                        square_size=params['square_mm'])
                    data = (self.root / 'camera_calibration.npz').read_bytes()
                finally:
                    self.camera.display_jupyter = display
                    os.chdir(previous)
                    self.last_seen = time.monotonic()
                return {'npz': base64.b64encode(data).decode()}
            raise ValueError('Unknown camera action')
