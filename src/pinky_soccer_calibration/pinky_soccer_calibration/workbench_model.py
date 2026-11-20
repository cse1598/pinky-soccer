"""Immutable experiment snapshots shared by the desktop UI and offline checks."""

import csv
from datetime import datetime, timezone
import io
from pathlib import Path
import shutil
import time
import uuid

import cv2
import numpy as np

from . import core


class SampleBag:
    """Write exact received sample arrays; recording needs no DDS or rclpy node."""

    def __init__(self, path):
        import rosbag2_py
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions('', ''))
        for index, purpose in enumerate(('calibration', 'validation')):
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                id=index, name=f'/calibration/{purpose}/image_raw',
                type='sensor_msgs/msg/Image', serialization_format='cdr'))

    def write(self, purpose, frame, stamp):
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image
        msg = Image()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id = 'pinky_calibration_image'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.writer.write(f'/calibration/{purpose}/image_raw', serialize_message(msg), stamp)

    def close(self):
        self.writer = None


def ros_model(npz, camera_id, size):
    with np.load(io.BytesIO(npz), allow_pickle=False) as arrays:
        k = arrays['camera_matrix'].astype(float).reshape(3, 3)
        d = arrays['distortion_coefficients'].astype(float).reshape(-1)
    return {'image_width': size[0], 'image_height': size[1], 'camera_name': camera_id,
            'distortion_model': 'plumb_bob',
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': k.ravel().tolist()},
            'distortion_coefficients': {'rows': 1, 'cols': len(d), 'data': d.tolist()},
            'rectification_matrix': {'rows': 3, 'cols': 3,
                                     'data': np.eye(3).ravel().tolist()},
            'projection_matrix': {'rows': 3, 'cols': 4,
                                  'data': np.c_[k, np.zeros(3)].ravel().tolist()}}


class Experiment:
    def __init__(self, remote, output, square_mm, camera_id, size, notes='', bag_factory=SampleBag):
        self.remote = remote
        self.square_mm = core.positive(square_mm)
        self.camera_id = camera_id.strip()
        if not self.camera_id:
            raise ValueError('카메라 식별명을 입력하세요.')
        self.size = tuple(size)
        self.root = Path(output).resolve() / (
            datetime.now().strftime('jupyter_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6])
        self.root.mkdir(parents=True)
        self.samples = {'calibration': [], 'validation': []}
        self.descriptors = {'calibration': [], 'validation': []}
        self.train_report = self.valid_report = self.camera_path = None
        self.train_path = self.valid_path = None
        self.exported = False
        self.revision = self.validation_revision = 0
        self.bag = self.csv_file = None
        self.settings = remote.info['orientation'] + ';' + remote.info['configuration']
        self.meta = {'schema_version': 1, 'calibration_backend': 'pinkylib',
                     'camera_id': self.camera_id, 'camera_settings': self.settings,
                     'square_mm': self.square_mm, 'board_inner_corners': [9, 6],
                     'image_size': list(self.size), 'notes': notes,
                     'started_utc': datetime.now(timezone.utc).isoformat(),
                     'sensor_model': remote.info['model'],
                     'image_orientation': remote.info['orientation'],
                     'timestamp_source': 'robot wall clock immediately after get_frame, not sensor time',
                     'raw_bag_scope': 'accepted captured samples only; preview is not recorded'}
        try:
            self.bag = bag_factory(self.root / 'raw_bag')
            self.remote.call('new')
            for purpose in self.samples:
                (self.root / purpose).mkdir()
            core.save(self.root / 'experiment.yaml', self.meta)
            self.csv_file = (self.root / 'samples.csv').open('x', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['purpose', 'index', 'robot_observed_unix_ns', 'pc_received_unix_ns',
                               'sharpness', 'raw_png', 'native_jpeg', 'raw_png_sha256'])
        except Exception:
            self.close()
            raise

    def capture(self, purpose, sharpness=30.):
        if self.exported or self.bag is None or purpose not in self.samples:
            raise ValueError('종료된 실험이거나 잘못된 촬영 단계입니다.')
        if purpose == 'validation' and not (self.train_report or {}).get('passed'):
            raise ValueError('학습 보정을 통과한 후 별도로 검증 사진을 촬영하세요.')
        if purpose == 'calibration' and (self.train_report or {}).get('passed'):
            raise ValueError('학습 모델은 고정되었습니다. 다시 학습하려면 새 실험을 시작하세요.')
        if len(self.samples[purpose]) >= 80:
            raise ValueError('단계별 최대 80장입니다. 새 실험을 시작하세요.')
        sample = self.remote.frame(purpose=purpose, min_sharpness=float(sharpness))
        if not sample['accepted']:
            raise ValueError('9×6 코너가 모두 보이고 흔들림이 멈춘 후 다시 촬영하세요.')
        if sample['image'].shape[:2][::-1] != self.size:
            raise ValueError('카메라 해상도가 바뀌었습니다. 새 실험을 시작하세요.')
        index = len(self.samples[purpose]) + 1
        png = self.root / purpose / f'{index:06d}.png'
        jpg = png.with_suffix('.jpg')
        png.write_bytes(sample['png_bytes'])
        jpg.write_bytes(sample['jpeg_bytes'])
        stamp = int(sample['observed_unix_ns'])
        self.bag.write(purpose, sample['image'], stamp)
        self.csv.writerow([purpose, index, stamp, time.time_ns(), sample['sharpness'],
                           str(png.relative_to(self.root)), str(jpg.relative_to(self.root)),
                           core.digest(png)])
        self.csv_file.flush()
        self.samples[purpose].append(png)
        corners = core.detect(cv2.cvtColor(sample['image'], cv2.COLOR_BGR2GRAY))
        if corners is not None:
            center = corners.reshape(-1, 2).mean(axis=0) / self.size
            area = cv2.contourArea(cv2.convexHull(corners)) / np.prod(self.size)
            self.descriptors[purpose].append([*map(float, center), float(area)])
        if purpose == 'validation':
            self.valid_report = self.valid_path = None
        return sample

    def coverage(self, purpose):
        entries = self.descriptors[purpose]
        if not entries:
            return '촬영 범위: 아직 사진이 없습니다.'
        points = np.asarray(entries)
        span = np.ptp(points[:, :2], axis=0) * 100
        ratio = points[:, 2].max() / points[:, 2].min()
        return f'중심 범위 X {span[0]:.0f}% · Y {span[1]:.0f}%\n보드 면적 최대/최소 {ratio:.1f}배\n목표: 각각 25% 이상 · 1.5배 이상'

    def _snapshot(self, path, purpose):
        path.mkdir(parents=True)
        sources = self.samples[purpose]
        core.save(path / 'session.yaml', dict(self.meta, purpose=purpose,
                  source_image_sha256=[core.digest(p) for p in sources],
                  source_experiment=str(self.root)))
        core.save(path / 'acquisition.yaml', {'completed': True, 'image_size': list(self.size),
                                              'frame_id': 'pinky_calibration_image',
                                              'sampled_images': len(sources)})
        folder = path / ('native' if purpose == 'calibration' else 'frames')
        folder.mkdir()
        for source in sources:
            if purpose == 'calibration':
                # Score the exact JPEG input used by the native calibration routine.
                image = cv2.imread(str(source.with_suffix('.jpg')))
                if image is None or not cv2.imwrite(str(folder / source.name), image):
                    raise OSError('내장 보정 입력 영상을 저장하지 못했습니다.')
            else:
                shutil.copyfile(source, folder / source.name)
        return folder

    def fit(self, max_rms=.7):
        if len(self.samples['calibration']) < 15:
            raise ValueError('학습 사진이 최소 15장 필요합니다. 25장 이상을 권장합니다.')
        if (self.train_report or {}).get('passed'):
            raise ValueError('모델이 고정되었습니다. 새 실험에서 다시 학습하세요.')
        npz = self.remote.fit(self.square_mm)
        self.revision += 1
        path = self.root / f'attempt_{self.revision:03d}' / 'training'
        folder = self._snapshot(path, 'calibration')
        (folder / 'camera_calibration.npz').write_bytes(npz)
        self.camera_path = folder / 'camera.yaml'
        core.save(self.camera_path, ros_model(npz, self.camera_id, self.size))
        self.train_report = core.check(path, self.camera_path, max_rms=max_rms)
        self.train_path = path / 'check/report.yaml'
        return self.train_report

    def validate(self, max_rms=.7):
        if not (self.train_report or {}).get('passed'):
            raise ValueError('학습 보정부터 통과해야 합니다.')
        if len(self.samples['validation']) < 8:
            raise ValueError('새 자세의 검증 사진이 최소 8장 필요합니다.')
        self.validation_revision += 1
        path = self.train_path.parents[2] / f'validation_{self.validation_revision:03d}'
        self._snapshot(path, 'validation')
        self.valid_report = core.check(path, self.camera_path, max_rms=max_rms)
        self.valid_path = path / 'check/report.yaml'
        return self.valid_report

    def export(self, visual_check):
        if self.train_path is None or self.valid_path is None:
            raise ValueError('학습·독립 검증을 모두 완료하세요.')
        output = core.export(self.train_path, self.valid_path, visual_check)
        # Close the bag before declaring an export complete.
        self.close()
        self.exported = True
        return output

    def close(self):
        if self.bag is not None:
            self.bag.close()
            self.bag = None
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
