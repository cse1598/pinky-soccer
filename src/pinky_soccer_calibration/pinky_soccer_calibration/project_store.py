"""Persistent editable calibration projects; raw acquisition files remain immutable."""

from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import hashlib
from pathlib import Path
import uuid

import cv2
import numpy as np
import yaml

from .board_geometry import Board, floor_pose
from .core import stats


def utc():
    return datetime.now(timezone.utc).isoformat()


# Control/communication measurements (2026-11-25). They are evidence for MPC work, not part of
# the launch calibration: adding/removing them never archives the exported profile.yaml and
# they are exported separately (identification.yaml / control_results.yaml), not in profile.
CONTROL_CATEGORIES = ('clock_sync', 'communication_latency', 'command_latency',
                      'motor_response', 'braking', 'footprint')


class ProjectStore:
    def init_project(self):
        self.measurements = {}
        self.stale = set()
        self.revision = 0
        self.deleted = []
        self._project_lock = (self.root/'.project.lock').open('a')
        try:
            fcntl.flock(self._project_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._project_lock.close()
            raise ValueError('다른 UI가 이 프로젝트를 사용 중입니다. 먼저 닫으세요.')

    def release(self):
        lock = getattr(self, '_project_lock', None)
        if lock and not lock.closed:
            fcntl.flock(lock, fcntl.LOCK_UN)
            lock.close()

    def relative(self, path):
        return str(Path(path).resolve().relative_to(self.root))

    def local_path(self, relative):
        path = (self.root/relative).resolve()
        path.relative_to(self.root)
        return path

    def save(self):
        camera = None
        if self.camera:
            k, d, size, square = self.camera
            camera = {'k': k.tolist(), 'd': d.tolist(), 'size': list(size), 'square': square}
        state = {'project_schema': 1, 'meta': self.meta, 'revision': self.revision,
                 'board': [self.board.columns, self.board.rows, self.board.square_mm],
                 'floor_board': [self.floor_board.columns, self.floor_board.rows,
                                 self.floor_board.square_mm],
                 'frames': {key: [self.relative(p) for p in paths]
                            for key, paths in self.frames.items()},
                 'camera': camera, 'trials': self.trials, 'measurements': self.measurements,
                 'results': self.results, 'stale': sorted(self.stale), 'deleted': self.deleted}
        state['photo_hashes'] = {self.relative(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sum(self.frames.values(), []) if p.is_file()}
        self.write('session.yaml', self.meta)
        self.write('results.yaml', self.results)
        self.write('wheel_trials.yaml', self.trials)
        for kind, types in [('straight', {'straight_displacement', 'straight_distance'}),
                            ('rotation', {'rotation_angle'})]:
            self.write('wheel_'+kind+'_trials.yaml',
                       [v for v in self.trials if v.get('measurement_type') in types])
        return self.write('project.yaml', state)  # Authoritative state, atomically replaced last.

    @classmethod
    def open(cls, folder):
        root = Path(folder).expanduser().resolve()
        def read(name, default):
            path = root/name
            return yaml.safe_load(path.read_text()) if path.is_file() else default
        state = read('project.yaml', None)
        meta = state['meta'] if state else read('session.yaml', None)
        if not isinstance(meta, dict) or meta.get('environment') not in ('real', 'rehearsal'):
            raise ValueError('보정 프로젝트 폴더(project.yaml 또는 ROS UI session.yaml)를 선택하세요.')
        if not meta.get('robot_id'):
            raise ValueError('로봇 식별명이 없는 폴더입니다.')
        if state and state.get('project_schema') != 1:
            raise ValueError('지원하지 않는 프로젝트 형식입니다.')
        obj = cls.__new__(cls)
        obj.root, obj.meta = root, meta
        obj.init_project()
        try:
            shape = state['board'] if state else [*meta.get('board_inner_corners', [9, 6]),
                                                 meta.get('square_mm', 30)]
            obj.board = Board(*shape)
            obj.floor_board = Board(*(state.get('floor_board', shape) if state else
                                      meta.get('floor_board', shape)))
            obj.frames = {key: ([obj.local_path(p) for p in state['frames'][key]] if state else
                                sorted((root/key).glob('*.png')))
                          for key in ('training', 'validation')}
            for path in sum(obj.frames.values(), []):
                obj.relative(path)
                if not path.is_file():
                    raise ValueError(f'사진 파일이 없습니다: {path.name}')
            if len(set(sum(obj.frames.values(), []))) != sum(map(len, obj.frames.values())):
                raise ValueError('학습/검증 사진 목록이 중복되어 있습니다.')
            obj.results = state['results'] if state else (read('results.yaml', {}) or {})
            obj.trials = state['trials'] if state else (read('wheel_trials.yaml', []) or [])
            obj.camera = None
            if state:
                obj.measurements = state.get('measurements', {})
                obj.revision, obj.stale = state['revision'], set(state['stale'])
                obj.deleted = state.get('deleted', [])
                if state.get('camera'):
                    c = state['camera']
                    obj.camera = (np.array(c['k']), np.array(c['d']).reshape(-1), tuple(c['size']), c['square'])
                    if (obj.camera[0].shape != (3, 3) or obj.camera[1].shape != (5,)
                            or not np.isfinite(obj.camera[0]).all() or not np.isfinite(obj.camera[1]).all()
                            or len(obj.camera[2]) != 2 or min(obj.camera[2]) <= 0):
                        raise ValueError('저장된 카메라 모델이 올바르지 않습니다.')
            else:
                # Legacy report-only rows are explicitly marked: do not invent lost raw samples.
                for key in ('lidar', 'ultrasonic'):
                    obj.measurements[key] = [{'id': uuid.uuid4().hex, 'legacy_report': v}
                                             for v in obj.results.get(key, [])]
                for key in ('imu_stationary', 'timing'):
                    if key in obj.results:
                        obj.measurements[key] = [{'id': uuid.uuid4().hex,
                                                 'legacy_report': obj.results[key]}]
                for i, row in enumerate(obj.results.get('floor_camera', [])):
                    path = obj.local_path(row['image_file']) if 'image_file' in row else root/'floor'/f'{i:04d}.png'
                    obj.measurements.setdefault('floor', []).append(
                        {'id': uuid.uuid4().hex, 'image_file': obj.relative(path),
                         'board': row.get('board', list(shape)),
                         'legacy_report': row})
                if obj.results.get('camera', {}).get('passed') and (root/'camera.yaml').exists():
                    from .core import camera_model
                    _, k, d, size = camera_model(root/'camera.yaml')
                    obj.camera = (k, d, size, obj.board.square_mm)
            for row in obj.trials:
                row.setdefault('id', uuid.uuid4().hex)
            # Before board separation every floor photo used the intrinsic board.
            # Freeze that historical geometry before the user changes the next board.
            for row in obj.measurements.get('floor', []):
                row.setdefault('board', list(shape))
            if state and any(hashlib.sha256(obj.local_path(p).read_bytes()).hexdigest() != value
                             for p, value in state.get('photo_hashes', {}).items()):
                obj.invalidate('camera')
            if (obj.results.get('wheels') and any('measurement_type' in row for row in obj.trials)
                    and obj.results['wheels'].get('radius_model') != 'shared_effective_radius'):
                obj.invalidate('wheels')
            obj.save()
            return obj
        except Exception:
            obj.release()
            raise

    def invalidate(self, category):
        self.revision += 1
        self.stale.add(category)
        keys = {'camera': ('camera', 'floor_camera', 'camera_mount', 'extrinsics'),
                'floor': ('floor_camera', 'camera_mount', 'extrinsics')}.get(category, (category,))
        for key in keys:
            self.results.pop(key, None)
        if category == 'camera':
            self.camera = None
            if self.measurements.get('floor'):
                self.stale.add('floor')
        if category == 'ball_review' or category in CONTROL_CATEGORIES:
            return  # Validation/control evidence never changes the runtime calibration profile.
        # Previously exported profiles remain immutable, but no stale root profile is advertised.
        profile = self.root/'profile.yaml'
        if profile.exists():
            archive = self.root/'exports'
            archive.mkdir(exist_ok=True)
            profile.replace(archive/('previous_'+uuid.uuid4().hex+'.yaml'))

    def assign_photos(self, paths, purpose=None):
        if purpose not in (None, 'training', 'validation'):
            raise ValueError('학습/검증 분류를 선택하세요.')
        selected = {self.local_path(p) for p in paths}
        active = set(sum(self.frames.values(), []))
        if not selected or not selected <= active:
            raise ValueError('현재 프로젝트 사진을 선택하세요.')
        for key in self.frames:
            self.frames[key] = [p for p in self.frames[key] if p not in selected]
        if purpose:
            self.frames[purpose].extend(sorted(selected))
        else:
            self.deleted.extend({'type': 'photo', 'path': self.relative(p), 'deleted_utc': utc()}
                                for p in selected)
        self.invalidate('camera')
        self.save()

    def import_photos(self, paths):
        known = {hashlib.sha256(cv2.imread(str(p)).tobytes()).hexdigest()
                 for p in sum(self.frames.values(), []) if cv2.imread(str(p)) is not None}
        added, skipped = 0, 0
        for source in paths:
            path = Path(source)
            image = cv2.imread(str(path))
            if image is None:
                raise ValueError(f'이미지를 읽을 수 없습니다: {path.name}')
            # Persist each successful import immediately, even if a later source is invalid.
            raw_hash = hashlib.sha256(path.read_bytes()).hexdigest()
            pixel_hash = hashlib.sha256(image.tobytes()).hexdigest()
            if pixel_hash in known:
                skipped += 1
                continue
            provenance = {'source': 'file_import', 'original_name': path.name,
                          'imported_utc': utc(), 'source_sha256': raw_hash,
                          'original_ros_capture_verified': False}
            try:
                self.capture(image, 'training', provenance)
            except ValueError as error:
                raise ValueError(f'{path.name}: {error}') from error
            known.add(pixel_hash)
            added += 1
        return {'imported': added, 'duplicates_skipped': skipped}

    def add_measurement(self, category, payload):
        row = deepcopy(payload)
        row.update(id=uuid.uuid4().hex, recorded_utc=utc())
        self.measurements.setdefault(category, []).append(row)
        self.invalidate(category)
        self.save()
        return row

    def remove_measurements(self, category, identifiers):
        rows = self.trials if category == 'wheels' else self.measurements.get(category, [])
        selected = [v for v in rows if v['id'] in identifiers]
        if not selected:
            raise ValueError('삭제할 기록을 선택하세요.')
        self.deleted.extend({'type': category, 'record': v, 'deleted_utc': utc()} for v in selected)
        rows[:] = [v for v in rows if v['id'] not in identifiers]
        self.invalidate(category)
        self.save()

    def recalculate(self, category):
        from .suite_model import override_mount_height, scalar_report, wheel_fit
        if category == 'camera':
            self.camera_fit(self.board.square_mm)
            return self.camera_validate()
        self.invalidate(category)
        self.save()
        rows = self.trials if category == 'wheels' else self.measurements.get(category, [])
        if not rows:
            self.save()
            raise ValueError('계산할 기록이 없습니다.')
        try:
            if category == 'wheels':
                result = wheel_fit(rows)
            elif category == 'ball_review':
                result = {'error_m': stats([v['error_m'] for v in rows]),
                          'corrected_error_m': stats([v['corrected_error_m'] for v in rows]),
                          'purpose': 'review_only_not_applied_to_gameplay'}
            elif category in ('lidar', 'ultrasonic'):
                result = []
                for row in rows:
                    if 'samples' not in row:
                        result.append(row['legacy_report'])
                    else:
                        report = scalar_report([v['distance'] for v in row['samples']],
                                               row['reference']['sensor_reference_m'])
                        report.update(row['reference'])
                        result.append(report)
            elif category == 'imu_stationary':
                raw = [v for row in rows for v in row.get('samples', [])]
                if raw:
                    result = {key: stats([v[key] for v in raw]) for key in raw[0]
                              if key.startswith(('angular_velocity_', 'linear_acceleration_'))}
                    if (max(result['angular_velocity_'+a]['sample_std'] or 0 for a in 'xyz') > .02
                            or max(abs(result['angular_velocity_'+a]['mean']) for a in 'xyz') > .05):
                        raise ValueError('정지 gyro 변동이 큽니다. 문제 기록을 제외하고 다시 계산하세요.')
                    if any('legacy_report' in row for row in rows):
                        result['legacy_runs_not_pooled'] = True
                else:
                    result = rows[-1]['legacy_report']
            elif category == 'clock_sync':
                # One row per measurement; the history is rebuilt from the kept records.
                history = [dict(row['summary'], record_id=row['id'],
                                recorded_utc=row.get('recorded_utc')) for row in rows]
                result = {'history': history, 'latest': history[-1]}
            elif category == 'timing':
                result = []
                for row in rows:
                    if 'topics' not in row:
                        result.append(row['legacy_report'])
                        continue
                    report = {}
                    for topic, series in row['topics'].items():
                        if len(series) > 2:
                            report[topic] = {'receive_interval_s': stats(np.diff([v[0] for v in series])),
                                             'delay_status': 'header age only; excludes exposure/driver time'}
                            ages = [v[2]['header_age_s'] for v in series if
                                    row.get('clock_sync_verified') and v[2].get('clock_sync_verified')
                                    and 'header_age_s' in v[2]]
                            if ages:
                                report[topic]['header_to_receive_s'] = stats(ages)
                            else:
                                report[topic]['delay_status'] = 'not_measured: clock reference required'
                    result.append(report)
            elif category == 'floor':
                if not self.camera or not self.results.get('camera', {}).get('passed'):
                    raise ValueError('현재 사진으로 카메라 내부 보정·검증을 먼저 계산하세요.')
                entries = []
                for row in rows:
                    frame = cv2.imread(str(self.local_path(row['image_file'])))
                    if frame is None or frame.shape[:2][::-1] != self.camera[2]:
                        raise ValueError('장착 사진 파일/해상도를 확인하세요.')
                    board_spec = row.get('board', [self.board.columns, self.board.rows,
                                                   self.board.square_mm])
                    board = Board(*board_spec)
                    corners = board.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                    if corners is None:
                        raise ValueError('장착 사진의 체커보드를 검출하지 못했습니다.')
                    pose = floor_pose(board.points(), corners, *self.camera[:2], allow_normal_flip=True)
                    entries.append({'height_m': pose['height_m'], 'roll_rad': pose['mechanical_rpy_rad'][0],
                                    'pitch_rad': pose['mechanical_rpy_rad'][1],
                                    'reprojection_rms_px': pose['reprojection_rms_px'],
                                    'image_file': row['image_file'],
                                    'board': list(board_spec),
                                    'capture_metadata_file': row.get('capture_metadata_file'),
                                    'measurement_mode': 'floor_height_tilt'})
                self.results['floor_camera'] = entries
                result = {key: stats([v[key] for v in entries])
                          for key in ('height_m', 'roll_rad', 'pitch_rad')}
                result.update(repeats=len(entries), applicable=False, reference='floor_plane')
                result['repeatability_passed'] = (len(entries) >= 5
                    and result['height_m']['sample_std'] < .005
                    and max(result[k]['sample_std'] for k in ('roll_rad', 'pitch_rad')) < .035)
                result = override_mount_height(result, self.meta.get('camera_mount_height_override'))
                self.results['camera_mount'] = result
            else:
                raise ValueError('알 수 없는 보정 항목입니다.')
            if category != 'floor':
                self.results[category] = result
            self.stale.discard(category)
            self.save()
            return result
        except Exception:
            self.stale.add(category)
            self.save()
            raise
