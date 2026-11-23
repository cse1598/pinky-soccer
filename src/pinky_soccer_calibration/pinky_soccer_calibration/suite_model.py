"""Auditable calibration calculations, independent of the UI and ROS graph."""

from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import uuid

import cv2
import numpy as np
import yaml

from .core import stats
from .board_geometry import Board, floor_pose, registered_pose
from .project_store import ProjectStore, utc


def override_mount_height(summary, override):
    """Keep measured tilt and raw height statistics when selecting a model height."""
    if not override:
        return summary
    height = float(override['height_m'])
    if not np.isfinite(height) or not .01 < height < 3:
        raise ValueError('카메라 높이는 0.01~3m 사이의 유한한 값이어야 합니다.')
    result = deepcopy(summary)
    result['measured_height_m'] = deepcopy(summary.get('measured_height_m', summary['height_m']))
    result['height_m'] = {'mean': height, 'source': 'user_selected_urdf_height'}
    result['height_override'] = deepcopy(override)
    return result


GUIDES = {
    'camera': ('카메라 내부', '보드 내부 코너 수·칸 실측 길이 입력(기본9×6·30mm). '
               '카메라 보정 탭에서 촬영/파일 추가 후 학습15장 이상·검증8장 이상 분류. '
               '독립 검증 계산은 현재 학습 사진으로 매번 새로 보정하고 검증 사진으로 평가합니다. '
               '사진 추가·이동·삭제 후 다시 계산하고 왜곡 제거 영상의 직선도 확인하세요.'),
    'wheels': ('바퀴·오도메트리', '줄자·바닥 테이프·각도 기준·펜 준비. 장애물 없는 바닥에서 '
               '전후진: 범퍼 앞면 중앙의 시작~끝 직선 거리(m)만 입력(전진+/후진-). '
               '회전: 실제 누적 각도(deg)만 별도 저장(좌+/우-). 좌우 편향으로 각도를 역산하지 않습니다. '
               '공통 유효 바퀴 반경·간격 추정이며 좌우 반경 차이는 식별하지 않습니다. '
               '각 방향 3회 이상 권장. S자·큰 곡선·미끄러진 궤적은 재실험.'),
    'lidar': ('LiDAR', '넓고 수직인 벽과 범퍼 앞면을 평행하게 놓으세요. 범퍼 전면 바깥쪽 '
              '중앙에서 벽까지 로봇 전방으로 잰 거리(m)를 입력합니다. 센서 위치는 실행 모델에서 '
              '자동 변환합니다. 로봇 정면 ±0.035rad 레이를 사용하며 여러 거리에서 3회 이상 반복. '
              '방향 확인. 단일 벽 거리만으로 XYZ/RPY 전체가 자동 결정되지는 않습니다.'),
    'ultrasonic': ('초음파', '범퍼 앞면과 평행하게 넓은 수직 판을 배치. 센서 높이까지 판이 '
                   '닿도록 하세요. 범퍼 전면 바깥쪽 중앙~판 거리(m)를 입력하면 센서 기준으로 자동 변환. '
                   '여러 거리에서 정지 수집. 감지 실패 비율과 평균/표준편차 확인. 좁은 물체와 '
                   '비스듬한 표면도 별도 확인하세요.'),
    'imu': ('IMU 정지·회전', '로봇을 수평 바닥에 놓고 완전히 정지해 10초 이상 수집. '
            'gyro 바이어스와 각 축 노이즈를 계산. 가속도에는 중력이 포함되므로 '
            '한 자세만으로 3축 가속도 바이어스를 분리할 수 없습니다. '
            '회전은 각도 기준으로 실측하고 적분 회전각과 비교하세요.'),
    'timing': ('주기·수신 지연', '모든 센서를 켜고 수집. 토픽별 수신 간격·지터·표본 수 확인. '
               '실물 PC/로봇 시계를 동기화해야 header→수신 지연이 의미 있습니다. '
               '현재 카메라 stamp는 읽기 완료 시각이므로 노출부터의 전체 지연은 아닙니다. '
               '전체 지연은 외부 시계/LED·고속 촬영 등 별도 기준이 필요합니다.'),
    'extrinsics': ('바닥 높이·기울기', '내부 보정/독립 검증 후 보드를 바닥에 평평하게 놓고 '
                   '로봇을 수평·정지시키세요. 촬영하면 바닥에서 카메라 광학 중심까지 높이와 '
                   '좌우 roll/상하 pitch를 추정합니다. 보드 위치·0번·축 방향 등록은 필요 없습니다. '
                   'pitch +는 아래쪽 기울기입니다. 보드 위치를 바꿔 5회 이상 반복 권장. '
                   '보정 높이/각도는 공 검토에 우선 적용하며 프로필로 launch에도 적용합니다. '
                   '측정하지 않은 X/Y/yaw는 기존 모델/TF 값을 유지합니다.'),
}


def finite(value):
    value = float(value)
    if not np.isfinite(value):
        raise ValueError('유한한 실측 숫자를 입력하세요.')
    return value


def wheel_fit(trials):
    if any('measurement_type' in row for row in trials):
        from .wheel_measurement import distance_rotation_fit
        return distance_rotation_fit(trials)
    rows, targets = [], []
    for t in trials:
        left, right, angle = map(finite, (t['left_rad'], t['right_rad'], t['angle_rad']))
        if t.get('measurement_type') != 'rotation_angle':
            rows.append([left, right, 0])
            targets.append(2 * finite(t['distance_m']))
        rows.append([-left, right, -angle])
        targets.append(0)
    matrix = np.asarray(rows)
    kinds = {t.get('kind') for t in trials}
    if (len(trials) < 4 or np.linalg.matrix_rank(matrix) < 3
            or (any('measurement_type' in t for t in trials)
                and not {'forward', 'backward', 'left', 'right'} <= kinds)):
        raise ValueError('전후진·좌우 회전을 포함한 독립 실험이 최소 4회 필요합니다.')
    values, _, _, _ = np.linalg.lstsq(matrix, targets, rcond=None)
    if not (0.005 < values[0] < .1 and .005 < values[1] < .1 and .03 < values[2] < .5):
        raise ValueError('바퀴 추정값이 범위를 벗어납니다. 단위·부호·실측값을 확인하세요.')
    error = matrix @ values - targets
    repeats = {}
    for kind in ('forward', 'backward', 'left', 'right'):
        group = [t for t in trials if t.get('kind') == kind]
        if group:
            keys = ('longitudinal_m', 'lateral_m') if kind in ('forward', 'backward') else ('angle_rad',)
            repeats[kind] = {key: stats([t[key] for t in group if key in t])
                             for key in keys if any(key in t for t in group)}
    return {'left_radius_m': float(values[0]), 'right_radius_m': float(values[1]),
            'separation_m': float(values[2]), 'equation_residual': stats(np.abs(error)),
            'condition_number': float(np.linalg.cond(matrix)), 'trials': len(trials),
            'repeat_measurements': repeats,
            'straight_model': 'constant_curvature; no lateral slip; aligned initial heading'}


def scalar_report(values, reference):
    values = np.asarray(values, dtype=float)
    valid = values[np.isfinite(values)]
    if len(valid) < 10:
        raise ValueError('유효 표본 10개 이상 필요합니다. 토픽·배치·감지 범위를 확인하세요.')
    result = stats(valid)
    result.update(reference=finite(reference), bias=float(valid.mean() - reference),
                  invalid_fraction=float(1 - len(valid) / len(values)))
    return result


class Session(ProjectStore):
    def __init__(self, parent, environment, robot_id):
        if environment not in ('real', 'rehearsal') or not robot_id.strip():
            raise ValueError('환경과 로봇 식별명을 입력하세요.')
        self.root = Path(parent).expanduser().resolve() / (
            datetime.now().strftime('ros_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6])
        self.root.mkdir(parents=True)
        self.meta = {'schema_version': 1, 'environment': environment, 'robot_id': robot_id,
                     'started_utc': datetime.now(timezone.utc).isoformat()}
        self.results, self.trials = {}, []
        self.frames = {'training': [], 'validation': []}
        self.camera = None
        self.board = Board()
        self.floor_board = Board()
        self.init_project()
        self.save()

    def write(self, name, value):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_suffix('.tmp')
        temp.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False))
        temp.replace(path)
        return path

    def save_raw_capture(self, frame, metadata):
        """Keep every service shot, including shots later rejected by calibration checks."""
        folder = self.root / 'captures'
        folder.mkdir(exist_ok=True)
        stem = uuid.uuid4().hex
        path = folder / (stem+'.png')
        if not cv2.imwrite(str(path), frame):
            raise OSError('서비스 원본 사진 저장 실패')
        metadata = dict(metadata, raw_image_file=str(path.relative_to(self.root)))
        self.write('captures/'+stem+'.capture.yaml', metadata)
        return metadata

    def capture(self, frame, purpose, provenance=None):
        if purpose not in self.frames:
            raise ValueError('학습/검증 사진 분류를 선택하세요.')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = self.board.detect(gray)
        if corners is None:
            raise ValueError(f'{self.board.pattern} 내부 코너가 전부 보이게 보드를 배치하세요.')
        if cv2.Laplacian(gray, cv2.CV_64F).var() < 30:
            raise ValueError('영상이 흐립니다. 보드를 멈추고 초점을 확인하세요.')
        folder = self.root / purpose
        folder.mkdir(exist_ok=True)
        path = folder / (uuid.uuid4().hex+'.png')
        if not cv2.imwrite(str(path), frame):
            raise OSError('사진 저장 실패')
        if provenance is not None:
            self.write(str(path.relative_to(self.root).with_suffix('.capture.yaml')), provenance)
        self.frames[purpose].append(path)
        self.invalidate('camera')
        self.save()
        return len(self.frames[purpose])

    def camera_fit(self, square_mm):
        self.invalidate('camera')
        self.save()
        square = finite(square_mm)
        if square <= 0:
            raise ValueError('칸 길이는 양수 mm입니다.')
        paths = self.frames['training']
        if len(paths) < 15:
            raise ValueError('학습 사진 15장 이상 필요합니다.')
        images = [cv2.imread(str(p), 0) for p in paths]
        if len({im.shape for im in images}) != 1:
            raise ValueError('해상도가 바뀌었습니다. 새 세션이 필요합니다.')
        corners = [self.board.detect(im) for im in images]
        if square != self.board.square_mm or any(c is None for c in corners):
            raise ValueError('촬영한 보드 설정이 바뀌었습니다. 새 세션을 만드세요.')
        rms, k, d, _, _ = cv2.calibrateCamera(
            [self.board.points()] * len(corners), corners, images[0].shape[::-1], None, None)
        if not np.isfinite(rms) or rms > .7:
            raise ValueError(f'학습 RMS {rms:.3f}px: 위치·거리·기울기를 다양하게 재촬영하세요.')
        self.camera = (k, d, images[0].shape[::-1], square)
        self.meta.update(square_mm=square, board_inner_corners=list(self.board.pattern),
                         image_size=list(images[0].shape[::-1]))
        self.write('session.yaml', self.meta)
        self.save()
        return {'training_rms_px': float(rms), 'next': '새 자세로 검증 사진 8장 이상 촬영'}

    def camera_validate(self):
        from .core import score_view
        if not self.camera or len(self.frames['validation']) < 8:
            raise ValueError('학습 후 독립 검증 사진 8장 이상 필요합니다.')
        k, d, size, square = self.camera
        import hashlib
        hashes = [{hashlib.sha256(cv2.imread(str(p), 0).tobytes()).hexdigest()
                   for p in self.frames[key]} for key in ('training', 'validation')]
        if hashes[0] & hashes[1]:
            self.invalidate('camera')
            self.save()
            raise ValueError('학습과 검증에 동일한 사진이 있습니다. 중복 사진을 제외하세요.')
        errors = []
        centers = []
        for path in self.frames['validation']:
            image = cv2.imread(str(path), 0)
            if image.shape[::-1] != size:
                raise ValueError('검증 사진 해상도가 다릅니다.')
            corners = self.board.detect(image)
            if corners is None:
                raise ValueError('검증 보드를 검출하지 못했습니다.')
            residual, _ = score_view(self.board.points(), corners, k, d)
            errors.append(float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))))
            centers.append(corners.reshape(-1, 2).mean(axis=0) / size)
        report = stats(errors)
        report['passed'] = bool(max(errors) <= .7 and min(np.ptp(centers, axis=0)) >= .25)
        self.results['camera'] = report
        self.save()
        if not report['passed']:
            raise ValueError('검증 실패: RMS 0.7px 이하·상하좌우 중심 범위 각 25% 이상 필요')
        model = {'image_width': size[0], 'image_height': size[1], 'camera_name': self.meta['robot_id'],
                 'distortion_model': 'plumb_bob',
                 'camera_matrix': {'rows': 3, 'cols': 3, 'data': k.ravel().tolist()},
                 'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': d.ravel().tolist()},
                 'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
                 'projection_matrix': {'rows': 3, 'cols': 4,
                                       'data': np.c_[k, np.zeros(3)].ravel().tolist()}}
        self.write('camera.yaml', model)
        self.stale.discard('camera')
        self.save()
        return report

    def configure_board(self, board):
        if board != self.board and (any(self.frames.values()) or self.camera or self.measurements.get('floor')):
            raise ValueError('촬영 후 보드를 변경할 수 없습니다. 새 세션을 만드세요.')
        self.board = board
        self.meta.update(board_inner_corners=list(board.pattern), square_mm=board.square_mm)
        self.save()

    def floor_capture(self, frame, registration=None, provenance=None):
        if not self.results.get('camera', {}).get('passed'):
            raise ValueError('내부 보정과 독립 검증을 먼저 통과하세요.')
        k, d, size, _ = self.camera
        if frame.shape[:2][::-1] != size:
            raise ValueError('내부 보정과 영상 해상도가 다릅니다.')
        board = self.floor_board
        board_spec = [board.columns, board.rows, board.square_mm]
        corners = board.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if corners is None:
            raise ValueError(f'장착용 {board.pattern} 바닥 보드 전체 내부 코너가 보여야 합니다.')
        result = floor_pose(board.points(), corners, k, d, allow_normal_flip=registration is None)
        if registration is not None:
            result['base_pose'] = registered_pose(result, *registration)
            result['registration'] = list(registration)
        else:
            result = {'height_m': result['height_m'],
                      'roll_rad': result['mechanical_rpy_rad'][0],
                      'pitch_rad': result['mechanical_rpy_rad'][1],
                      'reprojection_rms_px': result['reprojection_rms_px'],
                      'measurement_mode': 'floor_height_tilt', 'reference': 'floor_plane',
                      'unmeasured': ['base_x', 'base_y', 'base_yaw']}
        result['board'] = board_spec
        entries = self.results.setdefault('floor_camera', [])
        folder = self.root / 'floor'
        folder.mkdir(exist_ok=True)
        path = folder / (uuid.uuid4().hex+'.png')
        if not cv2.imwrite(str(path), frame):
            raise OSError('바닥 사진 저장 실패')
        if provenance is not None:
            sidecar = str(path.relative_to(self.root).with_suffix('.capture.yaml'))
            self.write(sidecar, provenance)
            result['capture_metadata_file'] = sidecar
        entries.append(result)
        result['image_file'] = self.relative(path)
        # Full extrinsic application requires repeated registered estimates.
        if len(entries) >= 5 and all('base_pose' in e for e in entries):
            poses = np.asarray([e['base_pose'] for e in entries])
            if (np.max(np.std(poses[:, :3], axis=0, ddof=1)) < .005
                    and np.max(np.std(np.unwrap(poses[:, 3:], axis=0), axis=0, ddof=1)) < .035):
                self.results['extrinsics'] = np.r_[poses[:, :3].mean(axis=0),
                    np.arctan2(np.sin(poses[:, 3:]).mean(axis=0),
                               np.cos(poses[:, 3:]).mean(axis=0))].tolist()
            else:
                self.results.pop('extrinsics', None)
        else:
            self.results.pop('extrinsics', None)
        if registration is None:
            measured = [e for e in entries if e.get('measurement_mode') == 'floor_height_tilt']
            summary = {key: stats([e[key] for e in measured])
                       for key in ('height_m', 'roll_rad', 'pitch_rad')}
            summary.update(reference='floor_plane', unmeasured=['base_x', 'base_y', 'base_yaw'],
                           applicable=False, repeats=len(measured))
            summary['repeatability_passed'] = (len(measured) >= 5
                and summary['height_m']['sample_std'] < .005
                and max(summary[k]['sample_std'] for k in ('roll_rad', 'pitch_rad')) < .035)
            self.results['camera_mount'] = summary
        if registration is None:
            self.measurements.setdefault('floor', []).append(
                {'id': uuid.uuid4().hex, 'recorded_utc': utc(), 'image_file': self.relative(path),
                 'board': board_spec,
                 'capture_metadata_file': result.get('capture_metadata_file')})
            self.revision += 1
            self.recalculate('floor')
            entries = self.results['floor_camera']
        self.save()
        return dict(result, repeats=len(entries),
                    height_statistics=stats([e['height_m'] for e in entries]),
                    applicable='extrinsics' in self.results)

    def configure_floor_board(self, board):
        """Change the next mounting board without reinterpreting old photos or intrinsics."""
        self.floor_board = board
        self.meta['floor_board'] = [board.columns, board.rows, board.square_mm]
        self.save()

    def export(self, confirmed):
        if self.meta.get('image_source', {}).get('kind') == 'webcam' and self.meta['environment'] != 'rehearsal':
            raise ValueError('웹캠 보정은 rehearsal 프로필로만 내보낼 수 있습니다.')
        if not confirmed or not self.results:
            raise ValueError('분석 결과와 사람이 수행한 독립 확인이 필요합니다.')
        export_results = {k: v for k, v in self.results.items() if k not in self.stale and k != 'ball_review'
                          and not (k == 'camera' and not v.get('passed'))}
        if not any(k != 'time_sync' for k in export_results):
            raise ValueError('현재 기록으로 계산·검증한 유효 결과가 없습니다. 먼저 다시 계산하세요.')
        self.save()
        snapshot = self.root/'exports'/uuid.uuid4().hex
        snapshot.mkdir(parents=True)
        profile = dict(self.meta, results=export_results, launch={}, camera_pose=None,
                       project_revision=self.revision, exported_utc=utc(),
                       pending_recalculation=sorted(self.stale))
        if export_results.get('camera', {}).get('passed'):
            import shutil
            if not (self.root/'camera.yaml').is_file():
                raise ValueError('카메라 보정 파일이 없습니다. 현재 사진으로 다시 계산하세요.')
            shutil.copyfile(self.root/'camera.yaml', snapshot/'camera.yaml')
            profile['launch']['camera_info_url'] = (snapshot/'camera.yaml').as_uri()
        if 'wheels' in export_results:
            w = export_results['wheels']
            # Official motor driver supports one effective radius, not separate wheel radii.
            profile['launch'].update(wheel_radius=(w['left_radius_m'] + w['right_radius_m']) / 2,
                                     wheel_separation=w['separation_m'])
        if 'extrinsics' in export_results:
            profile['camera_pose'] = export_results['extrinsics']
        if 'camera_mount' in export_results and 'floor' not in self.stale:
            profile['camera_mount'] = export_results['camera_mount']
        profile['human_verified'] = True
        self.write('results.yaml', self.results)
        self.write('wheel_trials.yaml', self.trials)
        self.write(str((snapshot/'profile.yaml').relative_to(self.root)), profile)
        return self.write('profile.yaml', profile)
