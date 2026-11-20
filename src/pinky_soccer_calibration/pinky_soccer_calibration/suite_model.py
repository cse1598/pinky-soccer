"""Auditable calibration calculations, independent of the UI and ROS graph."""

from datetime import datetime, timezone
from pathlib import Path
import uuid

import cv2
import numpy as np
import yaml

from .core import stats
from .board_geometry import Board, floor_pose, registered_pose

GUIDES = {
    'camera': ('카메라 내부', '사용할 보드의 내부 코너 수·칸 실측 길이를 입력(기본 9×6·30mm). '
               '상하좌우·거리·기울기를 바꿔 '
               '학습 15장 이상 촬영 → 학습 → 새 자세 검증 8장 이상 → 검증. '
               '검증 시 왜곡 제거 화면에서 직선이 곧은지도 확인하세요.'),
    'wheels': ('바퀴·오도메트리', '줄자·바닥 테이프·각도 기준·펜 준비. 장애물 없는 바닥에서 '
               '전진/후진 1 m 및 좌/우 360°를 각각 3회 이상 실행. 정지 후 실제 중심 이동 거리(m), '
               '회전각(deg)을 부호 포함 입력하세요. 전진/좌회전은 +, 후진/우회전은 -. '
               '펜 궤적의 횡방향 이탈도 기록하고 보정 후 별도 실험으로 다시 확인하세요.'),
    'lidar': ('LiDAR', '평평한 벽을 센서 정면에 수직 배치. 센서 중심~벽 거리(m)를 줄자로 재고 '
              '기준값에 입력. 여러 거리에서 정지 수집을 3회 이상 반복. 정면 레이와 벽 모양으로 '
              '방향 확인. 단일 벽 거리만으로 XYZ/RPY 전체가 자동 결정되지는 않습니다.'),
    'ultrasonic': ('초음파', '넓고 평평한 판을 센서 정면에 배치. 센서~판 실측 거리(m) 입력. '
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
    'extrinsics': ('바닥 보드 외부 보정', '내부 보정 후 체커보드를 바닥에 평평하게 놓고 로봇을 정지. '
                   '사진의 빨간 0번 코너와 열/행 방향을 바닥 표시와 대조하세요. '
                   '높이·각도는 영상으로 추정합니다. base_link 전체 pose 적용에는 보드 0번의 '
                   '바닥 XY·열 방향 yaw와 base_link 바닥 높이가 추가로 필요합니다. '
                   '이는 카메라 각도를 재는 것이 아니라 보드 좌표를 로봇에 등록하는 측정입니다.'),
}


def finite(value):
    value = float(value)
    if not np.isfinite(value):
        raise ValueError('유한한 실측 숫자를 입력하세요.')
    return value


def wheel_fit(trials):
    rows, targets = [], []
    for t in trials:
        left, right, distance, angle = map(finite, (t['left_rad'], t['right_rad'],
                                                    t['distance_m'], t['angle_rad']))
        rows.extend([[left, right, 0], [-left, right, -angle]])
        targets.extend([2 * distance, 0])
    matrix = np.asarray(rows)
    if len(trials) < 4 or np.linalg.matrix_rank(matrix) < 3:
        raise ValueError('전후진·좌우 회전을 포함한 독립 실험이 최소 4회 필요합니다.')
    values, _, _, _ = np.linalg.lstsq(matrix, targets, rcond=None)
    if not (0.005 < values[0] < .1 and .005 < values[1] < .1 and .03 < values[2] < .5):
        raise ValueError('바퀴 추정값이 범위를 벗어납니다. 단위·부호·실측값을 확인하세요.')
    error = matrix @ values - targets
    return {'left_radius_m': float(values[0]), 'right_radius_m': float(values[1]),
            'separation_m': float(values[2]), 'equation_residual': stats(np.abs(error)),
            'condition_number': float(np.linalg.cond(matrix)), 'trials': len(trials)}


def scalar_report(values, reference):
    values = np.asarray(values, dtype=float)
    valid = values[np.isfinite(values)]
    if len(valid) < 10:
        raise ValueError('유효 표본 10개 이상 필요합니다. 토픽·배치·감지 범위를 확인하세요.')
    result = stats(valid)
    result.update(reference=finite(reference), bias=float(valid.mean() - reference),
                  invalid_fraction=float(1 - len(valid) / len(values)))
    return result


class Session:
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
        self.write('session.yaml', self.meta)

    def write(self, name, value):
        path = self.root / name
        temp = path.with_suffix('.tmp')
        temp.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False))
        temp.replace(path)
        return path

    def capture(self, frame, purpose):
        if purpose == 'training' and self.camera:
            raise ValueError('학습 모델이 고정됐습니다. 새 세션을 만드세요.')
        if purpose == 'validation' and not self.camera:
            raise ValueError('먼저 학습을 완료하세요.')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = self.board.detect(gray)
        if corners is None:
            raise ValueError(f'{self.board.pattern} 내부 코너가 전부 보이게 보드를 배치하세요.')
        if cv2.Laplacian(gray, cv2.CV_64F).var() < 30:
            raise ValueError('영상이 흐립니다. 보드를 멈추고 초점을 확인하세요.')
        folder = self.root / purpose
        folder.mkdir(exist_ok=True)
        path = folder / f'{len(self.frames[purpose]):04d}.png'
        if not cv2.imwrite(str(path), frame):
            raise OSError('사진 저장 실패')
        self.frames[purpose].append(path)
        if purpose == 'validation':
            self.results.pop('camera', None)
        return len(self.frames[purpose])

    def camera_fit(self, square_mm):
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
        self.results.pop('camera', None)
        return {'training_rms_px': float(rms), 'next': '새 자세로 검증 사진 8장 이상 촬영'}

    def camera_validate(self):
        from .core import score_view
        if not self.camera or len(self.frames['validation']) < 8:
            raise ValueError('학습 후 독립 검증 사진 8장 이상 필요합니다.')
        k, d, size, square = self.camera
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
        report['passed'] = max(errors) <= .7 and min(np.ptp(centers, axis=0)) >= .25
        self.results['camera'] = report
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
        return report

    def configure_board(self, board):
        if board != self.board and (any(self.frames.values()) or self.camera):
            raise ValueError('촬영 후 보드를 변경할 수 없습니다. 새 세션을 만드세요.')
        self.board = board
        self.meta.update(board_inner_corners=list(board.pattern), square_mm=board.square_mm)
        self.write('session.yaml', self.meta)

    def floor_capture(self, frame, registration=None):
        if not self.results.get('camera', {}).get('passed'):
            raise ValueError('내부 보정과 독립 검증을 먼저 통과하세요.')
        k, d, size, _ = self.camera
        if frame.shape[:2][::-1] != size:
            raise ValueError('내부 보정과 영상 해상도가 다릅니다.')
        corners = self.board.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if corners is None:
            raise ValueError('바닥 보드 전체 내부 코너가 보여야 합니다.')
        result = floor_pose(self.board.points(), corners, k, d)
        if registration is not None:
            result['base_pose'] = registered_pose(result, *registration)
            result['registration'] = list(registration)
        entries = self.results.setdefault('floor_camera', [])
        folder = self.root / 'floor'
        folder.mkdir(exist_ok=True)
        if not cv2.imwrite(str(folder / f'{len(entries):04d}.png'), frame):
            raise OSError('바닥 사진 저장 실패')
        entries.append(result)
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
        self.write('results.yaml', self.results)
        return dict(result, repeats=len(entries),
                    height_statistics=stats([e['height_m'] for e in entries]),
                    applicable='extrinsics' in self.results)

    def export(self, confirmed):
        if not confirmed or not self.results:
            raise ValueError('분석 결과와 사람이 수행한 독립 확인이 필요합니다.')
        if 'camera' in self.results and not self.results['camera'].get('passed'):
            raise ValueError('카메라 검증 실패를 해결하거나 새 세션으로 분리하세요.')
        profile = dict(self.meta, results=self.results, launch={}, camera_pose=None)
        if self.results.get('camera', {}).get('passed'):
            profile['launch']['camera_info_url'] = (self.root / 'camera.yaml').as_uri()
        if 'wheels' in self.results:
            w = self.results['wheels']
            # Official motor driver supports one effective radius, not separate wheel radii.
            profile['launch'].update(wheel_radius=(w['left_radius_m'] + w['right_radius_m']) / 2,
                                     wheel_separation=w['separation_m'])
        if 'extrinsics' in self.results:
            profile['camera_pose'] = self.results['extrinsics']
        profile['human_verified'] = True
        self.write('results.yaml', self.results)
        self.write('wheel_trials.yaml', self.trials)
        return self.write('profile.yaml', profile)
