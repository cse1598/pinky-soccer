"""Calibration review orchestration using gameplay vision and BallFilter unchanged."""

from dataclasses import fields
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
from pinky_soccer_world_model.ball_filter import BallFilter, BallFilterParams, LOST
from pinky_soccer_world_model.ball_vision import annotate_ball, project_ball
from pinky_soccer_world_model.geometry import (
    BLOB_SHAPE_DEFAULTS, intrinsics_from_hfov)
from pinky_soccer_world_model.robust_ball import BallImageDetector, ROBUST_DEFAULTS
import yaml


def gameplay_settings(path=None):
    """Read the same installed detector and world-model parameter YAMLs as gameplay."""
    directory = Path(get_package_share_directory('pinky_soccer_world_model')) / 'config'
    detector = yaml.safe_load((directory / 'ball_detector.yaml').read_text())['ball_detector']['ros__parameters']
    world = yaml.safe_load((directory / 'world_model.yaml').read_text())['world_model_node']['ros__parameters']
    if path:
        data = yaml.safe_load(Path(path).read_text())
        if not isinstance(data, dict) or not {'ball_detector', 'world_model_node'} & data.keys():
            raise ValueError('ball_detector 또는 world_model_node 설정 YAML을 선택하세요.')
        detector.update(data.get('ball_detector', {}).get('ros__parameters', {}))
        world.update(data.get('world_model_node', {}).get('ros__parameters', {}))
    params = BallFilterParams(**{f.name: world['ball.' + f.name] for f in fields(BallFilterParams)
                                if 'ball.' + f.name in world})
    flat = detector['hsv_ranges']
    if not flat or len(flat) % 6:
        raise ValueError('HSV 범위는 6개씩 입력해야 합니다.')
    for key in ('ball_radius', 'min_area_px', 'max_range', 'approximate_hfov_deg'):
        detector[key] = float(detector[key])
        if not math.isfinite(detector[key]) or detector[key] <= 0:
            raise ValueError(f'{key}는 유한한 양수여야 합니다.')
    if not 0 <= detector['min_circularity'] <= 1 or params.friction_tau <= 0:
        raise ValueError('원형도/마찰 필터 설정을 확인하세요.')
    detector['ranges'] = [(tuple(flat[i:i+3]), tuple(flat[i+3:i+6])) for i in range(0, len(flat), 6)]
    for key, default in BLOB_SHAPE_DEFAULTS.items():
        detector[key] = float(detector.get(key, default))
        if not 0 <= detector[key] <= 1:
            raise ValueError(f'{key}는 0~1 범위여야 합니다.')
    for key, default in ROBUST_DEFAULTS.items():
        detector[key] = float(detector.get(key, default))
        if not math.isfinite(detector[key]) or detector[key] <= 0:
            raise ValueError(f'{key}는 유한한 양수여야 합니다.')
    return detector, params


def select_intrinsics(shape, session, info, external, approximate, hfov):
    """Prefer explicit file, current verified project, then live info; fallback is labelled."""
    size = shape[1], shape[0]
    model, source = None, ''
    project_camera = session.camera if session else None
    if external:
        model, source = external, '선택한 카메라 보정 파일'
    elif (session and project_camera and session.results.get('camera', {}).get('passed')
          and 'camera' not in session.stale):
        model, source = project_camera[:3], '현재 프로젝트 · 독립 검증 통과'
    elif info is not None:
        k = np.asarray(info.k, dtype=float).reshape(3, 3)
        if not np.isfinite(k).all():
            raise ValueError('CameraInfo 내부값이 유한하지 않습니다.')
        if k[0, 0] > 0 and k[1, 1] > 0:
            if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
                raise ValueError('경기용 pinhole 경로에서 지원하지 않는 왜곡 모델입니다.')
            model = k, np.asarray(info.d, dtype=float), (info.width, info.height)
            source = '실시간 CameraInfo · 출처/정확도는 제공 노드 기준'
    if model is not None:
        k, d, expected = model
        k, d = np.asarray(k, dtype=float), np.asarray(d, dtype=float).reshape(-1)
        if tuple(expected) != size:
            raise ValueError(f'보정 해상도 {tuple(expected)} ≠ 영상 {size}. 보정 파일을 확인하세요.')
        if (k.shape != (3, 3) or not np.isfinite(k).all() or not np.isfinite(d).all()
                or k[0, 0] <= 0 or k[1, 1] <= 0 or d.size not in (0, 4, 5, 8, 12, 14)):
            raise ValueError('유효하지 않은 카메라 내부 보정값입니다.')
        return k, d, source, False
    if approximate:
        hfov = float(hfov)
        return (intrinsics_from_hfov(*size, hfov), np.zeros(5),
                f'미보정 추정 · HFOV {hfov:g}° 가정 / 왜곡 0 가정', True)
    return None, None, '보정 없음 · 픽셀 검출만 표시', False


class GameplayReview:
    """Run shared gameplay functions; never publish detections or control commands."""

    def __init__(self, settings=None, filter_params=None):
        if settings is None:
            settings, filter_params = gameplay_settings()
        self.settings = settings
        self.params = filter_params or BallFilterParams()
        self.stable_frames = ('map', 'odom')
        shape = {key: settings.get(key, value) for key, value in BLOB_SHAPE_DEFAULTS.items()}
        options = {key: settings.get(key, value) for key, value in ROBUST_DEFAULTS.items()}
        self.visual = BallImageDetector(
            settings['ranges'], settings['min_area_px'], settings['min_circularity'], shape, options)
        self.reset()

    def reset(self):
        """Drop temporal state on source/settings/time changes or a stopped video stream."""
        self.filter = BallFilter(self.params)
        self.visual.reset()
        self.signature = None
        self.last_frame = None
        self.last_time = None

    def process(self, frame, frame_key, observed_at, now, header, lookup,
                session=None, info=None, external=None, approximate=True, hfov=66.0, rotated=False):
        """lookup(target, source, stamp) supplies (R,t), using the acquisition TF buffer."""
        cfg = self.settings
        observation = self.visual.detect(frame, observed_at, frame_key)
        blob = observation.blob
        report = {'blob': list(blob) if blob else None, 'point': None, 'tracking': None,
                  'visual_confidence': observation.confidence,
                  'visible_arc_fraction': observation.arc_fraction,
                  'partial_outline': observation.partial,
                  'frame': cfg['output_frame'], 'mount_source': '실행 중 카메라 TF (장착 실측 자동 적용 없음)'}
        k = d = None
        is_approximate = False
        try:
            k, d, source, is_approximate = select_intrinsics(
                frame.shape, session, info, external, approximate, hfov)
            report['intrinsics_source'] = source
        except ValueError as error:
            report['intrinsics_source'] = str(error)
        report['approximate'] = is_approximate
        report['intrinsics_available'] = k is not None
        calibration = 'none' if k is None else ('approximate' if is_approximate else 'calibrated')
        preview = annotate_ball(frame, blob, calibration, observation)
        report['reason'] = '주황색 공 후보 미검출' if blob is None else '미터 좌표 계산 대기'
        cam = lookup(cfg['floor_frame'], header.frame_id, header.stamp)
        output = lookup(cfg['output_frame'], cfg['floor_frame'], None)
        point = project_ball(blob, frame.shape, k, d, cam, output, cfg['ball_radius'],
                             cfg['max_range'], rotated)
        if blob is not None:
            if k is None:
                report['reason'] = '내부 보정/미보정 추정 설정 확인'
            elif cam is None or output is None:
                report['reason'] = '카메라/바닥/출력 TF 없음 · 픽셀 검출은 정상'
            elif point is None:
                report['reason'] = '바닥 교점 없음 또는 경기 검출 거리 제한 초과'
            else:
                report.update(point=point.tolist(), range_m=float(np.hypot(*point[:2])),
                              bearing_deg=math.degrees(math.atan2(point[1], point[0])), reason='공 관측')
        stable, stable_rt = None, None
        for candidate in self.stable_frames:
            rt = lookup(candidate, cfg['output_frame'], header.stamp)
            if rt is not None:
                stable, stable_rt = candidate, rt
                break
        signature = (None if k is None else k.tobytes(), None if d is None else d.tobytes(),
                     frame.shape, rotated, stable, repr(cfg),
                     None if cam is None else (cam[0].tobytes(), cam[1].tobytes()))
        if signature != self.signature or (self.last_time is not None and now < self.last_time):
            self.reset()
            self.signature = signature
        self.last_time = now
        if stable_rt is None:
            report['tracking_reason'] = 'map/odom TF 없음 · 경기 BallFilter 추적 대기'
        elif k is None:
            report['tracking_reason'] = '내부값 없음 · 추적 대기'
        elif abs(now - observed_at) > 2:
            self.filter = BallFilter(self.params)
            report['tracking_reason'] = '영상/PC ROS 시각 차이 >2초 · 추적 대기'
        else:
            if frame_key != self.last_frame:
                if point is not None:
                    world = stable_rt[0] @ point + stable_rt[1]
                    self.filter.update(world[:2], observed_at, float(np.hypot(*point[:2])))
                self.last_frame = frame_key
            self.filter.predict(now)
            state = self.filter.state(now)
            if state != LOST:
                current_rt = lookup(stable, cfg['output_frame'], None)
                if current_rt is not None:
                    world = np.array([*self.filter.x[:2], cfg['ball_radius']])
                    local = current_rt[0].T @ (world - current_rt[1])
                    report['tracking'] = {'state': ['OBSERVED', 'ESTIMATED', 'LOST'][state],
                                          'stable_frame': stable, 'point': local.tolist(),
                                          'range_m': float(np.hypot(*local[:2])),
                                          'sigma_m': self.filter.max_sigma(),
                                          'age_s': max(0., now-self.filter.last_seen)}
            report['tracking_reason'] = 'LOST' if state == LOST else '경기 BallFilter · 고정 좌표계 추적'
        return preview, report
