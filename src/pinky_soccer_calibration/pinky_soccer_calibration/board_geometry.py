"""Configurable chessboard geometry and camera pose above a measured floor board."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class Board:
    columns: int = 9
    rows: int = 6
    square_mm: float = 30.

    def __post_init__(self):
        if (type(self.columns) is not int or type(self.rows) is not int
                or not 3 <= self.columns <= 30 or not 3 <= self.rows <= 30
                or not math.isfinite(self.square_mm) or not 0 < self.square_mm < 1000):
            raise ValueError('내부 코너 수는 정수 3~30, 칸 길이는 양수 mm로 입력하세요.')

    @property
    def pattern(self):
        return self.columns, self.rows

    def points(self):
        result = np.zeros((self.columns*self.rows, 3), np.float32)
        result[:, :2] = np.mgrid[0:self.columns, 0:self.rows].T.reshape(-1, 2)
        result *= self.square_mm / 1000
        return result

    def detect(self, gray, fast=False):
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        if fast:
            flags |= cv2.CALIB_CB_FAST_CHECK
        ok, corners = cv2.findChessboardCorners(
            gray, self.pattern, flags)
        if not ok:
            return None
        return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def matrix_rpy(r):
    return [float(math.atan2(r[2, 1], r[2, 2])),
            float(math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))),
            float(math.atan2(r[1, 0], r[0, 0]))]


# Mechanical +X forward/+Y left/+Z up -> optical +X right/+Y down/+Z forward.
MECHANICAL_FROM_OPTICAL = rpy_matrix(-math.pi/2, 0, -math.pi/2)


def floor_pose(points, pixels, k, d, allow_normal_flip=False):
    """Invert planar PnP; reject below-floor and ambiguous positive-depth solutions."""
    points = np.asarray(points, dtype=np.float64)
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    if (len(points) != len(pixels) or len(points) < 9
            or not np.isfinite(pixels).all() or not np.isfinite(k).all()
            or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError('유효한 내부 보정값과 보드 코너가 필요합니다.')
    candidates = []
    models = [points]
    if allow_normal_flip:
        reflected = points.copy()
        reflected[:, 1] *= -1
        models.append(reflected)
    for model in models:
        ok, rotations, translations, _ = cv2.solvePnPGeneric(
            model, pixels, k, d, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        for rv, tv in zip(rotations, translations):
            camera_from_board, _ = cv2.Rodrigues(rv)
            if np.any((model @ camera_from_board.T + tv.reshape(1, 3))[:, 2] <= 0):
                continue
            board_from_camera = camera_from_board.T
            position = -board_from_camera @ tv.reshape(3)
            if not .01 < position[2] < 3.:
                continue
            projected, _ = cv2.projectPoints(model, rv, tv, k, d)
            rms = float(np.sqrt(np.mean(np.sum((pixels-projected)**2, axis=2))))
            mechanical = board_from_camera @ MECHANICAL_FROM_OPTICAL.T
            candidates.append((rms, position, mechanical))
    candidates.sort(key=lambda x: x[0])
    if not candidates or candidates[0][0] > .7:
        raise ValueError('바닥 pose 추정 실패: 내부 보정·평평한 배치·보드 전체 검출을 확인하세요.')
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < .05:
        raise ValueError('평면 pose가 모호합니다. 보드를 더 가까이/비스듬한 시야에 배치하세요.')
    rms, position, rotation = candidates[0]
    return {'height_m': float(position[2]), 'reprojection_rms_px': rms,
            'board_from_camera_xyz': position.tolist(),
            'board_from_mechanical_rotation': rotation.tolist(),
            'mechanical_rpy_rad': matrix_rpy(rotation)}


def registered_pose(result, board_x, board_y, board_yaw, base_height):
    """Register floor-board coordinates to base_link; no unmeasured XY/yaw guessed."""
    values = np.asarray([board_x, board_y, board_yaw, base_height], dtype=float)
    if not np.isfinite(values).all() or base_height < 0:
        raise ValueError('보드 원점·방향과 base_link의 바닥 높이를 실측 입력하세요.')
    rotation = rpy_matrix(0, 0, board_yaw)
    xyz = rotation @ result['board_from_camera_xyz'] + [board_x, board_y, -base_height]
    orientation = rotation @ np.asarray(result['board_from_mechanical_rotation'])
    return [*map(float, xyz), *matrix_rpy(orientation)]
