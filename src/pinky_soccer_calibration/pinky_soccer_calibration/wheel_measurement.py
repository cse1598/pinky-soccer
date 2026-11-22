"""Convert measured bumper displacement under an explicit constant-curvature model."""

import math

import numpy as np


def straight_measurement(kind, longitudinal, lateral, reference):
    """Keep raw X/Y; infer axle arc length and yaw, never from odometry."""
    x, y = float(longitudinal), float(lateral)
    b = np.asarray(reference['bumper_from_axle_xyz_m'], dtype=float)
    if b.shape != (3,) or not np.isfinite(b).all() or not np.isfinite([x, y]).all():
        raise ValueError('유한한 이동 거리·편향·범퍼 기준 좌표가 필요합니다.')
    if kind not in ('forward', 'backward') or x * (1 if kind == 'forward' else -1) <= 0:
        raise ValueError('전진 거리는 양수, 후진 거리는 음수로 입력하세요.')
    if abs(x) < .2 or abs(x + 2*b[0]) < .1:
        raise ValueError('전후진 측정 거리가 너무 짧습니다. 1m 실험을 다시 실행하세요.')
    angle = 2*math.atan(y/(x + 2*b[0]))
    if abs(angle) > math.radians(45):
        raise ValueError('편향이 너무 큽니다. 미끄러짐·기준선·단위를 확인 후 재실험하세요.')
    half = angle/2
    chord = x*math.cos(half) + y*math.sin(half) + 2*b[1]*math.sin(half)
    distance = chord / np.sinc(half/np.pi)
    return dict(measurement_type='straight_displacement', longitudinal_m=x, lateral_m=y,
                distance_m=float(distance), angle_rad=angle,
                derivation='constant_curvature_bumper_displacement',
                assumptions=['constant_curvature', 'no_lateral_slip', 'initial_heading_aligned'],
                measurement_reference='bumper_front_center', **reference)


def rotation_measurement(kind, degrees):
    """Record signed accumulated yaw without assuming a measured translation."""
    angle = float(degrees)
    if kind not in ('left', 'right') or not math.isfinite(angle):
        raise ValueError('회전 실험과 유한한 실측 회전각이 필요합니다.')
    if angle * (1 if kind == 'left' else -1) <= 0:
        raise ValueError('좌회전은 양수, 우회전은 음수로 입력하세요.')
    if not 180 <= abs(angle) <= 540:
        raise ValueError('한 바퀴 누적 회전각을 입력하세요(예: +358 또는 -362도).')
    return {'measurement_type': 'rotation_angle', 'angle_rad': math.radians(angle),
            'measured_angle_deg': angle}
