"""Convert measured bumper displacement under an explicit constant-curvature model."""

import math

import numpy as np

from .core import stats


def distance_measurement(kind, distance, reference):
    """Record signed start-to-end distance without inferring yaw from lateral drift."""
    distance = float(distance)
    if (kind not in ('forward', 'backward') or not math.isfinite(distance)
            or distance * (1 if kind == 'forward' else -1) < .2):
        raise ValueError('시작~끝 거리(m)는 전진 양수·후진 음수, 크기 0.2m 이상으로 입력하세요.')
    return dict(measurement_type='straight_distance', distance_m=distance,
                measurement_reference='bumper_front_center',
                assumptions=['approximately_straight_path', 'start_to_end_chord'], **reference)


def distance_rotation_fit(trials):
    """Fit one effective radius and track width from independent distance/turn runs."""
    kinds = {row.get('kind') for row in trials}
    if kinds != {'forward', 'backward', 'left', 'right'}:
        raise ValueError('전후진·좌우 회전을 포함한 독립 실험이 최소 4회 필요합니다.')
    straight, turns, normalization, repeats = [], [], [], {}
    expected = {'forward': (1, 1), 'backward': (-1, -1),
                'left': (-1, 1), 'right': (1, -1)}
    for row in trials:
        kind = row['kind']
        raw = np.asarray([row['left_rad'], row['right_rad']], dtype=float)
        if not np.isfinite(raw).all() or np.min(np.abs(raw)) < 1e-6:
            raise ValueError('바퀴 엔코더 표본이 없거나 회전량이 0입니다.')
        signs = np.asarray(expected[kind]) * np.sign(raw)
        left, right = raw * signs
        normalization.append({'trial_id': row.get('id'), 'kind': kind,
                              'encoder_multipliers': signs.tolist()})
        if kind in ('forward', 'backward'):
            if 'longitudinal_m' in row:
                chord = math.hypot(row['longitudinal_m'], row.get('lateral_m', 0))
                distance = math.copysign(chord, row['longitudinal_m'])
            else:
                distance = float(row['distance_m'])
            distance_measurement(kind, distance, {})
            straight.append(((left+right)/2, distance))
            repeats.setdefault(kind, []).append(distance)
        else:
            angle = float(row['angle_rad'])
            rotation_measurement(kind, math.degrees(angle))
            turns.append((right-left, angle))
            repeats.setdefault(kind, []).append(angle)
    a, target = np.asarray(straight).T
    radius = float(np.dot(a, target)/np.dot(a, a))
    difference, angles = np.asarray(turns).T
    separation = float(radius*np.dot(angles, difference)/np.dot(angles, angles))
    if not .005 < radius < .1 or not .03 < separation < .5:
        raise ValueError('바퀴 추정값이 범위를 벗어납니다. 거리 m·회전 deg·엔코더를 확인하세요.')
    residual = np.r_[a*radius-target, difference*radius-angles*separation]
    return {'left_radius_m': radius, 'right_radius_m': radius, 'separation_m': separation,
            'effective_radius_m': radius, 'individual_radii_identified': False,
            'radius_repeats_m': stats(target/a),
            'separation_repeats_m': stats(radius*difference/angles),
            'radius_model': 'shared_effective_radius', 'trials': len(trials),
            'encoder_normalization': normalization, 'equation_residual': stats(np.abs(residual)),
            'repeat_measurements': {kind: {('distance_m' if kind in ('forward', 'backward')
                                           else 'angle_rad'): stats(values)}
                                    for kind, values in repeats.items()},
            'straight_model': 'start_to_end_distance; lateral_drift_not_used_as_yaw',
            'assumption': 'approximately_straight_runs; shared_effective_radius'}


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
