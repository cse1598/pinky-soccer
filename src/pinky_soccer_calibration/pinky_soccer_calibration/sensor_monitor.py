"""Read-only sensor summaries with explicit units and missing-data semantics."""

import math

import numpy as np


def number(value, unit='', digits=3):
    return f'{value:.{digits}f}{unit}' if math.isfinite(value) else '미제공/유효하지 않음'


def vector(value):
    return ', '.join(number(getattr(value, axis)) for axis in 'xyz')


def attitude(q, covariance=None):
    """Report sensor-frame attitude; never turn an absent quaternion into zero yaw."""
    if covariance is not None and covariance[0] == -1:
        return '자세 미제공 (covariance=-1)'
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or not math.isfinite(norm) or abs(norm-1) > .02:
        return '유효하지 않은 quaternion (자세 계산 불가)'
    x, y, z, w = values/norm
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return 'RPY (deg): ' + ', '.join(number(math.degrees(a), digits=1)
                                    for a in (roll, pitch, yaw))


def range_text(msg):
    if math.isnan(msg.range):
        distance = '유효하지 않은 측정 (NaN)'
    elif msg.range == math.inf:
        distance = '최대 범위 초과/미검출 (+inf)'
    elif msg.range == -math.inf:
        distance = '최소 범위 미만 (-inf)'
    else:
        distance = number(msg.range, ' m')
        if not msg.min_range <= msg.range <= msg.max_range:
            distance += ' (센서 범위 밖)'
    return (f'{distance}\n감지 범위: {msg.min_range:g}~{msg.max_range:g} m · '
            f'FOV {math.degrees(msg.field_of_view):.1f}°')


def valid_scan(msg):
    ranges = np.asarray(msg.ranges, dtype=float)
    return ranges, np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)


def scan_points(msg, geometry=None, limit=3.):
    """Return XY points in base_link when known, otherwise in the actual scan frame."""
    ranges, mask = valid_scan(msg)
    mask &= ranges <= limit
    indices = np.flatnonzero(mask)
    indices = indices[::max(1, math.ceil(len(indices)/1500))]
    angles = msg.angle_min + indices*msg.angle_increment
    valid_angles = np.isfinite(angles)
    indices, angles = indices[valid_angles], angles[valid_angles]
    points = np.c_[ranges[indices]*np.cos(angles), ranges[indices]*np.sin(angles),
                   np.zeros(len(indices))]
    frame = msg.header.frame_id or '프레임 미지정'
    if geometry is not None:
        try:
            transform = geometry.sensor(frame)
            points = points @ transform[:3, :3].T + transform[:3, 3]
            frame = 'base_link'
        except ValueError:
            pass
    return points[:, :2], frame


def summary(topic, msg):
    if topic == '/battery/percent':
        return (number(msg.data, ' %', 1) if 0 <= msg.data <= 100 else
                '미제공 또는 범위 오류 (0~100%)')
    if topic == '/battery/voltage':
        return number(msg.data, ' V')
    if topic == '/batt_state':
        percent = (number(msg.percentage*100, ' %', 1)
                   if 0 <= msg.percentage <= 1 else '미제공')
        return (f'ADC 전압: {number(msg.voltage, " V")} · 잔량: {percent}\n'
                f'전류: {number(msg.current, " A")} · 온도: {number(msg.temperature, " °C")}')
    if topic in ('/imu_raw', '/imu_native'):
        q = msg.orientation
        unit = 'rad/s' if topic == '/imu_raw' else 'deg/s (드라이버 원본)'
        return (f'{attitude(q, msg.orientation_covariance)}\n'
                f'quaternion XYZW: {vector(q)}, {number(q.w)}\n'
                f'각속도 XYZ ({unit}): {vector(msg.angular_velocity)}\n'
                f'가속도 XYZ (m/s²): {vector(msg.linear_acceleration)}')
    if topic == '/scan':
        ranges, mask = valid_scan(msg)
        valid = ranges[mask]
        distances = (f'최단 {valid.min():.3f} m · 최장 {valid.max():.3f} m'
                     if valid.size else '유효 거리 없음')
        return (f'유효 {valid.size}/{len(ranges)} 레이 · {distances}\n'
                f'각도 {math.degrees(msg.angle_min):.1f}~{math.degrees(msg.angle_max):.1f}° · '
                f'간격 {math.degrees(msg.angle_increment):.3f}°')
    if topic == '/ir_sensor/range':
        return ('ADC raw [좌/중앙/우]: '+', '.join(str(v) for v in msg.data)+
                '\n실측 거리 보정표 없음: m/cm로 환산하지 않음')
    if topic == '/us_sensor/range' or topic.startswith('/sim/ir_'):
        return range_text(msg)
    if topic == '/odom':
        pose, twist = msg.pose.pose, msg.twist.twist
        return (f'위치 XYZ (m): {vector(pose.position)}\n{attitude(pose.orientation)}\n'
                f'선속도 XYZ (m/s): {vector(twist.linear)}\n'
                f'각속도 XYZ (rad/s): {vector(twist.angular)}')
    if topic == '/joint_states':
        return '\n'.join(f'{name}: 위치 {number(msg.position[i], " rad") if i < len(msg.position) else "미제공"}'
                         f' · 속도 {number(msg.velocity[i], " rad/s") if i < len(msg.velocity) else "미제공"}'
                         for i, name in enumerate(msg.name)) or '관절 값 없음'
    if topic == '/camera/image_raw':
        return f'{msg.width}×{msg.height} · {msg.encoding} · 카메라 영상은 왼쪽 미리보기'
    if topic == '/camera/image_raw/compressed':
        return f'JPEG 압축 미리보기 · {len(msg.data)/1024:.1f} KiB/프레임 · {msg.format}'
    if topic == '/camera/camera_info':
        return (f'{msg.width}×{msg.height} · {msg.distortion_model}\n'
                f'fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}, cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}\n'
                'D: '+', '.join(number(v) for v in msg.d))
    if topic == '/cmd_vel':
        return f'명령 선속도 XYZ (m/s): {vector(msg.linear)}\n명령 각속도 XYZ (rad/s): {vector(msg.angular)}'
    return '원본 상세 탭에서 확인'


MONITOR_TOPICS = [
    ('배터리 잔량', '/battery/percent', 15.),
    ('배터리 전압', '/battery/voltage', 15.),
    ('ADC 배터리 상태', '/batt_state', 2.),
    ('IMU · 센서 프레임 기준 자세', '/imu_raw', 1.5),
    ('LiDAR', '/scan', 1.5),
    ('초음파', '/us_sensor/range', 1.5),
    ('적외선 · 실물 원시 ADC', '/ir_sensor/range', 1.5),
    ('적외선 왼쪽 · 리허설 전용 거리', '/sim/ir_l_link/distance', 1.5),
    ('적외선 중앙 · 리허설 전용 거리', '/sim/ir_mid_link/distance', 1.5),
    ('적외선 오른쪽 · 리허설 전용 거리', '/sim/ir_r_link/distance', 1.5),
    ('오도메트리', '/odom', 1.5),
    ('바퀴 관절', '/joint_states', 1.5),
    ('카메라 · JPEG 미리보기', '/camera/image_raw/compressed', 1.5),
    ('카메라 내부 행렬', '/camera/camera_info', 3.),
    ('제어 명령 · 센서 실측 아님', '/cmd_vel', 1.5),
]


def readings(snapshot, now):
    messages, seen, arrivals = snapshot
    result = {}
    for title, topic, timeout in MONITOR_TOPICS:
        if topic not in messages:
            result[topic] = (title, '미수신 · launch/옵션/통신 확인', '—', False)
            continue
        age = max(0., now-seen[topic])
        times = arrivals.get(topic, [])
        hz = ((len(times)-1)/(times[-1]-times[0])
              if len(times) >= 2 and times[-1] > times[0] else None)
        status = ('수신 중' if age <= timeout else '수신 중단 · 마지막 값')
        status += f' · {age:.1f}초 전'
        if hz is not None:
            status += f' · {hz:.1f} Hz (수신 간격)'
        msg = messages[topic]
        frame = getattr(getattr(msg, 'header', None), 'frame_id', '')
        if frame:
            status += '\n프레임: '+frame
        result[topic] = (title, status, summary(topic, msg), age <= timeout)
    return result
