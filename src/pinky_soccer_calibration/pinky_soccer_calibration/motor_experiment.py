"""Motor step-response experiments: plans, safety limits and raw-data preservation.

Raw data first: every experiment keeps the continuous Bag/CSV recording it ran in and its
own CSV extract (sensor observations + every published command with timestamps), so
models can be identified or re-identified later without the robot. Nothing here changes
the MPC or applies identified values.
"""
import csv
import json
import math
from pathlib import Path
import time
import uuid

import yaml

KINDS = {   # label, sign of (v, w) used with the entered magnitudes
    'forward': ('직진 계단응답', (1, 0)),
    'backward': ('후진 계단응답', (-1, 0)),
    'spin_left': ('제자리 좌회전', (0, 1)),
    'spin_right': ('제자리 우회전', (0, -1)),
    'combined': ('복합 (선속도+각속도)', (1, 1)),
}
RAW_TOPICS = ('/joint_states', '/odom', '/cmd_vel', '/imu_raw', '/batt_state',
              '/battery/voltage', '/battery/percent')
SAFE_LIMITS = {'v': .20, 'w': 2.0}           # default caps (m/s, rad/s)
APPROVED_LIMITS = {'v': .25, 'w': 3.0}       # after explicit high-speed approval
COMMAND_RATE_HZ = 50.


def wheel_parameters(session):
    """Kinematics used to check limits, with their provenance (never silently swapped)."""
    wheels = session.results.get('wheels') if 'wheels' not in session.stale else None
    if wheels and {'left_radius_m', 'right_radius_m', 'separation_m'} <= set(wheels):
        return {'radius_m': (wheels['left_radius_m'] + wheels['right_radius_m']) / 2,
                'separation_m': wheels['separation_m'], 'source': 'project_wheels_result',
                'project_revision': session.revision}
    return {'radius_m': .027, 'separation_m': .0961, 'source': 'official_default (unmeasured)',
            'project_revision': session.revision}


def wheel_speed_limit():
    """Controller wheel limit (0.9 x manual 103 rpm) from pinky_soccer_dynamics."""
    try:
        from pinky_soccer_dynamics import params
        return float(params.load('real').wheel_cmd_max())
    except Exception:           # dynamics package missing: the same documented value
        return .9 * 103 * 2 * math.pi / 60


def build_plan(kind, speed, rate, pre, hold, post, max_distance, approved=False,
               wheels=None):
    """Validated command profile; raises ValueError with the reason."""
    if kind not in KINDS:
        raise ValueError('실험 종류를 선택하세요.')
    sv, sw = KINDS[kind][1]
    v, w = sv * abs(speed), sw * abs(rate)
    if kind in ('forward', 'backward') and not speed:
        raise ValueError('선속도를 입력하세요.')
    if kind.startswith('spin') and not rate:
        raise ValueError('각속도를 입력하세요.')
    for name, value in (('정지 준비', pre), ('명령 유지', hold), ('정지 후 기록', post),
                        ('최대 이동 거리', max_distance)):
        if not (math.isfinite(value) and value > 0):
            raise ValueError(f'{name} 값이 올바르지 않습니다.')
    if hold > 10 or pre > 10 or post > 10:
        raise ValueError('각 구간은 10초 이하로 설정하세요.')
    limits = APPROVED_LIMITS if approved else SAFE_LIMITS
    if abs(v) > limits['v'] or abs(w) > limits['w']:
        raise ValueError(f"속도 제한 초과: |v|≤{limits['v']} m/s, |w|≤{limits['w']} rad/s"
                         + ('' if approved else ' (고속은 별도 승인 필요)'))
    if abs(v) * hold > max_distance + 1e-9:
        raise ValueError(f'예상 이동 {abs(v)*hold:.2f} m > 최대 이동 거리 {max_distance:.2f} m')
    wheels = wheels or {'radius_m': .027, 'separation_m': .0961}
    r, b = wheels['radius_m'], wheels['separation_m']
    left, right = (v - w * b / 2) / r, (v + w * b / 2) / r
    limit = wheel_speed_limit()
    if max(abs(left), abs(right)) > limit:
        raise ValueError(f'바퀴 속도 {max(abs(left), abs(right)):.2f} rad/s > 한계 {limit:.2f} rad/s')
    return {'kind': kind, 'segments': [(pre, 0., 0.), (hold, v, w), (post, 0., 0.)],
            'rate_hz': COMMAND_RATE_HZ, 'max_distance_m': max_distance,
            'max_angle_rad': max(abs(w) * hold * 1.5, .5),
            'commanded': {'v_mps': v, 'w_radps': w,
                          'wheel_left_radps': left, 'wheel_right_radps': right},
            'wheel_parameters': wheels, 'limits': limits, 'high_speed_approved': approved}


def start_record(session, plan, options, recording):
    """Persist the experiment as 'running' before any command is sent."""
    exp_id = uuid.uuid4().hex
    folder = Path('motor_response') / exp_id
    (session.root / folder).mkdir(parents=True)
    payload = {'experiment_id': exp_id, 'status': 'running', 'plan': plan,
               'purpose': options['purpose'], 'floor': options['floor'],
               'robot_id': session.meta['robot_id'], 'namespace': options['namespace'],
               'started_unix': time.time(), 'files': {'folder': str(folder)},
               'recording': recording, 'clock_sync': options.get('clock_sync')}
    row = session.add_measurement('motor_response', payload)
    return row


def finish_record(session, row_id, node, trial, battery=None):
    """Write the raw extract and finalize the record status (never deletes data)."""
    rows = session.measurements['motor_response']
    row = next(r for r in rows if r['id'] == row_id)
    folder = session.root / row['files']['folder']
    t0 = trial['started'] - .5          # a little before the first (zero) command
    t1 = trial.get('stopped_monotonic', time.monotonic()) + .3
    with node.lock:
        series = {t: [v for v in node.values.get(t, []) if t0 <= v[0] <= t1] for t in RAW_TOPICS}
    with (folder / 'observations.csv').open('x', newline='') as f:
        out = csv.writer(f)
        out.writerow(['topic', 'receive_monotonic', 'header_seconds', 'observation_json'])
        for topic, values in series.items():
            for mono, stamp, obs in values:
                out.writerow([topic, mono, stamp, json.dumps(obs)])
    with (folder / 'commands.csv').open('x', newline='') as f:
        out = csv.writer(f)
        out.writerow(['publish_monotonic', 'publish_ros_ns', 'publish_unix_ns', 'v_mps', 'w_radps'])
        out.writerows(trial.get('commands', []))
    reason = trial.get('reason', '')
    status = ('complete' if trial.get('complete') else
              'user_stopped' if '사용자' in reason or '즉시' in reason else
              'safety_stopped' if '안전' in reason or '센서' in reason else 'system_error')
    counts = {t: len(v) for t, v in series.items()}
    row.update(status=status, stop_reason=reason, ended_unix=time.time(),
               samples=counts, commands_sent=len(trial.get('commands', [])),
               start_odom=trial.get('start'), odom_angle_rad=trial.get('angle'),
               battery_at_end=battery, analysis_status='pending')
    row['files'].update(observations='observations.csv', commands='commands.csv')
    (folder / 'meta.yaml').write_text(yaml.safe_dump(row, allow_unicode=True, sort_keys=False))
    session.invalidate('motor_response')
    session.save()
    return row


def recover_interrupted(session):
    """Mark experiments left 'running' by a crash as interrupted (data kept, never resumed)."""
    changed = 0
    for row in session.measurements.get('motor_response', []):
        if row.get('status') == 'running':
            row.update(status='interrupted', stop_reason='프로그램 종료/오류로 미완료 (다음 실행에서 발견)')
            changed += 1
    if changed:
        session.save()
    return changed


def command_rate(commands):
    """Achieved publish rate statistics of a run (Hz, from the logged monotonic times)."""
    times = [c[0] for c in commands]
    if len(times) < 3:
        return None
    gaps = [b - a for a, b in zip(times, times[1:])]
    gaps.sort()
    return {'mean_hz': (len(times) - 1) / (times[-1] - times[0]),
            'gap_p95_s': gaps[int(.95 * (len(gaps) - 1))], 'gap_max_s': gaps[-1]}
