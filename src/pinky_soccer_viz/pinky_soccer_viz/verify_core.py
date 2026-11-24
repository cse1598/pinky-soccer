"""
ROS-free logic of the behaviour verifier: catalog, path prediction and run evaluation.

Path prediction runs the real controller law (pinky_soccer.reactive_controller.compute_command)
on a unicycle model from the current estimated pose through the goal's planned_path, so the
map shows where the current command would take the robot. It ignores wheel dynamics, slip and
the BT re-planning while moving, so it is a preview, not a promise.
"""

import csv
import math
import os

from pinky_soccer.reactive_controller import compute_command, Gains
import yaml

WORLD_INPUTS = ('playing', 'pose', 'ball', 'opponents', 'own_goal', 'opp_goal')
TERMINAL = ('success', 'failure', 'stopped', 'timeout', 'world_model_stale',
            'verifier_lost', 'error', 'done_condition')
STATE_TEXT = {
    'idle': '대기', 'running': '실행 중', 'success': 'SUCCESS', 'failure': 'FAILURE',
    'stopped': '사용자 정지', 'timeout': '제한 시간 초과', 'world_model_stale': '월드 모델 끊김',
    'verifier_lost': '검증 UI 연결 끊김', 'error': '요청 오류', 'done_condition': '완료 조건 충족',
}


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    """Return the yaw of a geometry_msgs Quaternion-like object."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------- catalog
def load_catalog(path):
    """Load and minimally validate the verifier catalog YAML."""
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if data.get('schema') != 1:
        raise ValueError(f'{path}: unsupported catalog schema {data.get("schema")}')
    ids = [item['id'] for item in data['items']]
    if len(ids) != len(set(ids)):
        raise ValueError(f'{path}: duplicate item ids')
    for item in data['items']:
        item.setdefault('ports', [])
        item.setdefault('inputs', [])
        for port in item['ports']:
            if not port['min'] <= port['default'] <= port['max']:
                raise ValueError(f'{item["id"]}.{port["name"]}: default outside [min, max]')
        for key in item.get('pose_input', {}).values():
            if key not in {p['name'] for p in item['ports']}:
                raise ValueError(f'{item["id"]}: pose_input refers to unknown port {key}')
    return data


def enabled_items(catalog):
    """Items shown and runnable in the verifier (`enabled: false` hides one)."""
    return [item for item in catalog['items'] if item.get('enabled', True)]


def groups(catalog):
    """Return [(group, [enabled items])] in catalog order."""
    out = {}
    for item in enabled_items(catalog):
        out.setdefault(item.get('group', '기타'), []).append(item)
    return list(out.items())


def format_port(value):
    """Return a compact decimal string for a port value."""
    text = f'{float(value):.6f}'.rstrip('0').rstrip('.')
    return '0' if text in ('-0', '') else text


def run_request(request_id, item, values, timeout_sec):
    """Build the bt_node run request (see pinky_soccer_bt/verify_mode.hpp)."""
    ports = {}
    for port in item['ports']:
        value = float(values[port['name']])
        if not math.isfinite(value):
            raise ValueError(f'{port["label"]}: 숫자가 아닙니다')
        if not port['min'] <= value <= port['max']:
            raise ValueError(f'{port["label"]}: {port["min"]} ~ {port["max"]} 범위 밖')
        ports[port['name']] = format_port(value)
    return {'command': 'run', 'id': int(request_id), 'target': item['id'], 'ports': ports,
            'timeout_sec': float(timeout_sec)}


def probe_request(catalog):
    """Build the condition probe request."""
    return {'command': 'probe', 'conditions': [
        {'label': p['label'], 'target': p['target'], 'ports': p.get('ports') or {}}
        for p in catalog.get('probe', [])]}


def snap_angle(theta, step_deg=15.0):
    """Round an angle to multiples of step_deg."""
    step = math.radians(step_deg)
    return wrap(round(theta / step) * step)


def drag_pose(start, end, previous_theta, min_drag=0.03, snap=False):
    """Pose from a map press at `start` dragged to `end`; a short drag keeps the old heading."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    theta = math.atan2(dy, dx) if math.hypot(dx, dy) >= min_drag else previous_theta
    if snap and theta is not None:
        theta = snap_angle(theta)
    return start[0], start[1], theta


# ---------------------------------------------------------------- controller preview
def load_gains(config_path=None, overrides=None):
    """Load controller gains from reactive_controller.yaml (if given) plus overrides."""
    values = {}
    if config_path and os.path.exists(config_path):
        with open(config_path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        values = (data.get('reactive_controller') or {}).get('ros__parameters') or {}
    values.update(overrides or {})
    fields = Gains.__dataclass_fields__
    return Gains(**{k: float(v) for k, v in values.items() if k in fields})


def predict_path(pose, waypoints, gains, max_speed=0.0, dt=0.05, horizon=20.0,
                 switch_tol=0.06):
    """
    Simulate the controller on a unicycle from pose (x, y, th) through waypoints.

    Waypoints are (x, y, th); intermediate ones are passed once within switch_tol.
    Returns the list of (x, y) points (starting at pose).
    """
    if not waypoints:
        return []
    x, y, th = pose
    points = [(x, y)]
    index = 0
    for _ in range(int(horizon / dt)):
        target = waypoints[index]
        last = index == len(waypoints) - 1
        reach = max(gains.pos_tol, switch_tol)
        if not last and math.hypot(target[0] - x, target[1] - y) <= reach:
            index += 1
            continue
        v, w = compute_command((x, y, th), target, gains, max_speed)
        if last and v == 0.0 and w == 0.0:
            break
        if abs(w) > 1e-9:
            x += v / w * (math.sin(th + w * dt) - math.sin(th))
            y -= v / w * (math.cos(th + w * dt) - math.cos(th))
        else:
            x += v * dt * math.cos(th)
            y += v * dt * math.sin(th)
        th = wrap(th + w * dt)
        if math.hypot(x - points[-1][0], y - points[-1][1]) >= 0.005:
            points.append((x, y))
    return points


# ---------------------------------------------------------------- evaluation
def _path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


class RunRecord:
    """Samples of one verification run and its evaluation."""

    FIELDS = ('t', 'est_x', 'est_y', 'est_th', 'true_x', 'true_y', 'true_th', 'ball_x', 'ball_y',
              'ball_state', 'true_ball_x', 'true_ball_y', 'cmd_v', 'cmd_w', 'action',
              'target_x', 'target_y', 'target_th',
              'mpc_state', 'mpc_status', 'mpc_ms', 'mpc_direction', 'wheel_l', 'wheel_r',
              'profile_l', 'profile_r', 'wheel_cmd_l', 'wheel_cmd_r', 'feedback_age',
              'feedback_enabled', 'period_s', 'v_bound', 'w_bound', 'mpc_residuals')

    def __init__(self, number, item, ports, start_time, field=None):
        self.number = number
        self.item = item
        self.ports = dict(ports)
        self.start = start_time
        self.field = field
        self.rows = []
        self.state = 'running'
        self.message = ''
        self.end = None
        self.goal_scored = False
        self.ball_started_in_goal = None    # a ball already in the goal does not count
        self.initial_prediction = []

    @property
    def done(self):
        """Return True once the run has a terminal state."""
        return self.state in TERMINAL

    def add(self, t, est, truth, ball, ball_state, ball_truth, cmd, goal, mpc=None):
        """Append one sample; est/truth/goal are (x, y, th) or None."""
        def part(v, n):
            return list(v[:n]) if v is not None else [None] * n
        if goal and goal[0].startswith('Verify'):
            goal = None             # idle/stop goals of bt_node between runs
        target = goal[1] if goal else None
        self.rows.append([t - self.start] + part(est, 3) + part(truth, 3) + part(ball, 2) +
                         [ball_state] + part(ball_truth, 2) + part(cmd, 2) +
                         [goal[0] if goal else ''] + part(target, 3))
        diag = mpc or {}
        self.rows[-1].extend(diag.get(k) for k in ('state','status','ms','direction',
            'wheel_l','wheel_r','profile_l','profile_r','wheel_cmd_l','wheel_cmd_r',
            'feedback_age','feedback_enabled','period_s','v_bound','w_bound','residuals'))
        if self.field is not None:
            b = ball_truth if ball_truth is not None else ball
            if b is not None:
                inside = self.field.ball_in_goal(b[0], b[1], self.field.opp_goal_color)
                if self.ball_started_in_goal is None:
                    self.ball_started_in_goal = inside
                elif inside and not self.ball_started_in_goal:
                    self.goal_scored = True

    def finish(self, t, state, message=''):
        """Close the run with a terminal state."""
        if not self.done:
            self.state, self.message, self.end = state, message, t

    def column(self, *names):
        """Return rows of the given columns where all are present."""
        idx = [self.FIELDS.index(n) for n in names]
        return [tuple(r[i] for i in idx) for r in self.rows if all(r[i] is not None for i in idx)]

    def metrics(self):
        """Return [(label, value_text)] for the evaluation panel."""
        out = [('결과', STATE_TEXT.get(self.state, self.state) +
                (f' ({self.message})' if self.message else ''))]
        duration = (self.end if self.end is not None else
                    (self.start + self.rows[-1][0] if self.rows else self.start)) - self.start
        out.append(('소요 시간', f'{duration:.2f} s'))
        est = self.column('est_x', 'est_y', 'est_th')
        truth = self.column('true_x', 'true_y', 'true_th')
        if est:
            out.append(('이동 거리(추정)', f'{_path_length(est):.3f} m'))
        if truth:
            out.append(('이동 거리(정답)', f'{_path_length(truth):.3f} m'))
        pose_input = self.item.get('pose_input')
        if pose_input:
            goal = tuple(float(self.ports[pose_input[k]]) for k in ('x', 'y', 'theta'))
            for label, series in (('추정', est), ('정답', truth)):
                if series:
                    x, y, th = series[-1]
                    out.append((f'목표 위치 오차({label})',
                                f'{100 * math.hypot(goal[0] - x, goal[1] - y):.1f} cm'))
                    out.append((f'목표 방향 오차({label})',
                                f'{math.degrees(wrap(goal[2] - th)):+.1f}°'))
        else:
            targets = self.column('target_x', 'target_y', 'est_x', 'est_y')
            if targets:
                tx, ty, x, y = targets[-1]
                out.append(('마지막 목표까지(추정)', f'{100 * math.hypot(tx - x, ty - y):.1f} cm'))
        pairs = [r for r in self.rows if r[1] is not None and r[4] is not None]
        if pairs:
            errs = [math.hypot(r[1] - r[4], r[2] - r[5]) for r in pairs]
            herrs = [abs(math.degrees(wrap(r[3] - r[6]))) for r in pairs]
            out.append(('위치 추정 오차 평균/최대',
                        f'{100 * sum(errs) / len(errs):.1f} / {100 * max(errs):.1f} cm'))
            out.append(('방향 추정 오차 평균/최대',
                        f'{sum(herrs) / len(herrs):.1f} / {max(herrs):.1f}°'))
        balls = self.column('true_ball_x', 'true_ball_y') or self.column('ball_x', 'ball_y')
        if balls:
            src = '정답' if self.column('true_ball_x') else '추정'
            moved = math.hypot(balls[-1][0] - balls[0][0], balls[-1][1] - balls[0][1])
            out.append((f'공 이동({src})', f'{100 * moved:.1f} cm'))
        if self.field is not None:
            out.append(('득점', '예' if self.goal_scored else (
                '공이 처음부터 골대 안' if self.ball_started_in_goal else '아니오')))
        facing = self.column('est_x', 'est_y', 'est_th', 'ball_x', 'ball_y')
        if facing:
            x, y, th, bx, by = facing[-1]
            err = math.degrees(wrap(math.atan2(by - y, bx - x) - th))
            out.append(('공 방향 오차(추정)', f'{err:+.1f}°'))
        actions = [r[14] for r in self.rows if r[14]]
        switches = sum(1 for a, b in zip(actions, actions[1:]) if a != b)
        out.append(('행동 전환 횟수', str(switches)))
        cmds = self.column('cmd_v', 'cmd_w')
        if cmds:
            vs = [abs(v) for v, _ in cmds]
            ws = [abs(w) for _, w in cmds]
            out.append(('명령 속도 평균/최대', f'{sum(vs) / len(vs):.3f} / {max(vs):.3f} m/s'))
            out.append(('명령 회전 최대', f'{max(ws):.2f} rad/s'))
        return out

    def summary(self):
        """Return one short line for the history table."""
        m = dict(self.metrics())
        key = (m.get('목표 위치 오차(정답)') or m.get('목표 위치 오차(추정)') or
               m.get('득점') or m.get('마지막 목표까지(추정)') or '')
        return m['결과'], m['소요 시간'], key

    def write(self, directory):
        """Write samples to run_<n>.csv and append the summary to runs.csv."""
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, f'run_{self.number:03d}_{self.item["id"]}.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(self.FIELDS)
            writer.writerows(self.rows)
        summary = os.path.join(directory, 'runs.csv')
        new = not os.path.exists(summary)
        metrics = self.metrics()
        with open(summary, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if new:
                writer.writerow(['run', 'item', 'ports', 'metrics', 'samples_file'])
            writer.writerow([self.number, self.item['id'],
                             ' '.join(f'{k}={v}' for k, v in self.ports.items()),
                             '; '.join(f'{k}: {v}' for k, v in metrics),
                             os.path.basename(path)])
        return path
