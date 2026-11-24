"""Behaviour verifier logic without ROS or a display."""

import math
import os
import types

from ament_index_python.packages import get_package_share_directory
from pinky_soccer.reactive_controller import Gains
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_viz import verify_core as core
import pytest

CATALOG = os.path.join(os.path.dirname(__file__), '..', '..', 'pinky_soccer_bt', 'config',
                       'verify_catalog.yaml')


@pytest.fixture(scope='module')
def catalog():
    return core.load_catalog(CATALOG)


def test_catalog_groups_and_requests(catalog):
    names = [g for g, _ in core.groups(catalog)]
    assert names[0] == '단일 행동' and '전체 경기' in names
    goto = next(i for i in catalog['items'] if i['id'] == 'GoToPose')
    req = core.run_request(7, goto, {'x': 0.5, 'y': -0.25, 'theta': 1.5708}, 20)
    assert req == {'command': 'run', 'id': 7, 'target': 'GoToPose',
                   'ports': {'x': '0.5', 'y': '-0.25', 'theta': '1.5708'}, 'timeout_sec': 20.0}
    with pytest.raises(ValueError):
        core.run_request(1, goto, {'x': 9.0, 'y': 0.0, 'theta': 0.0}, 20)
    probe = core.probe_request(catalog)
    assert probe['command'] == 'probe'
    assert {c['label'] for c in probe['conditions']} >= {'BallKnown', 'IsBehindBall'}


def test_drag_pose_and_snap():
    assert core.drag_pose((0.1, 0.2), (0.1, 0.21), 0.7) == (0.1, 0.2, 0.7)   # short: keep
    x, y, th = core.drag_pose((0, 0), (0, 0.2), 0.0)
    assert (x, y) == (0, 0) and th == pytest.approx(math.pi / 2)
    _, _, th = core.drag_pose((0, 0), (0.2, 0.05), 0.0, snap=True)
    assert th == pytest.approx(math.radians(15))
    assert core.drag_pose((0, 0), (0, 0), None, snap=True)[2] is None


def test_prediction_reaches_target_and_follows_waypoints():
    gains = Gains()
    path = core.predict_path((0, 0, 0), [(0.5, 0.0, 0.0)], gains)
    assert math.hypot(path[-1][0] - 0.5, path[-1][1]) < 0.05
    detour = core.predict_path((0, 0, 0), [(0.3, 0.3, 0.0), (0.6, 0.0, 0.0)], gains)
    assert max(p[1] for p in detour) > 0.25        # passes the waypoint side
    assert math.hypot(detour[-1][0] - 0.6, detour[-1][1]) < 0.05
    assert core.predict_path((0, 0, 0), [], gains) == []


def test_gains_from_controller_yaml():
    cfg = os.path.join(get_package_share_directory('pinky_soccer'), 'config',
                       'reactive_controller.yaml')
    gains = core.load_gains(cfg, {'max_lin': 0.08})
    assert gains.max_lin == 0.08 and gains.k_lin == pytest.approx(1.2)


def test_run_record_metrics(catalog):
    field = Field.from_file(None, 'yellow')
    goto = next(i for i in catalog['items'] if i['id'] == 'GoToPose')
    rec = core.RunRecord(1, goto, {'x': '0.5', 'y': '0', 'theta': '0'}, 10.0, field)
    for k in range(11):
        x = 0.05 * k
        rec.add(10.0 + 0.1 * k, (x, 0.0, 0.0), (x + 0.01, 0.0, 0.0), (0.3, 0.0), 0, None,
                (0.2, 0.0), ('GoToPose', (0.5, 0.0, 0.0)))
    rec.finish(11.0, 'success')
    m = dict(rec.metrics())
    assert m['결과'] == 'SUCCESS'
    assert m['소요 시간'] == '1.00 s'
    assert m['목표 위치 오차(추정)'] == '0.0 cm'
    assert m['목표 위치 오차(정답)'] == '1.0 cm'
    assert m['위치 추정 오차 평균/최대'] == '1.0 / 1.0 cm'
    assert m['득점'] == '아니오'
    rec.finish(12.0, 'failure')          # first terminal state wins
    assert rec.state == 'success'


def test_goal_detection_and_csv(tmp_path, catalog):
    field = Field.from_file(None, 'yellow')
    push = next(i for i in catalog['items'] if i['id'] == 'PushBall')
    rec = core.RunRecord(2, push, {'speed': '0.25', 'beyond': '0.1'}, 0.0, field)
    inside = field.goal_interior(field.opp_goal_color)
    rec.add(0.0, (0, 0, 0), None, (0.5, 0.0), 0, None, (0, 0), None)
    rec.add(0.5, (0, 0, 0), None, ((inside.xmin + inside.xmax) / 2, 0.0), 0, None, (0, 0), None)
    assert rec.goal_scored
    rec.finish(0.5, 'done_condition', 'goal_scored')
    already = core.RunRecord(3, push, {}, 0.0, field)
    for t in (0.0, 0.5):
        already.add(t, None, None, ((inside.xmin + inside.xmax) / 2, 0.0), 0, None, None, None)
    assert not already.goal_scored
    assert dict(already.metrics())['득점'] == '공이 처음부터 골대 안'
    path = rec.write(str(tmp_path))
    assert os.path.exists(path) and (tmp_path / 'runs.csv').exists()


def test_yaw_from_quaternion():
    q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))
    assert core.yaw_from_quaternion(q) == pytest.approx(0.8)
