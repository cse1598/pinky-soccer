"""Pure-Python tests of geometry, prediction, motion and planning (no simulator)."""

import math

from pinky_soccer_skills import params, prediction
from pinky_soccer_skills.context import BallEstimate
from pinky_soccer_skills.geometry import Polyline, Pose, round_corners
from pinky_soccer_skills.motion import (candidate_paths, Limiter, plan_around_ball, rollout,
                                        side_clearance)
from pinky_soccer_skills.planning import avoid_path

P = params.load()


def test_polyline_projection_sign():
    path = Polyline([(0, 0), (1, 0)])
    s, lat, h = path.project((0.4, 0.1))
    assert abs(s - 0.4) < 1e-9 and abs(lat - 0.1) < 1e-9 and abs(h) < 1e-9
    assert path.project((0.4, -0.1))[1] < 0


def test_round_corners_keeps_ends_and_shortens():
    pts = [(0, 0), (1, 0), (1, 1)]
    out = round_corners(pts, 0.3)
    assert out[0] == (0, 0) and out[-1] == (1, 1)
    assert Polyline(out).length < 2.0


def test_constant_decel_prediction():
    b = BallEstimate(0, 0, 1.0, 0.0)
    assert abs(prediction.reach(1.0, 0.35) - 1.0 / 0.7) < 1e-9
    x, _ = prediction.predict(b, 100.0, 0.35)
    assert abs(x - 1.0 / 0.7) < 1e-9
    t, y = prediction.crossing_x(b, 0.5, 5.0, 0.35)
    assert abs(prediction.travel(1.0, t, 0.35) - 0.5) < 1e-9 and y == 0.0
    assert prediction.crossing_x(b, 2.0, 10.0, 0.35) is None   # stops before


def test_limiter_keeps_curvature():
    lim = Limiter(P.robot, 1.0)
    v, w = lim.saturate(1.0, 4.0)
    assert abs(w / v - 4.0) < 1e-9 and abs(v) <= P.robot.max_lin + 1e-9


def test_drive_reaches_pose_including_reverse():
    for goal in (Pose(1.0, 0.0, 0.0), Pose(-0.3, 0.0, 0.0), Pose(0.0, 0.3, 0.0)):
        res = rollout(Pose(), [goal], P.robot, P.drive)
        x, y, th = res['path'][-1]
        assert res['ok']
        assert math.hypot(x - goal.x, y - goal.y) < 0.03
    back = rollout(Pose(), [Pose(-0.3, 0.0, 0.0)], P.robot, P.drive)
    assert back['time'] < 2.0          # straight reverse, no U-turn


def test_side_clearance_allows_face_contact_only():
    r = P.robot
    front = (r.contact_dist + 0.001, 0.0)
    assert side_clearance(Pose(), front, r) > 0
    assert side_clearance(Pose(), (0.0, 0.08), r) < 0.03   # beside a wheel


def test_planner_goes_around_ball_without_touching():
    ball = (0.15, 0.0)
    target = Pose(ball[0] - P.robot.contact_dist - 0.03, 0.0, 0.0)
    keep = P.robot.ball_radius + math.hypot(P.robot.footprint[0], P.robot.footprint[3]) + 0.03
    name, wps, res = plan_around_ball(Pose(0.35, 0.0, math.pi), target, ball, P.robot,
                                      P.drive, 0.03, keep)
    assert res['ok'] and res['clearance'] >= 0.03, name
    assert 'line_8' in candidate_paths(Pose(), target, ball, P.robot, keep)


def test_avoid_path_detours_obstacle():
    path = avoid_path((0, 0), (1, 0), [(0.5, 0.0)], 0.2)
    assert len(path) == 3 and abs(path[1][1]) >= 0.2
