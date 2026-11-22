"""Unit tests for the ROS-free world model parts."""

import math
from pathlib import Path

import numpy as np
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_world_model.ball_filter import (BallFilter, BallFilterParams, ESTIMATED,
                                                  LOST, OBSERVED)
from pinky_soccer_world_model.geometry import (compose2d, detect_color_blob,
                                               intersect_plane_z, inverse2d, pixel_ray,
                                               quat_to_matrix)
from pinky_soccer_world_model.opponents import cluster_opponents, scan_points
from pinky_soccer_world_model.search_grid import SearchGrid
import pytest

SPEC = Path(__file__).resolve().parents[2] / 'pinky_soccer_sim' / 'config' / 'rcj_field.yaml'


@pytest.fixture(scope='module')
def field():
    return Field.from_file(SPEC)


def test_filter_states_and_growing_uncertainty(field):
    f = BallFilter(BallFilterParams(), field.inner_walls)
    assert f.state(0.0) == LOST
    for i in range(10):
        f.update((0.5, 0.2), i * 0.033, distance=0.5)
    t = 0.3
    f.predict(t)
    assert f.state(t) == OBSERVED
    s0 = f.max_sigma()
    f.predict(1.5)
    assert f.state(1.5) == ESTIMATED
    assert f.max_sigma() > s0
    f.predict(20.0)
    assert f.state(20.0) == LOST


def test_filter_tracks_rolling_ball_with_friction(field):
    p = BallFilterParams(friction_tau=1.0)
    f = BallFilter(p, field.inner_walls)
    # Truth: v(t) = v0 exp(-t/tau), x(t) = x0 + v0 tau (1 - exp(-t/tau)).
    v0, x0 = 0.6, -0.5
    for k in range(30):
        t = k * 0.05
        f.update((x0 + v0 * (1 - math.exp(-t)), 0.0), t, distance=0.3)
    f.predict(3.0)
    expected = x0 + v0 * (1 - math.exp(-3.0))
    assert f.x[0] == pytest.approx(expected, abs=0.05)
    assert abs(f.x[2]) < 0.1


def test_filter_clamps_inside_walls(field):
    f = BallFilter(BallFilterParams(friction_tau=10.0), field.inner_walls)
    for k in range(10):
        t = k * 0.05
        f.update((1.0 + 1.0 * t, 0.5), t, distance=0.3)
    f.predict(5.0)
    assert f.x[0] <= field.inner_walls.xmax - f.radius + 1e-9
    assert f.x[2] == 0.0


def test_filter_rejects_single_outlier_but_reacquires(field):
    f = BallFilter(BallFilterParams(), field.inner_walls)
    for k in range(10):
        f.update((0.0, 0.0), k * 0.03, distance=0.3)
    assert not f.update((0.8, 0.5), 0.33, distance=0.3)
    assert not f.update((0.8, 0.5), 0.36, distance=0.3)
    assert f.update((0.8, 0.5), 0.39, distance=0.3)   # third consistent jump re-acquires
    assert f.x[0] == pytest.approx(0.8)


def test_covariance3_layout(field):
    f = BallFilter(BallFilterParams(), field.inner_walls)
    f.update((0.0, 0.0), 0.0, distance=1.0)
    cov = f.covariance3()
    assert len(cov) == 9 and cov[0] == pytest.approx(f.P[0, 0]) and cov[8] > 1e3


def test_search_grid_lowers_seen_cells_and_relaxes(field):
    g = SearchGrid(field.inner_walls, 0.05)
    mask = g.fov_mask(0.0, 0.0, 0.0, math.radians(60), 0.15, 1.0)
    assert mask.any()
    j, i = np.argwhere(mask)[0]
    assert g.cx[j, i] > 0                                   # in front of the robot
    behind = g.fov_mask(0.0, 0.0, math.pi, math.radians(60), 0.15, 1.0)
    assert not (mask & behind).any()
    g.observe(mask, 1.0)
    assert g.p[mask].max() < 0.5 and g.p[~mask].min() == pytest.approx(0.5)
    g.relax(100.0)
    assert g.p[mask].min() == pytest.approx(0.5, abs=1e-3)
    g.observe(mask, 0.0, (0.5, 0.0))
    assert g.best_cell() == pytest.approx((0.5, 0.0), abs=0.05)
    assert g.as_int8().max() >= 95   # peak sits between cell centres


def test_pose_algebra():
    a = (1.0, 2.0, 0.5)
    ident = compose2d(a, inverse2d(a))
    assert ident == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_ray_hits_floor_at_ball_height():
    # Camera 0.1 m above the floor, optical z forward = robot +x, looking 30 deg down.
    pitch = math.radians(30)
    # optical (x right, y down, z fwd) -> body (x fwd, y left, z up), then pitch down.
    base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    c, s = math.cos(pitch), math.sin(pitch)
    tilt = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    R = tilt @ base
    K = [500, 0, 320, 0, 500, 240, 0, 0, 1]
    hit = intersect_plane_z(np.array([0, 0, 0.1]), R @ pixel_ray(320, 240, K), 0.02)
    assert hit[0] == pytest.approx(0.08 / math.tan(pitch), rel=1e-6)
    assert hit[1] == pytest.approx(0.0, abs=1e-9)
    assert intersect_plane_z(np.zeros(3), np.array([1.0, 0, 0.1]), 0.0) is None


def test_quaternion_matrix_yaw():
    R = quat_to_matrix(0, 0, math.sin(0.25), math.cos(0.25))
    assert R[0, 0] == pytest.approx(math.cos(0.5)) and R[1, 0] == pytest.approx(math.sin(0.5))


def test_detect_orange_blob():
    pytest.importorskip('cv2')
    import cv2
    img = np.full((120, 160, 3), (40, 140, 40), np.uint8)       # green carpet (BGR)
    cv2.circle(img, (100, 70), 8, (0, 77, 255), -1)              # rgba 1 0.30 0 -> BGR
    blob = detect_color_blob(img, [((5, 120, 80), (25, 255, 255))])
    assert blob is not None
    assert blob[0] == pytest.approx(100, abs=1) and blob[1] == pytest.approx(70, abs=1)
    assert detect_color_blob(np.full_like(img, 40), [((5, 120, 80), (25, 255, 255))]) is None


def test_opponent_clusters_exclude_walls(field):
    walls = field.inner_walls
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    ranges = []
    for a in angles:
        # Robot at the centre: distance to the rectangle walls along each beam.
        dx = walls.xmax / abs(math.cos(a)) if abs(math.cos(a)) > 1e-9 else math.inf
        dy = walls.ymax / abs(math.sin(a)) if abs(math.sin(a)) > 1e-9 else math.inf
        r = min(dx, dy)
        if abs(a) < 0.08:                    # a 0.1 m wide object at 0.6 m straight ahead
            r = 0.6
        ranges.append(r)
    pts = scan_points(ranges, -math.pi, 2 * math.pi / 720, 0.05, 12.0)
    found = cluster_opponents(pts, walls)
    assert len(found) == 1
    assert found[0][0] == pytest.approx(0.6, abs=0.02)
    assert found[0][1] == pytest.approx(0.0, abs=0.02)
