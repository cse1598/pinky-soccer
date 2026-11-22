"""Render the top view and markers from synthetic messages."""

import math
import os
from pathlib import Path

import numpy as np
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import BehaviorGoal, TrackedObject, WorldModel
from pinky_soccer_viz import style
from pinky_soccer_viz.topview import PANEL_WIDTH, TopView
from pinky_soccer_viz.world_model_viz import field_markers
import pytest

SPEC = Path(__file__).resolve().parents[2] / 'pinky_soccer_sim' / 'config' / 'rcj_field.yaml'


@pytest.fixture(scope='module')
def field():
    return Field.from_file(SPEC)


def world(ball_state):
    wm = WorldModel()
    wm.header.stamp.sec = 10
    wm.self.pose.x, wm.self.pose.theta = -0.3, 0.3
    wm.ball.pose.x, wm.ball.pose.y = 0.4, 0.2
    wm.ball.velocity.x = 0.3
    wm.ball.state = ball_state
    wm.ball.covariance = [0.01, 0.002, 0, 0.002, 0.004, 0, 0, 0, 1e6]
    wm.ball.last_seen.sec = 8
    opp = TrackedObject()
    opp.pose.x, opp.pose.y, opp.state = 0.7, -0.3, TrackedObject.ESTIMATED
    opp.last_seen.sec = 9
    wm.opponents = [opp]
    wm.ball_search_grid.info.resolution = 0.05
    wm.ball_search_grid.info.width, wm.ball_search_grid.info.height = 49, 37
    wm.ball_search_grid.info.origin.position.x = -1.215
    wm.ball_search_grid.info.origin.position.y = -0.91
    wm.ball_search_grid.data = list(np.linspace(0, 100, 49 * 37).astype(int))
    return wm


@pytest.mark.parametrize('state', [0, 1, 2])
def test_render_all_ball_states(field, tmp_path, state):
    view = TopView(field, 150.0)
    goal = BehaviorGoal(branch='ATTACK', action='PushBall', mode=1)
    goal.target.x = 1.2
    img = view.render(world(state), goal, {'ball': (0.42, 0.2), 'robot': (-0.3, 0.0)}, 29.9)
    assert img.shape[0] >= view.h and img.shape[1:] == (view.w + PANEL_WIDTH, 3)
    assert img[view.px(0.4, 0.2)[1], view.px(0.4, 0.2)[0]].any()
    out = os.environ.get('TOPVIEW_TEST_PNG')
    if out:
        import cv2
        cv2.imwrite(f'{out}_{state}.png', img)


def test_render_without_messages(field):
    img = TopView(field, 100.0).render()
    assert img.ndim == 3


def test_ball_pixel_is_orange_when_observed(field):
    view = TopView(field, 200.0)
    img = view.render(world(0))
    u, v = view.px(0.4, 0.2)
    b, g, r = img[v, u]
    assert r > 200 and b < 80


def test_covariance_ellipse_axes():
    major, minor, angle = style.covariance_ellipse([0.04, 0, 0, 0, 0.01, 0, 0, 0, 0])
    assert major == pytest.approx(0.4) and minor == pytest.approx(0.2)
    assert abs(math.sin(angle)) == pytest.approx(0.0, abs=1e-9)


def test_field_markers_cover_field(field):
    from rclpy.serialization import serialize_message
    msg = field_markers(field)
    serialize_message(msg)          # catches int-in-float-field assertions
    markers = msg.markers
    namespaces = {m.ns for m in markers}
    assert {'field', 'walls', 'lines', 'goals'} <= namespaces
    assert len({m.id for m in markers}) == len(markers)


def test_dynamic_markers_serialize(field):
    import rclpy
    from rclpy.serialization import serialize_message
    from pinky_soccer_viz.world_model_viz import WorldModelViz
    from visualization_msgs.msg import MarkerArray
    rclpy.init()
    try:
        node = WorldModelViz()
        sent = []
        node.pub.publish = sent.append
        node.goal = BehaviorGoal(branch='ATTACK', action='PushBall', mode=1)
        for state in (0, 1, 2):
            node.on_wm(world(state))
        assert len(sent) == 3 and all(isinstance(m, MarkerArray) for m in sent)
        for m in sent:
            serialize_message(m)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_search_grid_heat_is_visible(field):
    from pinky_soccer_field.field_geometry import occupancy_grid_msg
    view = TopView(field, 200.0)
    wm = WorldModel()
    grid = np.full((37, 49), 100, np.int8)
    wm.ball_search_grid = occupancy_grid_msg(grid, 0.05, (-1.215, -0.91))
    u, v = view.px(0.5, 0.5)
    hot = view.render(wm)[v, u].astype(int)
    assert np.abs(hot - view.background[v, u].astype(int)).sum() > 100


def test_search_grid_markers_preserve_origin_and_unknown_cells():
    from nav_msgs.msg import OccupancyGrid
    from pinky_soccer_viz.world_model_viz import search_grid_markers
    from rclpy.serialization import serialize_message
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width, grid.info.height, grid.info.resolution = 2, 2, 0.1
    grid.info.origin.position.x, grid.info.origin.position.y = -0.8, -0.5
    grid.info.origin.orientation.z, grid.info.origin.orientation.w = 0.6, 0.8
    grid.data = [0, -1, 50, 100]
    result = search_grid_markers(grid)
    serialize_message(result)
    m = result.markers[0]
    assert m.header == grid.header
    assert m.pose.orientation == grid.info.origin.orientation
    assert m.pose.position.x == -0.8 and m.pose.position.y == -0.5
    assert grid.info.origin.position.z == 0.0  # source message is not modified
    assert len(m.points) == len(m.colors) == 3
    assert m.points[-1].x == pytest.approx(0.15)
    assert m.points[-1].y == pytest.approx(0.15)
    assert m.colors[0].b == 1.0 and m.colors[-1].r == 1.0
