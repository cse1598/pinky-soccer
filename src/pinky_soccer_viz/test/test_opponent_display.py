"""Display movement, never infer heading from a sparse scan."""
import math

import numpy as np
from pinky_soccer_msgs.msg import TrackedObject
from pinky_soccer_viz import style


def obj(vx=0., vy=0., state=0):
    o = TrackedObject()
    o.velocity.x, o.velocity.y = vx, vy
    o.confidence, o.state = 1., state
    return o


def test_fixed_geometry_envelope_separate_from_collision_radius():
    from pinky_soccer_mpc.collision_geometry import FOOTPRINT_RECTS
    assert style.OPPONENT_RADIUS == max(math.hypot(x, y)
        for x0, y0, x1, y1 in FOOTPRINT_RECTS for x in (x0, x1) for y in (y0, y1))
    assert .08 < style.OPPONENT_RADIUS < .10


def test_speed_length_reverse_threshold_and_heading_independence():
    o = obj(.1, 0.)
    np.testing.assert_allclose(style.opponent_velocity(o), [.05, 0.])
    o.pose.theta = 2.
    np.testing.assert_allclose(style.opponent_velocity(o), [.05, 0.])
    np.testing.assert_allclose(style.opponent_velocity(obj(-.2, 0.)), [-.1, 0.])
    assert style.opponent_velocity(obj(.019)) is None
    assert style.opponent_velocity(obj()) is None
    assert style.opponent_velocity(obj(.2, state=2)) is None
    assert np.linalg.norm(style.opponent_velocity(obj(2.))) == .35


def test_topview_lost_hidden_heading_has_no_effect():
    from pathlib import Path
    from pinky_soccer_field.field_geometry import Field
    from pinky_soccer_viz.topview import TopView
    field = Field.from_file(Path(__file__).resolve().parents[2] /
                            'pinky_soccer_sim/config/hardboard_field.yaml')
    view = TopView(field)
    image = view.background.copy()
    o = obj(.2)
    view._draw_opponent(image, o, 10.)
    image2 = view.background.copy()
    o.pose.theta = -2.5
    view._draw_opponent(image2, o, 10.)
    np.testing.assert_array_equal(image, image2)
    lost = view.background.copy()
    o.state = 2
    view._draw_opponent(lost, o, 10.)
    np.testing.assert_array_equal(lost, view.background)


def test_rviz_circle_velocity_and_lost_cleanup(tmp_path, monkeypatch):
    import rclpy
    from pinky_soccer_msgs.msg import WorldModel
    from pinky_soccer_viz.world_model_viz import WorldModelViz
    from visualization_msgs.msg import Marker
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=222)
    node = WorldModelViz()
    try:
        sent = []
        node.pub.publish = sent.append
        w = WorldModel()
        w.ball.state = 2
        w.opponents = [obj(-.1)]
        w.opponents[0].pose.theta = 2.1
        node.on_wm(w)
        circles = [m for m in sent[-1].markers if m.ns == 'opponents']
        arrows = [m for m in sent[-1].markers if m.ns == 'opponent_velocity']
        assert circles[0].type == Marker.CYLINDER
        assert circles[0].scale.x == 2*style.OPPONENT_RADIUS
        assert len(arrows) == 1
        assert arrows[0].points[1].x == -.05
        assert arrows[0].points[1].y == 0.
        assert arrows[0].color.g > circles[0].color.g  # visible inside the body circle
        w.opponents[0].state = 2
        node.on_wm(w)
        assert sent[-1].markers[0].action == Marker.DELETEALL
        assert not any(m.ns in ('opponents', 'opponent_velocity') for m in sent[-1].markers)
    finally:
        node.destroy_node()
        rclpy.shutdown()
