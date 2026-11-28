"""Exercise the Hybrid publication path with NumPy-derived contact geometry."""
import json

import rclpy

from pinky_soccer_world_model.world_model_node import WorldModelNode


def test_hybrid_contact_publication_is_json_boolean(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'roslogs'))
    rclpy.init(args=['--ros-args', '-p', 'ball_source:=hybrid'], domain_id=197)
    node = WorldModelNode()
    try:
        node.pose = (0., 0., 0.)
        node.now = lambda: 10.
        sent, worlds = [], []
        node.contact_pub.publish = lambda msg: sent.append(json.loads(msg.data))
        node.pub.publish = worlds.append
        for stamp, xy, expected in ((9.98, [.1, 0.], True),
                                    (9.99, [.2, 0.], False),
                                    (10., [.1, .1], False)):
            node.ball.receive_referee(stamp, xy, [0., 0.])
            node.tick()
            assert sent[-1]['present'] is expected
            assert sent[-1]['input_mode'] == 'hybrid'
        assert len(worlds) == 3
    finally:
        node.destroy_node()
        rclpy.shutdown()
