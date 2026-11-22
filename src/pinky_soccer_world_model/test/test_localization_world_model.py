"""A localization reseed must update decision pose and discard old map tracks."""

import time

import numpy as np
from pinky_soccer_world_model.world_model_node import WorldModelNode
import rclpy


def test_tf_reseed_invalidates_tracks_and_stale_localization_stops_game(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=['--ros-args', '-p', 'localization:=tf',
                    '-p', 'localization_pose_timeout:=2.0'], domain_id=190)
    node = WorldModelNode()
    try:
        node.pose = (.2, .1, 0.)
        node.ball.reset([.3, .1], node.now(), .01)
        node.grid.p.fill(.1)
        node.opponents, node.opp_time = [(.4, .1, .1)], node.now()
        rotation = np.diag([-1., -1., 1.])
        node.lookup = lambda *args: (rotation, np.array([-.2, -.1, 0.]))
        node.update_pose_from_tf()
        assert node.pose[:2] == (-.2, -.1)
        assert not node.ball.initialized
        assert np.all(node.grid.p == node.grid.prior)
        assert not node.opponents and node.opp_time is None
        node.playing = True
        node.last_odom_received = time.monotonic()
        node.last_localization_received = time.monotonic()-3
        sent = []
        node.pub.publish = sent.append
        node.tick()
        assert sent and not sent[-1].playing
        assert getattr(sent[-1], 'self').confidence == 0.
        node.last_localization_received = time.monotonic()
        node.tick()
        assert sent[-1].playing
    finally:
        node.destroy_node()
        rclpy.shutdown()
