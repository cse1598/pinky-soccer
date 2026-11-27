"""Exclusive referee observations must never mix with robot-camera estimates."""
import math
import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from pinky_soccer_msgs.msg import TrackedObject
from pinky_soccer_referee.msg import RefereeState
from pinky_soccer_world_model.world_model_node import WorldModelNode
from std_msgs.msg import String


@pytest.fixture
def node(tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'roslogs'))
    rclpy.init(args=['--ros-args','-p','ball_source:=referee'],domain_id=197)
    n=WorldModelNode();n.pose=(0.,0.,0.)
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def observation(n):
    m=RefereeState();m.header.stamp=n.get_clock().now().to_msg()
    m.calibrated=m.metric_valid=m.ball.visible=m.ball.velocity_valid=True
    m.ball.position.x=.26783;m.ball.position.y=-.12279
    m.ball.velocity.x=.21;m.ball.velocity.y=-.03;m.ball.tracking_confidence=.9
    return m


def test_direct_xy_survives_local_camera_and_is_not_predicted(node):
    m=observation(node);node.on_referee_ball(m)
    # Even a previously initialized local filter must not affect this output.
    node.ball.reset([-.6,.4],node.now(),.01)
    node.ball.predict=lambda *args:pytest.fail('exclusive referee mode predicted a local estimate')
    node.lookup=lambda *args:pytest.fail('exclusive mode used local camera TF')
    local=PointStamped();local.point.x=-.5;node.on_ball(local)
    node.on_ball_evidence(String(data='{"bearing_valid":true}'))
    published=[];node.pub.publish=published.append
    node.tick();node.tick()
    for msg in published:
        assert msg.ball.pose.x==m.ball.position.x
        assert msg.ball.pose.y==m.ball.position.y
        assert msg.ball.velocity.x==m.ball.velocity.x
        assert msg.ball.state==TrackedObject.OBSERVED
    assert len(published)==2
    assert node.ball_contact['present'] is False


def test_stale_invalid_and_unseen_never_fall_back(node):
    m=observation(node);node.on_referee_ball(m)
    stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    assert node.referee_ball_object(stamp+.2).state==TrackedObject.OBSERVED
    lost=node.referee_ball_object(stamp+.5)
    assert lost.state==TrackedObject.LOST and lost.confidence==0.
    assert lost.velocity.x==0. and lost.pose.x==m.ball.position.x
    m.ball.visible=False;node.on_referee_ball(m)
    assert node.referee_ball_object(node.now()).state==TrackedObject.LOST
    m.ball.visible=True;m.ball.position.x=math.nan;node.on_referee_ball(m)
    assert node.referee_ball_object(node.now()).state==TrackedObject.LOST


@pytest.fixture
def logic_node():
    from types import SimpleNamespace
    import numpy as np
    n=SimpleNamespace(ball_input='referee',last_referee_state_stamp=-math.inf,
        last_local_ball=2.,referee_observation=None,referee_observation_valid=False,
        now=lambda:2.,ball_source='none',ball=SimpleNamespace(p=SimpleNamespace(meas_sigma_base=.02)),
        grid=SimpleNamespace(p=np.zeros((1,1)),observe=lambda *a:None))
    return n


def raw_observation():
    from types import SimpleNamespace as S
    return S(header=S(stamp=S(sec=2,nanosec=0)),calibrated=True,metric_valid=True,
        ball=S(visible=True,position=S(x=.25,y=-.12),velocity=S(x=.2,y=.1),
               velocity_valid=True,tracking_confidence=.9))


def test_exclusive_callback_ignores_recent_local_fix_and_accepts_abrupt_referee_jump(logic_node):
    n=logic_node;m=raw_observation()
    WorldModelNode.on_referee_ball(n,m)
    assert WorldModelNode.referee_ball_object(n,2.).pose.x==.25
    m.header.stamp.nanosec=100000000;n.now=lambda:2.1
    m.ball.position.x=-.46;m.ball.position.y=.21
    WorldModelNode.on_referee_ball(n,m)
    b=WorldModelNode.referee_ball_object(n,2.2)
    assert (b.pose.x,b.pose.y)==(-.46,.21)
    assert b.state==TrackedObject.OBSERVED
    # Local input must return before touching transforms or filter state.
    n.lookup=lambda *args:pytest.fail('local camera used in referee-only mode')
    WorldModelNode.on_ball(n,PointStamped())
    WorldModelNode.on_ball_evidence(n,String(data='not even valid JSON'))
    assert WorldModelNode.referee_ball_object(n,2.2).pose.x==-.46


def test_exclusive_logic_marks_lost_without_predicting_or_falling_back(logic_node):
    n=logic_node;m=raw_observation();WorldModelNode.on_referee_ball(n,m)
    b=WorldModelNode.referee_ball_object(n,2.5)
    assert b.state==TrackedObject.LOST and b.confidence==0.
    assert (b.pose.x,b.pose.y)==(.25,-.12) and b.velocity.x==0.
    m.header.stamp.nanosec=100000000;n.now=lambda:2.1;m.ball.visible=False
    WorldModelNode.on_referee_ball(n,m)
    m.header.stamp.nanosec=0;m.ball.visible=True
    WorldModelNode.on_referee_ball(n,m)  # older valid message cannot undo newer loss
    assert WorldModelNode.referee_ball_object(n,2.1).state==TrackedObject.LOST
