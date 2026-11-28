import json
import time

from nav_msgs.msg import Odometry
from pinky_soccer_msgs.msg import BehaviorGoal, MotionResult
from pinky_soccer_viz.match_preparation import MatchPreparation
import pytest
import rclpy
from std_msgs.msg import Bool, String


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=221)
    n = MatchPreparation()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def localized(n, ready=True, records='new'):
    n.on_localization(String(data=json.dumps(dict(ready=ready, records=records, state='READY'))))


def test_idle_and_early_start_cannot_forward_strategy(node):
    goals, commands = [], []
    node.goal_pub.publish = goals.append
    node.command_pub.publish = commands.append
    node.on_playing(Bool(data=True))
    node.on_strategy(BehaviorGoal(mode=0, action='GoToPose'))
    node.tick()
    assert node.core.state == 'IDLE'
    assert all(g.mode == 2 for g in goals)
    assert all(m.linear.x == 0 and m.angular.z == 0 for m in commands)


def test_stale_session_late_result_and_stop_confirmation(node):
    localized(node, records='old')
    assert node.prepare()[0]
    node.calibrate_sent = node.calibration_ack = True
    localized(node, records='old')
    assert node.core.state == 'CALIBRATING'
    localized(node, records='new')
    assert node.core.state == 'MOVING_TO_START' and not node.core.prepared
    node.on_result(MotionResult(goal_id=node.goal_id-1, state=1))
    assert not node.core.arrived
    node.on_result(MotionResult(goal_id=node.goal_id, state=1))
    node.tick()
    assert node.core.state == 'MOVING_TO_START'
    odom = Odometry()
    node.on_odom(odom)
    node.settle_sec = 0
    node.tick()
    node.tick()
    assert node.core.state == 'READY'
    assert node.start()[0]
    node.loc_received = time.monotonic()-2
    node.tick()
    assert node.core.state == 'RECOVERING' and node.core.prepared
    assert not node.core.command_allowed
    localized(node)
    node.tick()
    assert node.core.state == 'PLAYING'


def test_motion_failure_never_ready(node):
    node.prepare()
    node.calibrate_sent = node.calibration_ack = True
    localized(node)
    node.on_result(MotionResult(goal_id=node.goal_id, state=2, reason='infeasible'))
    assert node.core.state == 'ERROR'
    assert 'infeasible' in node.core.reason


def test_preserves_each_command_sample_only_while_owned(node):
    from geometry_msgs.msg import Twist
    sent = []
    node.command_pub.publish = sent.append
    message = Twist()
    message.linear.x = .1
    node.on_command(message)
    assert not sent
    localized(node)
    node.core.state, node.core.prepared = 'PLAYING', True
    for _ in range(50):
        node.on_command(message)
    assert len(sent) == 50
    node.stop()
    count = len(sent)
    node.on_command(message)
    assert len(sent) == count


def test_kickoff_requires_preparation_and_does_not_repeat(node):
    node.on_kickoff(Bool(data=True))
    assert node.kickoff_pending is None
    localized(node)
    node.core.state, node.core.prepared = 'PLAYING', True
    node.on_kickoff(Bool(data=False))
    node.on_kickoff(Bool(data=True))
    assert node.kickoff_pending is not None
    node.on_playing(Bool(data=False))
    node.kickoff_pending = time.monotonic()-1
    node.tick()
    assert node.core.state == 'MOVING_TO_START'
    generation = node.goal_id
    node.on_kickoff(Bool(data=True))
    node.tick()
    assert node.goal_id == generation


def test_team_change_requires_stop_and_invalidates_ready(node):
    from std_srvs.srv import SetBool
    node.core.state, node.core.prepared = 'PLAYING', True
    assert not node.set_team(SetBool.Request(data=True), SetBool.Response()).success
    node.stop()
    reply = node.set_team(SetBool.Request(data=True), SetBool.Response())
    assert reply.success and node.core.team == 'blue'
    assert node.core.state == 'IDLE' and not node.core.prepared
    assert node.target[0] > 0


@pytest.mark.parametrize('stop_globally', [False, True])
def test_stop_cancels_localization_recovery_and_late_restore(node, stop_globally):
    localized(node)
    node.core.state, node.core.prepared = 'PLAYING', True
    node.loc_received = time.monotonic()-2
    node.tick()
    assert node.core.state == 'RECOVERING'
    assert node.core.led == ((0, 96, 0), True)
    if stop_globally:
        node.on_playing(Bool(data=False))
    else:
        node.stop()
    localized(node)
    node.tick()
    assert node.core.state == 'STOPPED' and node.core.resume_state is None


def test_move_waits_for_pose_without_consuming_move_deadline(node):
    node.core.state = 'MOVING_TO_START'
    node.goal_id = 123
    node.entered = time.monotonic()-10
    node.tick()
    assert node.core.state == 'RECOVERING'
    node.recovery_entered -= 120
    node.entered -= 120
    node.tick()
    assert node.core.state == 'RECOVERING'  # waiting itself is not a timeout
    node.on_result(MotionResult(goal_id=123, state=MotionResult.SUCCESS))
    assert not node.core.arrived  # late pre-pause result is ignored
    localized(node)
    node.tick()
    assert node.core.state == 'MOVING_TO_START' and not node.core.arrived
    assert time.monotonic()-node.entered < 11
