"""The blocked-start planner's reverse leg must reach the wheel command."""
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
from pinky_soccer_viz.attack_controller import AttackController
from pinky_soccer_viz.attack_motion import Grid


@pytest.mark.parametrize('mpc', [False, True])
def test_escape_direction_and_release_at_next_waypoint(tmp_path, monkeypatch, mpc):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'roslogs'))
    rclpy.init(args=[], domain_id=197)
    node = AttackController()
    try:
        calls = []
        if mpc:
            from types import SimpleNamespace
            node.path_mpc = SimpleNamespace(reset=lambda: None, status='test solver',
                command=lambda *args: calls.append(args) or np.zeros(2))
        now = node.get_clock().now().nanoseconds/1e9
        node.goal = BehaviorGoal(action='GoBehindBall')
        node.goal.mode = BehaviorGoal.REACTIVE
        node.goal.target.x, node.goal.target.y = .616, .277
        node.goal_at = node.world_at = node.scan_at = node.wheel_at = now
        node.world = WorldModel()
        node.world.self.pose.x = .763
        node.world.self.confidence = 1.
        node.world.self.state = node.world.self.OBSERVED
        node.world.ball.state = node.world.ball.LOST
        node.wheels = np.zeros(2)
        node.scan = LaserScan()
        node.scan.header.stamp = node.get_clock().now().to_msg()
        node.scan_points = lambda: np.empty((0, 2))
        node.obstacles = lambda points, pushing: points
        node.safe = lambda *args: True
        node.publish_path = lambda: None
        node.report = lambda *args: None
        sent = []
        node.send = sent.append
        monkeypatch.setattr(Grid, 'clear', lambda self, p, extra=0.: p[0] < .74)
        monkeypatch.setattr(Grid, 'line', lambda *args: True)
        monkeypatch.setattr(Grid, 'plan', lambda self, start, target:
                            [start, target] if self.clear(start) else [])
        node.tick()
        assert node.escape_leg and node.travel_reverse
        assert np.all(sent[-1] < 0.)
        assert not calls  # A forward-only solver cannot execute this escape.
        node.world.self.pose.x, node.world.self.pose.y = node.path[1]
        node.last_tick = None
        node.tick()
        assert not node.escape_leg
        assert node.travel_reverse is False
        assert len(calls) == int(mpc)
        # A stalled approach enters recovery after one second in either strategy.
        node.progress_pose = np.array([node.world.self.pose.x, node.world.self.pose.y, 0.])
        node.progress_at = node.now()-1.1
        node.last_tick = None
        recovered = []
        node.recover = lambda *args: recovered.append(args)
        node.tick()
        assert len(recovered) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('action', ['GoBehindBall', 'PushBall', 'Backoff'])
def test_rollback_retains_forward_approach_and_explicit_backoff(tmp_path, monkeypatch, action):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'roslogs'))
    rclpy.init(args=[], domain_id=197)
    node = AttackController()
    try:
        now=node.now()
        node.goal=BehaviorGoal(action=action,mode=BehaviorGoal.REACTIVE)
        node.goal.target.x=-.25
        node.goal_at=node.world_at=node.scan_at=node.wheel_at=now
        node.world=WorldModel()
        node.world.self.state=node.world.self.OBSERVED
        node.world.self.confidence=1.
        node.world.ball.state=node.world.ball.LOST
        node.wheels=np.zeros(2)
        node.scan=LaserScan();node.scan.header.stamp=node.get_clock().now().to_msg()
        node.scan_points=lambda:np.empty((0,2))
        node.obstacles=lambda points,pushing:points
        node.safe=lambda *args:True
        node.publish_path=lambda:None
        node.report=lambda *args:None
        sent=[];node.send=sent.append
        node.tick()
        assert node.travel_reverse == (action=='Backoff')
        if action=='Backoff':assert np.all(sent[-1]<0.)
        else:assert sent[-1][0]*sent[-1][1]<0.
    finally:
        node.destroy_node();rclpy.shutdown()
