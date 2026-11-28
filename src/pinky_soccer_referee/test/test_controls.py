"""Preparation controls: no vision pose or velocity command enters the robot stack."""
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from referee_controls import MatchControls, allowed, robot_namespaces


def state(phase='READY', ready=True, valid=True):
    return dict(state=phase, ready=ready, localization_valid=valid)


@pytest.mark.parametrize('phase,ready,valid,age,pending,expected', [
    ('READY', True, True, .1, False, True),
    ('STOPPED', True, True, .1, False, True),
    ('IDLE', False, False, .1, False, False),
    ('CALIBRATING', False, False, .1, False, False),
    ('MOVING_TO_START', False, True, .1, False, False),
    ('RECOVERING', True, True, .1, False, False),
    ('READY', True, False, .1, False, False),
    ('READY', True, True, 2., False, False),
    ('READY', True, True, .1, True, False),
])
def test_start_requires_fresh_ready(phase, ready, valid, age, pending, expected):
    assert allowed('start', [(state(phase, ready, valid), 10-age)], 10, pending) == expected
    assert allowed('stop', [], 10, pending)


def test_namespaces_and_all_robot_gate():
    assert robot_namespaces('/yellow/, blue,yellow') == ['yellow', 'blue']
    assert robot_namespaces('') == ['']
    with pytest.raises(ValueError):
        robot_namespaces('yellow; invalid')
    assert not allowed('start', [(state(), 10), (state('ERROR', False), 10)], 10)
    assert not allowed('prepare', [(state('PLAYING'), 10)], 10)
    assert allowed('retry', [(state('RECOVERING', True, False), 10)], 10)


class Future:
    def __init__(self):
        self.finished = self.canceled = False
        self.response = NS(success=True, message='accepted')
    def done(self):
        return self.finished
    def result(self):
        return self.response
    def cancel(self):
        self.canceled = True


class Client:
    def __init__(self):
        self.calls = []
        self.ready = True
    def service_is_ready(self):
        return self.ready
    def call_async(self, request):
        f = Future()
        self.calls.append(f)
        return f


def controls():
    pytest.importorskip('rclpy')
    clients = {}
    def create_client(kind, topic):
        clients[topic] = Client()
        return clients[topic]
    node = NS(create_client=create_client, create_subscription=lambda *a: None,
              create_timer=lambda *a: None)
    c = MatchControls(node, 'yellow,blue')
    c.status = {n: (state(), time.monotonic()) for n in c.names}
    return c, clients


def test_pending_request_blocks_duplicate_but_stop_interrupts():
    c, clients = controls()
    c.request('start')
    futures = [f for f, _ in c.pending.values()]
    assert len(futures) == 2
    c.request('start')
    assert len(clients['/yellow/preparation/start'].calls) == 1
    c.request('stop')
    assert all(f.canceled for f in futures)
    assert set(a for _, a in c.pending) == {'stop'}
    assert len(clients['/blue/preparation/stop'].calls) == 1


def test_partial_unavailable_service_never_starts_other_robot():
    c, clients = controls()
    clients['/blue/preparation/start'].ready = False
    c.request('start')
    assert not clients['/yellow/preparation/start'].calls


def test_stop_is_reasserted_if_an_earlier_start_takes_effect_late():
    c, clients = controls()
    c.request('stop')
    for future, _ in c.pending.values():
        future.finished = True
    c.status = {n: (state('PLAYING'), time.monotonic()) for n in c.names}
    c.last_stop -= 1.
    c.poll()
    assert len(clients['/yellow/preparation/stop'].calls) == 2


def test_rejected_or_timeout_request_stops_all_robots():
    for timeout in (False, True):
        c, clients = controls()
        c.status = {n: (state('IDLE', False, False), time.monotonic()) for n in c.names}
        c.request('prepare')
        key = ('yellow', 'prepare')
        f, t = c.pending[key]
        if timeout:
            c.pending[key] = f, t-4
        else:
            f.finished = True
            f.response = NS(success=False, message='robot rejected')
        c.poll()
        assert all(clients['/'+n+'/preparation/stop'].calls for n in c.names)


def test_controls_only_append_footer_and_mouse_uses_same_gate():
    c, clients = controls()
    original = np.full((480, 960, 3), 77, np.uint8)
    image = c.render(original)
    assert np.array_equal(image[:480], original)
    c.status.clear()
    _, box = c.buttons[1]
    c.mouse(4, box[0]+1, box[1]+1, 0, None)  # OpenCV EVENT_LBUTTONUP
    assert not clients['/yellow/preparation/start'].calls


def test_native_ros_service_bridge_never_sends_velocity(tmp_path, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    import json

    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    rclpy.init(domain_id=189)
    robot = Node('stub_preparation')
    ui = Node('referee_control_test')
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(ui)
    calls = []
    services = []
    def reply(action, request, response):
        calls.append(action)
        response.success, response.message = True, action+' accepted'
        return response
    for action in MatchControls.ACTIONS:
        services.append(robot.create_service(Trigger, '/preparation/'+action,
            lambda req, res, action=action: reply(action, req, res)))
    pub = robot.create_publisher(String, '/preparation/status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    c = MatchControls(ui, '')
    def pump(condition):
        end = time.monotonic()+4
        while not condition() and time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
        assert condition()
    try:
        c.request('start')
        assert calls == []
        pub.publish(String(data=json.dumps(state('IDLE', False, False))))
        pump(lambda: '' in c.status and c.action_clients['']['prepare'].service_is_ready())
        c.request('prepare')
        pump(lambda: 'prepare' in calls and not c.pending)
        c.request('start')
        assert 'start' not in calls
        pub.publish(String(data=json.dumps(state())))
        pump(lambda: c.enabled('start'))
        c.request('start')
        pump(lambda: 'start' in calls and not c.pending)
        c.request('stop')
        pump(lambda: 'stop' in calls)
        assert not ui.get_publishers_info_by_topic('/cmd_vel')
        assert not ui.get_publishers_info_by_topic('/game/playing')
    finally:
        executor.shutdown()
        ui.destroy_node()
        robot.destroy_node()
        rclpy.shutdown()
