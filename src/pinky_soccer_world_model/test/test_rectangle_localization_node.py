"""Synthetic sensor integration in an isolated ROS domain; never drives hardware."""
import math
import time
from types import SimpleNamespace

import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from pinky_interfaces.srv import SetLed
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, LaserScan
from std_srvs.srv import Trigger

from pinky_soccer_world_model import rectangle_localization_node as module
from pinky_soccer_world_model.geometry import compose2d, inverse2d, wrap


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=['--ros-args', '-p', 'heading_record:='+str(tmp_path/'heading.yaml'),
                     '-p', 'led_enabled:=false'], domain_id=187)
    node = module.RectangleLocalization()
    clock = SimpleNamespace(t=1000.)
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock.t,
                                                      perf_counter=time.perf_counter))
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=clock.t))
    camera = np.eye(4)
    camera[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    node.transform = lambda source: camera if source == 'optical' else np.eye(4)
    node.info = CameraInfo(width=640, height=480,
        k=[500., 0., 320., 0., 500., 240., 0., 0., 1.])
    # Constructor's actual monotonic timestamps must not affect the synthetic clock.
    node.last_fit_attempt = node.last_status = node.last_map_publish = -math.inf
    node.imu_received = node.fit_received = node.image_received = -math.inf
    yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def feed(node, clock, theta, gyro=0., goal=True, dt=.2):
    clock.t += dt
    stamp = Time(seconds=clock.t).to_msg()
    pose = (.2, .1, theta)
    imu = Imu()
    imu.header.stamp, imu.header.frame_id = stamp, 'imu'
    imu.orientation.z, imu.orientation.w = math.sin((theta+.42)/2), math.cos((theta+.42)/2)
    imu.angular_velocity.z = gyro
    node.on_imu(imu)
    odom = Odometry()
    odom.header.stamp = stamp
    odom_pose = compose2d(inverse2d((.3, .1, .7)), pose)
    odom.pose.pose.position.x, odom.pose.pose.position.y = odom_pose[:2]
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(odom_pose[2]/2), math.cos(odom_pose[2]/2)
    node.on_odom(odom)
    scan = LaserScan()
    scan.header.stamp, scan.header.frame_id = stamp, 'scan'
    scan.range_min, scan.range_max = .035, 12.
    scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/720
    angles = theta+scan.angle_min+np.arange(720)*scan.angle_increment
    dx, dy = np.cos(angles), np.sin(angles)
    with np.errstate(divide='ignore', invalid='ignore'):
        hits = np.array([(x-pose[0])/dx for x in (-node.length/2, node.length/2)]+
                        [(y-pose[1])/dy for y in (-node.width/2, node.width/2)])
    hits[hits <= 0] = np.inf
    scan.ranges = np.min(hits, axis=0).astype(float).tolist()
    node.on_scan(scan)
    frame = np.zeros((480, 640, 3), np.uint8)
    if goal:
        for color in ('blue', 'yellow'):
            a, b = node.field.goal_back(color)
            bearing = wrap(math.atan2((a[1]+b[1])/2-pose[1], (a[0]+b[0])/2-pose[0])-theta)
            if abs(bearing) < .25:
                u = round(320-500*math.tan(bearing))
                cv2.rectangle(frame, (u-50, 210), (u+50, 240),
                              (134, 75, 5) if color == 'blue' else (4, 229, 237), -1)
    _, jpeg = cv2.imencode('.jpg', frame)
    image = CompressedImage(format='jpeg', data=jpeg.tobytes())
    image.header.stamp, image.header.frame_id = stamp, 'optical'
    node.on_image(image)
    node.tick()


def test_one_goal_initializes_corrected_axes_and_preserves_each_controller_sample(node):
    n, clock = node
    commands, poses = [], []
    n.command.publish = commands.append
    n.pose_pub.publish = poses.append
    n.recalibrate(Trigger.Request(), Trigger.Response())
    blue = node[0].field.goal_back('blue')
    theta0 = math.atan2(-.1, (blue[0][0]+blue[1][0])/2-.2)
    for _ in range(12):
        feed(n, clock, theta0)
    assert n.ready and set(n.initializer.anchors) == {'yellow', 'blue'}
    assert n.initializer.anchors['blue']['observed']
    assert not n.initializer.anchors['yellow']['observed']
    move = Twist()
    move.linear.x = .08
    n.on_command(move)
    assert np.allclose(n.map_odom, (.3, .1, .7), atol=.02)
    assert poses
    assert (n.root/'heading_reference.yaml').is_file()
    assert not (n.root/'outer_walls.pgm').exists()  # debug mapping is disabled by default
    assert (n.root/'initial_scan.csv').is_file()
    n.on_command(move)
    assert commands[-1] is move  # no 20 Hz resampling or changed MPC twist
    clock.t += 1.
    n.tick()
    assert not n.ready and n.state == 'LOCALIZATION_LOST'
    n.on_command(move)
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    # Lost localization must not trigger an unsolicited rotation after startup.
    assert n.initializer.offset is not None
    clock.t += 3.
    n.tick()
    assert n.state == 'REINITIALIZATION_REQUIRED'


def test_abort_timeout_and_new_session(node):
    n, clock = node
    commands = []
    n.command.publish = commands.append
    n.recalibrate(Trigger.Request(), Trigger.Response())
    feed(n, clock, 0., goal=False)
    assert n.state == 'AIMING_GOAL'
    clock.t += 61
    n.tick()
    assert n.state == 'CALIBRATION_TIMEOUT'
    assert commands[-1].angular.z == 0
    prior = n.root
    result = n.recalibrate(Trigger.Request(), Trigger.Response())
    assert result.success and n.root != prior and prior.exists()
    assert n.initializer.offset is None
    result = n.abort(Trigger.Request(), Trigger.Response())
    assert result.success
    n.tick()
    assert n.state == 'ABORTED' and commands[-1].angular.z == 0


def test_green_blink_then_team_colour_uses_existing_service_only(node):
    n, clock = node
    n.recalibrate(Trigger.Request(), Trigger.Response())
    requests = []

    class Done:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(success=True)

    client = SimpleNamespace(service_is_ready=lambda: True,
        call_async=lambda request: requests.append(request) or Done())
    n.led_clients = [(client, SetLed, 'led')]
    n.update_led(1000.)
    n.update_led(1000.6)
    assert {(r.r, r.g, r.b) for r in requests} == {(0, 0, 0), (0, 96, 0)}
    n.ready = True
    n.team = 'yellow'
    n.update_led(1001.)
    assert (requests[-1].r, requests[-1].g, requests[-1].b) == (96, 96, 0)
    n.team = 'blue'
    n.update_led(1001.1)
    assert (requests[-1].r, requests[-1].g, requests[-1].b) == (0, 0, 96)


def test_invalid_imu_or_reset_cannot_leave_an_old_heading_active(node):
    n, clock = node
    n.recalibrate(Trigger.Request(), Trigger.Response())
    feed(n, clock, 0.)
    clock.t += .2
    imu = Imu()
    imu.header.stamp, imu.header.frame_id = Time(seconds=clock.t).to_msg(), 'imu'
    imu.orientation.z, imu.orientation.w = 1., 0.
    for _ in range(3):
        clock.t += .02
        imu.header.stamp = Time(seconds=clock.t).to_msg()
        n.on_imu(imu)
    assert n.gate.aborted
    n.tick()
    assert not n.ready


def test_connected_robot_is_data_only_until_explicit_signal(node):
    n, clock = node
    commands, led_requests, transforms = [], [], []
    n.command.publish = commands.append
    n.broadcaster.sendTransform = transforms.append
    n.led_clients = [(SimpleNamespace(service_is_ready=lambda: True,
        call_async=lambda request: led_requests.append(request)), SetLed, 'led')]
    for _ in range(6):
        feed(n, clock, 0.)
    move = Twist()
    move.linear.x = .08
    n.on_command(move)
    assert n.state == 'WAITING_FOR_CALIBRATION'
    assert n.seek_started is None and not n.initializer.anchors
    assert n.fits and n.imus and n.odoms  # real sensor processing is still active
    assert not commands and not led_requests and not transforms
    n.recalibrate(Trigger.Request(), Trigger.Response())
    for _ in range(3):
        feed(n, clock, 0., goal=False)
    assert n.state == 'AIMING_GOAL' and n.calibration_requested
    assert any(abs(msg.angular.z) > 0 for msg in commands)
    assert led_requests


def test_signal_before_connection_waits_then_runs_without_another_signal(node):
    n, clock = node
    commands = []
    n.command.publish = commands.append
    n.recalibrate(Trigger.Request(), Trigger.Response())
    clock.t += 120
    n.tick()
    assert n.state == 'WAITING_FOR_ROBOT'
    assert n.seek_started is None and commands[-1].angular.z == 0
    feed(n, clock, 0., goal=False)
    assert n.state == 'AIMING_GOAL'
    assert n.seek_started == clock.t


def test_initial_reconnection_recovers_but_does_not_override_abort(node):
    n, clock = node
    n.recalibrate(Trigger.Request(), Trigger.Response())
    for _ in range(5):
        feed(n, clock, 0.)
    start = n.seek_started
    clock.t += 1.
    n.tick()
    assert n.state == 'WAITING_FOR_ROBOT'
    feed(n, clock, 0., goal=False)
    assert n.state == 'AIMING_GOAL' and not n.gate.aborted
    assert n.seek_started == start
    n.abort(Trigger.Request(), Trigger.Response())
    clock.t += 1.
    feed(n, clock, 0., goal=False)
    assert n.state == 'ABORTED'


def test_ready_never_decodes_goal_images_and_single_imu_outlier_is_ignored(node, monkeypatch):
    n, clock = node
    n.recalibrate(Trigger.Request(), Trigger.Response())
    theta = math.atan2(-.1, n.length/2-.2)
    for _ in range(12):
        feed(n, clock, theta)
    assert n.ready
    previous = n.last_imu
    imu = Imu()
    clock.t += .02
    imu.header.stamp, imu.header.frame_id = Time(seconds=clock.t).to_msg(), 'imu'
    imu.orientation.z, imu.orientation.w = 1., 0.
    n.on_imu(imu)
    assert not n.gate.aborted and n.last_imu == previous
    def forbidden(*args, **kwargs):
        raise AssertionError('Goal image decoding must be disabled after ready')
    monkeypatch.setattr(cv2, 'imdecode', forbidden)
    n.on_image(CompressedImage())


def test_scan_schedule_uses_sensor_time_despite_wall_arrival_jitter(node, monkeypatch):
    n, clock = node
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: .99*clock.t,
                                                      perf_counter=time.perf_counter))
    for _ in range(12):
        feed(n, clock, 0., goal=False, dt=.1)
    # 99 ms wall arrivals used to skip alternating 100 ms source slots.
    assert n.fit_attempts == 12
    assert n.fit_successes == 12
