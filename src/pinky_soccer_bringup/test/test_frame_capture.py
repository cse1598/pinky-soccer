"""Capture service regression tests without motors, real cameras or the robot network."""

import threading
import time

import cv2
import numpy as np
from pinky_soccer_bringup.camera_publisher import CameraPublisher
from pinky_soccer_bringup.capture_image import save_capture
from pinky_soccer_bringup.frame_capture import FrameCaptureService
from pinky_soccer_msgs.srv import CaptureFrame
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


class SyntheticCamera:
    """A changing image makes it possible to distinguish acquisitions."""

    def start(self, width, height):
        """Configure an in-memory source."""
        self.shape = (height, width, 3)
        self.count = 0

    def get_frame(self):
        """Produce the next image."""
        self.count += 1
        return np.full(self.shape, self.count % 200, np.uint8)

    def close(self):
        """No hardware to release."""


@pytest.fixture
def graph(tmp_path, monkeypatch):
    """Keep middleware and files separate from the user graph."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    context = Context()
    rclpy.init(args=[], context=context, domain_id=90)
    executor = SingleThreadedExecutor(context=context)
    nodes = []

    def add(node):
        executor.add_node(node)
        nodes.append(node)
        return node

    yield context, executor, add
    executor.shutdown()
    for node in nodes:
        node.destroy_node()
    context.shutdown()


def spin(executor, predicate, timeout=4):
    """Bound every asynchronous test by steady wall time."""
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.01)
    assert predicate(), 'Capture test exceeded its wall-clock deadline'


def client_node(graph):
    """Return a service client with no subscriptions to any image topic."""
    context, _, add = graph
    node = add(Node('capture_test_client', context=context))
    client = node.create_client(CaptureFrame, '/camera/capture_frame')
    assert client.wait_for_service(timeout_sec=3)
    return client


@pytest.mark.parametrize('kind,suffix', [('raw', '.png'), ('jpeg', '.jpg')])
def test_capture_and_save_without_video_subscriptions(graph, tmp_path, kind, suffix):
    """Exercise real camera callbacks and DDS service responses, keeping raw stream idle."""
    context, executor, add = graph
    camera = add(CameraPublisher(SyntheticCamera, context=context, parameter_overrides=[
        Parameter('width', value=8), Parameter('height', value=6), Parameter('fps', value=20.0)]))
    client = client_node(graph)
    before = camera.get_clock().now().nanoseconds
    future = client.call_async(CaptureFrame.Request(format=kind, timeout_sec=2.0))
    spin(executor, future.done)
    result = future.result()
    assert result.success, result.message
    assert camera.raw_pub.get_subscription_count() == 0
    assert camera.jpeg_pub.get_subscription_count() == 0
    payload = result.image if kind == 'raw' else result.compressed_image
    assert payload.header == result.camera_info.header
    stamp = payload.header.stamp
    assert stamp.sec * 10**9 + stamp.nanosec > before
    assert not (result.compressed_image.data if kind == 'raw' else result.image.data)
    path, sidecar = save_capture(result, kind, tmp_path / ('frame' + suffix))
    decoded = cv2.imread(str(path))
    assert decoded.shape == (6, 8, 3) and sidecar.exists()
    if kind == 'raw':
        assert decoded.tobytes() == bytes(result.image.data)
    else:
        assert path.read_bytes() == bytes(result.compressed_image.data)
    with pytest.raises(FileExistsError):
        save_capture(result, kind, path)


@pytest.mark.parametrize('kind,timeout', [('png', 1.0), ('raw', 0.0), ('raw', float('nan'))])
def test_invalid_requests_fail_without_payloads(graph, kind, timeout):
    """Malformed requests cannot trigger an unbounded capture."""
    context, executor, add = graph
    server = add(Node('capture_service_only', context=context))
    service = FrameCaptureService(server)
    client = client_node(graph)
    future = client.call_async(CaptureFrame.Request(format=kind, timeout_sec=timeout))
    spin(executor, future.done)
    result = future.result()
    assert not result.success and not result.image.data and not service.pending
    service.close()


def test_paused_clock_expires_and_old_frames_are_not_returned(graph):
    """Sim pause must not freeze the service timeout or return an old snapshot."""
    context, executor, add = graph
    server = add(Node('paused_capture_server', context=context, parameter_overrides=[
        Parameter('use_sim_time', value=True)]))
    service = FrameCaptureService(server)
    client = client_node(graph)
    future = client.call_async(CaptureFrame.Request(format='raw', timeout_sec=.2))
    spin(executor, lambda: bool(service.pending))
    service.complete(np.zeros((6, 8, 3), np.uint8), Time(nanoseconds=0).to_msg(),
                     'camera', CameraInfo(width=8, height=6))
    assert not future.done()
    spin(executor, future.done, timeout=1)
    assert not future.result().success and 'Timed out' in future.result().message
    assert not future.result().image.data
    service.close()


def test_newer_frame_and_bounded_parallel_requests(graph):
    """Serve new frames to admitted clients and reject excess pending requests."""
    context, executor, add = graph
    server = add(Node('bounded_capture_server', context=context))
    service = FrameCaptureService(server)
    client = client_node(graph)
    futures = [client.call_async(CaptureFrame.Request(format='raw', timeout_sec=2.0))
               for _ in range(5)]
    spin(executor, lambda: any(f.done() for f in futures))
    assert len(service.pending) == 4
    rejected = [f.result() for f in futures if f.done()]
    assert len(rejected) == 1 and not rejected[0].success
    assert 'busy' in rejected[0].message
    stamp = max(item[2] for item in service.pending)
    service.complete(np.zeros((6, 8, 3), np.uint8), Time(nanoseconds=stamp).to_msg(),
                     'camera', CameraInfo(width=8, height=6))
    # At least the most recent request still needs a strictly newer acquisition.
    assert service.pending
    service.complete(np.zeros((6, 8, 3), np.uint8), Time(nanoseconds=stamp+1).to_msg(),
                     'camera', CameraInfo(width=8, height=6))
    spin(executor, lambda: all(f.done() for f in futures))
    assert sum(f.result().success for f in futures) == 4
    service.close()


def test_capture_timeout_remains_responsive_when_sdk_read_stalls(graph):
    """A blocked capture thread must not block service callbacks or their timeout."""
    context, single, add = graph
    entered, release = threading.Event(), threading.Event()

    class SlowCamera(SyntheticCamera):
        def get_frame(self):
            entered.set()
            release.wait(timeout=3)
            return super().get_frame()

    camera = add(CameraPublisher(SlowCamera, context=context, parameter_overrides=[
        Parameter('width', value=8), Parameter('height', value=6), Parameter('fps', value=20.0)]))
    client = client_node(graph)
    receiver = next(node for node in single.get_nodes()
                    if node.get_name() == 'capture_test_client')
    multi = MultiThreadedExecutor(num_threads=2, context=context)
    for node in (camera, receiver):
        single.remove_node(node)
        multi.add_node(node)
    try:
        spin(multi, entered.is_set)
        future = client.call_async(CaptureFrame.Request(format='raw', timeout_sec=.15))
        spin(multi, future.done, timeout=1.5)
        assert not future.result().success and 'Timed out' in future.result().message
    finally:
        release.set()
        camera.capture_service.close()
        multi.shutdown(timeout_sec=4)
