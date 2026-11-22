"""Bounded, asynchronous next-frame capture service shared by real and simulated cameras."""

import math
import threading
import time

from pinky_soccer_bringup.camera_support import image_messages
from pinky_soccer_msgs.srv import CaptureFrame
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.task import Future


class FrameCaptureService:
    """Resolve requests only with newer acquisition stamps, without subscribing to raw video."""

    def __init__(self, node, jpeg_quality=85):
        """Attach a service and a steady-clock timeout timer to an owning camera node."""
        self.node, self.jpeg_quality = node, jpeg_quality
        self.pending = []
        self.lock = threading.Lock()
        self.closed = False
        self.group = ReentrantCallbackGroup()
        self.service = node.create_service(
            CaptureFrame, 'camera/capture_frame', self.capture, callback_group=self.group)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(.05, self.expire, clock=self.steady,
                                       callback_group=self.group)

    @staticmethod
    def failure(message):
        """Return an unsuccessful response containing no stale image or calibration."""
        return CaptureFrame.Response(success=False, message=message)

    async def capture(self, request, response):
        """Wait cooperatively so a single-threaded simulator can deliver the next frame."""
        if request.format not in ('raw', 'jpeg'):
            return self.failure('format must be raw or jpeg')
        if not math.isfinite(request.timeout_sec) or not 0 < request.timeout_sec <= 10:
            return self.failure('timeout_sec must be finite and in (0, 10] seconds')
        future = Future()
        with self.lock:
            if self.closed:
                return self.failure('Camera service is closing')
            if len(self.pending) >= 4:
                return self.failure('Camera capture busy: maximum 4 pending requests')
            self.pending.append((future, time.monotonic() + request.timeout_sec,
                                 self.node.get_clock().now().nanoseconds, request.format))
        return await future

    def expire(self):
        """Bound waits using wall time, even when the ROS simulation clock is paused."""
        now = time.monotonic()
        with self.lock:
            expired = [item for item in self.pending if item[1] <= now]
            self.pending = [item for item in self.pending if item[1] > now]
        for future, _, _, _ in expired:
            if not future.done():
                future.set_result(self.failure('Timed out waiting for a new camera frame'))

    def complete(self, frame, stamp, frame_id, calibration):
        """Deliver one matching acquisition to all requests that predate that frame."""
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        now = time.monotonic()
        with self.lock:
            ready = [item for item in self.pending if stamp_ns > item[2]]
            self.pending = [item for item in self.pending if stamp_ns <= item[2]]
        if not ready:
            return
        # Encode at most once per format, regardless of the number of waiting clients.
        needed = {kind for _, deadline, _, kind in ready if deadline > now}
        try:
            raw, jpeg, info = image_messages(
                frame, stamp, frame_id, calibration, self.jpeg_quality,
                include_raw='raw' in needed, include_compressed='jpeg' in needed)
            error = None
        except Exception as exc:
            error = f'Camera frame conversion failed: {exc}'
        for future, deadline, _, kind in ready:
            if future.done():
                continue
            if time.monotonic() >= deadline:
                result = self.failure('Timed out waiting for a new camera frame')
            elif error:
                result = self.failure(error)
            else:
                result = CaptureFrame.Response(success=True, message='Captured a new frame',
                                               camera_info=info)
                if kind == 'raw':
                    result.image = raw
                else:
                    result.compressed_image = jpeg
            future.set_result(result)

    def close(self):
        """Release pending callbacks before the owning node tears down."""
        with self.lock:
            if self.closed:
                return
            self.closed = True
            pending, self.pending = self.pending, []
        for future, _, _, _ in pending:
            if not future.done():
                future.set_result(self.failure('Camera stopped before a new frame arrived'))
        self.node.destroy_timer(self.timer)
        self.node.destroy_service(self.service)
