"""Local webcam rehearsal adapter: no ROS context, discovery, subscribers or motor publisher."""

from collections import defaultdict, deque
from queue import Empty
import threading
import time
from types import SimpleNamespace

from cv_bridge import CvBridge
import numpy as np
from pinky_soccer_bringup.camera_support import image_messages
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import CameraInfo

from .board_geometry import MECHANICAL_FROM_OPTICAL, rpy_matrix
from .camera_capture import decode_capture
from .suite_ros import Acquisition, PREVIEW_TOPIC
from .webcam_source import WebcamSource


class WebcamAcquisition:
    """Use the same Bag/CSV/PNG pipeline with local frames instead of robot services."""

    # These functions operate on messages/data only; no Node or DDS constructor is used.
    receive = Acquisition.receive
    end = Acquisition.end
    fresh = Acquisition.fresh
    record_capture = Acquisition.record_capture
    monitor_snapshot = Acquisition.monitor_snapshot
    measurement_geometry = Acquisition.measurement_geometry
    review_frame = Acquisition.review_frame

    source_kind = 'webcam'
    floor_frame = 'webcam_floor'
    optical_frame = 'webcam_optical_frame'

    def __init__(self, device=0, width=640, height=480, fps=15., hfov=None, source=None):
        self.device, self.hfov = str(device), hfov
        self.requested_size, self.requested_fps = [width, height], fps
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.latest, self.seen = {}, {}
        self.values = defaultdict(lambda: deque(maxlen=20000))
        self.bridge = CvBridge()
        self.writer = self.csv_file = self.recording_session = None
        self.motion = self.last_motion = None
        self.geometry = None
        self.geometry_error = '웹캠 리허설에는 로봇/범퍼 모델이 없습니다.'
        self.capture_service_name = 'local_webcam'
        self.capture_timeout = 3.
        self.sync_enabled = False
        self.sync_state = {'ready': True, 'reason': '같은 PC 시계 · 노출이 아닌 읽기 완료 시각'}
        self.running, self.error, self.raw, self.resolution = True, '', None, None
        self.project = None
        self.ready = threading.Event()
        self.source = source or WebcamSource(device, width, height, fps)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        if not self.ready.wait(8.) or self.error:
            error = self.error or '웹캠 첫 프레임 수신 시간 초과'
            self.close()
            raise ValueError(error)

    def get_clock(self):
        return self.clock

    def synchronized(self):
        return True  # Frame read and receipt use the same host system clock, not hardware exposure time.

    def identity(self):
        return {'kind': 'webcam', 'device': self.device, 'image_size': list(self.resolution),
                'requested_size': self.requested_size, 'requested_fps': self.requested_fps,
                'backend': 'opencv_v4l2', 'timestamp_basis': 'host_read_completion'}

    def bind_project(self, project, creating=False):
        if project.meta['environment'] != 'rehearsal':
            raise ValueError('웹캠은 rehearsal 프로젝트만 사용할 수 있습니다.')
        source = project.meta.get('image_source', {})
        if creating:
            project.meta['image_source'] = self.identity()
        elif (source.get('kind') != 'webcam' or source.get('device') != self.device
              or source.get('image_size') != list(self.resolution)):
            raise ValueError('현재 장치/해상도의 웹캠 전용 프로젝트를 선택하세요. 핑키 프로젝트와 섞을 수 없습니다.')
        self.project = project
        project.save()

    def begin(self, project):
        self.bind_project(project)
        Acquisition.begin(self, project)
        project.meta['image_acquisition'].update(
            source='local_webcam', capture_service=None, capture_method='next_local_raw_frame',
            network_transport=False, device=self.device)
        project.meta['recordings'][-1]['image_source'] = self.identity()
        project.save()

    def loop(self):
        try:
            while self.running:
                try:
                    item = self.source.get(timeout=.2)
                except Empty:
                    continue
                if item[0] == 'error':
                    raise ValueError(item[1])
                _, sequence, read_mono, read_unix, frame = item
                if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3:
                    raise ValueError('웹캠은 uint8 BGR 프레임을 반환해야 합니다.')
                size = frame.shape[1], frame.shape[0]
                if self.resolution is not None and size != self.resolution:
                    raise ValueError('웹캠 해상도가 바뀌었습니다. UI를 재실행하세요.')
                self.resolution = size
                calibration = CameraInfo(width=size[0], height=size[1])
                stamp = self.clock.now().to_msg()
                stamp.sec, stamp.nanosec = divmod(read_unix, 1_000_000_000)
                _, jpeg, info = image_messages(frame, stamp, self.optical_frame, calibration,
                                              include_raw=False)
                with self.condition:
                    self.raw = (sequence, read_mono, read_unix, frame.copy(), info)
                    self.receive(PREVIEW_TOPIC, jpeg)
                    self.receive('/camera/camera_info', info)
                    self.condition.notify_all()
                self.ready.set()
        except Exception as error:
            if self.running:
                with self.condition:
                    self.error, self.raw = str(error), None
                    self.seen.pop(PREVIEW_TOPIC, None)
                    self.condition.notify_all()
                self.ready.set()

    def frame(self):
        with self.lock:
            if self.error:
                raise ValueError(self.error)
            if not self.fresh(PREVIEW_TOPIC):
                raise ValueError('웹캠 프레임 수신 지연 · 카메라 연결/다른 앱 점유를 확인하세요.')
        return Acquisition.frame(self)

    def capture_raw(self):
        requested = time.monotonic_ns()
        sent = time.monotonic()
        sent_ros = self.clock.now().nanoseconds
        with self.condition:
            while self.running and not self.error:
                if self.raw is not None and self.raw[1] > requested:
                    sequence, read_mono, read_unix, frame, info = self.raw
                    break
                remaining = self.capture_timeout-(time.monotonic()-sent)
                if remaining <= 0:
                    raise ValueError('웹캠 새 원본 촬영 시간 초과. 이전 프레임을 재사용하지 않습니다.')
                self.condition.wait(remaining)
            else:
                raise ValueError(self.error or '웹캠이 종료됐습니다.')
        image, _, info = image_messages(frame, info.header.stamp, self.optical_frame, info,
                                       include_compressed=False)
        sample = decode_capture(SimpleNamespace(success=True, image=image, camera_info=info),
                                'local_webcam', sent_ros, sent, True)
        sample.metadata.pop('service', None)
        sample.metadata.update(source='local_webcam', device=self.device, sequence=sequence,
                               read_complete_unix_ns=read_unix, read_complete_monotonic_ns=read_mono,
                               timestamp_basis='host_read_completion', transport='local_process')
        return sample

    def review_transform(self, target, source, stamp=None):
        # The webcam's fixed floor axes are a local definition, never a robot pose or odometry.
        if target == source == self.floor_frame:
            return np.eye(3), np.zeros(3)
        project = self.project
        if not project or {'camera', 'floor'} & project.stale:
            return None
        summary = project.results.get('camera_mount', {})
        if target == self.floor_frame and source == self.optical_frame:
            if not all(k in summary for k in ('height_m', 'roll_rad', 'pitch_rad')):
                return None
            height, roll, pitch = [float(summary[k]['mean']) for k in ('height_m', 'roll_rad', 'pitch_rad')]
            if not np.isfinite([height, roll, pitch]).all() or height <= 0:
                return None
            return rpy_matrix(roll, pitch, 0.) @ MECHANICAL_FROM_OPTICAL, np.array([0., 0., height])
        return None

    def start_motion(self, kind):
        raise ValueError('웹캠 리허설에서는 로봇 주행/센서 실험을 실행할 수 없습니다.')

    def stop(self, reason='웹캠 정지'):
        pass  # No command publisher exists in local mode.

    def close(self):
        if not self.running:
            return
        self.running = False
        with self.condition:
            self.condition.notify_all()
        self.source.close()
        self.thread.join(timeout=2.)
        self.end()
