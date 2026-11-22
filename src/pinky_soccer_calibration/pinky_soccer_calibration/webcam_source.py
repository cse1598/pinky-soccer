"""V4L2 camera owner process with bounded latest-frame IPC and deterministic shutdown."""

import multiprocessing as mp
from queue import Empty, Full
import time

import cv2


def _put_latest(queue, item):
    try:
        queue.put_nowait(item)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(item)
        except Full:
            pass


def camera_worker(device, width, height, fps, queue, stopping):
    """Own the device only in this process, so a stalled driver can be terminated safely."""
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            _put_latest(queue, ('error', f'웹캠 {device}을 열 수 없습니다. 장치/권한/다른 앱 사용 여부를 확인하세요.'))
            return
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Request only; support depends on the driver.
        sequence = 0
        while not stopping.is_set():
            started = time.monotonic()
            ok, frame = camera.read()
            read_mono, read_unix = time.monotonic_ns(), time.time_ns()
            if not ok or frame is None:
                _put_latest(queue, ('error', '웹캠 영상 읽기가 중단됐습니다. 장치를 확인 후 UI를 재실행하세요.'))
                return
            sequence += 1
            _put_latest(queue, ('frame', sequence, read_mono, read_unix, frame))
            stopping.wait(max(0., 1./fps - (time.monotonic()-started)))
    finally:
        camera.release()


class WebcamSource:
    """Read a real local device; never replace a missing webcam with synthetic video."""

    def __init__(self, device=0, width=640, height=480, fps=15.):
        context = mp.get_context('spawn')
        self.queue = context.Queue(maxsize=1)
        self.stopping = context.Event()
        self.process = context.Process(target=camera_worker,
            args=(device, width, height, fps, self.queue, self.stopping), daemon=True)
        self.process.start()

    def get(self, timeout=.2):
        try:
            return self.queue.get(timeout=timeout)
        except Empty:
            if not self.process.is_alive():
                return 'error', '웹캠 프로세스가 종료됐습니다. 장치 연결을 확인하세요.'
            raise

    def close(self):
        self.stopping.set()
        self.process.join(timeout=1.5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.)
        self.queue.close()
        self.queue.cancel_join_thread()
