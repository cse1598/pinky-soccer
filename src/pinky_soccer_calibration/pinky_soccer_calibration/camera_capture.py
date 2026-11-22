"""Strict raw service decoding and capture provenance, separate from JPEG preview."""

from dataclasses import dataclass
import hashlib
import time
import uuid

from cv_bridge import CvBridge
from rosidl_runtime_py.convert import message_to_yaml
import yaml


@dataclass
class RawCapture:
    """An unmodified raw frame with the matching original ROS messages and provenance."""

    frame: object
    image: object
    camera_info: object
    metadata: dict


def decode_capture(response, service, request_ros_ns, request_monotonic, clock_sync_verified):
    """Reject failed/JPEG/mismatched responses instead of calibrating with preview data."""
    if response is None or not response.success:
        reason = response.message if response is not None else '응답 없음'
        raise ValueError('원본 촬영 서비스 실패: '+reason)
    image, info = response.image, response.camera_info
    if (image.encoding != 'bgr8' or not image.width or not image.height
            or image.step != image.width*3 or len(image.data) != image.step*image.height
            or image.header != info.header
            or (image.width, image.height) != (info.width, info.height)):
        raise ValueError('원본 촬영 응답의 이미지 형식·크기·CameraInfo 시각이 일치하지 않습니다.')
    frame = CvBridge().imgmsg_to_cv2(image, 'bgr8').copy()
    metadata = {
        'schema_version': 1, 'capture_id': uuid.uuid4().hex,
        'service': service, 'format': 'raw', 'encoding': image.encoding,
        'width': image.width, 'height': image.height, 'camera_info': yaml.safe_load(
            message_to_yaml(info)),
        'request_sent_ros_ns': request_ros_ns, 'request_monotonic': request_monotonic,
        'received_unix_ns': time.time_ns(),
        'request_to_receive_s': time.monotonic()-request_monotonic,
        'clock_sync_verified': clock_sync_verified,
        'pixel_sha256': hashlib.sha256(frame.tobytes()).hexdigest(),
    }
    return RawCapture(frame, image, info, metadata)
