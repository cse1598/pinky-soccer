"""ROS camera message conversion and strict loading of measured intrinsics."""

from copy import deepcopy
import math
from pathlib import Path
from urllib.parse import unquote, urlsplit

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import yaml


def positive(value, name):
    """Validate a positive finite driver parameter."""
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def camera_info(url, width, height):
    """Load ROS YAML without guessing intrinsics or rescaling a calibration."""
    info = CameraInfo(width=width, height=height)
    if not url:
        return info  # K[0] == 0: uncalibrated according to sensor_msgs/CameraInfo.
    parsed = urlsplit(url)
    if parsed.scheme != 'file' or parsed.netloc not in ('', 'localhost') or parsed.query:
        raise ValueError('camera_info_url must be a local file:///absolute/path.yaml URL')
    path = Path(unquote(parsed.path))
    if not path.is_absolute() or parsed.fragment:
        raise ValueError('camera_info_url must contain an absolute file path')
    with path.open() as stream:
        data = yaml.safe_load(stream)
    if (data['image_width'], data['image_height']) != (width, height):
        raise ValueError('Calibration resolution differs from the requested camera resolution')
    model = data['distortion_model']
    counts = {'plumb_bob': (5,), 'rational_polynomial': (8,), 'equidistant': (4,)}
    if model not in counts:
        raise ValueError(f'Unsupported distortion model: {model}')
    matrices = {}
    for key, shape in [('camera_matrix', (3, 3)), ('rectification_matrix', (3, 3)),
                       ('projection_matrix', (3, 4))]:
        entry = data[key]
        values = np.asarray(entry['data'], dtype=float)
        if ((entry['rows'], entry['cols']) != shape or values.size != math.prod(shape)
                or not np.isfinite(values).all()):
            raise ValueError(f'Invalid {key}')
        matrices[key] = values.tolist()
    d = np.asarray(data['distortion_coefficients']['data'], dtype=float)
    if d.ndim != 1 or d.size not in counts[model] or not np.isfinite(d).all():
        raise ValueError('Invalid distortion coefficients')
    k = np.asarray(matrices['camera_matrix']).reshape(3, 3)
    if (k[0, 0] <= 0 or k[1, 1] <= 0 or not 0 <= k[0, 2] < width
            or not 0 <= k[1, 2] < height or not np.allclose(k[2], [0, 0, 1])):
        raise ValueError('Invalid focal lengths or principal point')
    info.distortion_model = model
    info.d = d.tolist()
    info.k = matrices['camera_matrix']
    info.r = matrices['rectification_matrix']
    info.p = matrices['projection_matrix']
    return info


def image_messages(frame, stamp, frame_id, calibration, jpeg_quality=85,
                   include_raw=True, include_compressed=True):
    """Return raw/JPEG/CameraInfo messages with identical acquisition headers."""
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8
            or frame.ndim != 3 or frame.shape[2] != 3):
        raise ValueError('pinkylib camera must return a uint8 BGR image with three channels')
    if frame.shape[:2] != (calibration.height, calibration.width):
        raise ValueError('Camera frame resolution changed; restart with matching calibration')
    if not 1 <= jpeg_quality <= 100:
        raise ValueError('jpeg_quality must be in [1, 100]')
    info = deepcopy(calibration)
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    raw = compressed = None
    if include_raw:
        raw = Image()
        raw.header = deepcopy(info.header)
        raw.height, raw.width = frame.shape[:2]
        raw.encoding = 'bgr8'
        raw.is_bigendian = 0
        raw.step = raw.width * 3
        raw.data = frame.tobytes(order='C')
    if include_compressed:
        ok, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        compressed = CompressedImage()
        compressed.header = deepcopy(info.header)
        compressed.format = 'bgr8; jpeg compressed bgr8'
        compressed.data = buffer.tobytes()
    return raw, compressed, info
