#!/usr/bin/env python3
"""Request exactly one camera frame and save it without subscribing to a video topic."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
from pinky_soccer_msgs.srv import CaptureFrame
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml


def save_capture(result, format_name, output):
    """Save lossless raw PNG or the received JPEG, plus matching CameraInfo/provenance."""
    path = Path(output).expanduser().resolve()
    sidecar = path.with_name(path.name + '.camera_info.yaml')
    suffixes = ('.png',) if format_name == 'raw' else ('.jpg', '.jpeg')
    if path.suffix.lower() not in suffixes:
        raise ValueError('raw requires .png; jpeg requires .jpg or .jpeg')
    if not result.success:
        raise ValueError(result.message)
    if path.exists() or sidecar.exists():
        raise FileExistsError('Output image or metadata already exists; choose a new filename')
    info = result.camera_info
    if format_name == 'raw':
        image = result.image
        if (image.header != info.header or image.encoding != 'bgr8'
                or (image.width, image.height) != (info.width, info.height)
                or image.step != image.width * 3):
            raise ValueError('Unexpected raw image layout or mismatched CameraInfo')
        frame = np.frombuffer(bytes(image.data), np.uint8).reshape(image.height, image.width, 3)
        ok, data = cv2.imencode('.png', frame)
        if not ok:
            raise ValueError('PNG encoding failed')
        payload = data.tobytes()
    else:
        image = result.compressed_image
        if image.header != info.header:
            raise ValueError('JPEG and CameraInfo headers differ')
        payload = bytes(image.data)
        frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.shape[:2] != (info.height, info.width):
            raise ValueError('Invalid JPEG or mismatched CameraInfo dimensions')
    metadata = {
        'image_width': info.width, 'image_height': info.height,
        'camera_name': info.header.frame_id, 'distortion_model': info.distortion_model,
        'distortion_coefficients': {'rows': 1, 'cols': len(info.d),
                                    'data': [float(v) for v in info.d]},
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in info.k]},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': [float(v) for v in info.r]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': [float(v) for v in info.p]},
        'capture': {'format': format_name, 'frame_id': info.header.frame_id,
                    'stamp': {'sec': info.header.stamp.sec,
                              'nanosec': info.header.stamp.nanosec},
                    'calibrated': bool(info.k[0] > 0 and info.k[4] > 0)},
    }
    metadata_text = yaml.safe_dump(metadata, sort_keys=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream:
        stream.write(payload)
    with sidecar.open('x') as stream:
        stream.write(metadata_text)
    return path, sidecar


def main(args=None):
    """Call the service, bound discovery/response waits, and leave existing files intact."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--format', choices=['raw', 'jpeg'], default='raw')
    parser.add_argument('--output', required=True, help='raw: .png; jpeg: .jpg/.jpeg')
    parser.add_argument('--timeout', type=float, default=3.0, help='New-frame timeout, (0, 10] s')
    parser.add_argument('--service', default='/camera/capture_frame')
    argv = remove_ros_args(args=['capture_image', *args] if args is not None else None)[1:]
    options = parser.parse_args(argv)
    if not math.isfinite(options.timeout) or not 0 < options.timeout <= 10:
        parser.error('--timeout must be finite and in (0, 10] seconds')
    rclpy.init(args=args)
    node = Node('camera_capture_client')
    try:
        client = node.create_client(CaptureFrame, options.service)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Capture service not found; check camera launch, domain and DDS')
        future = client.call_async(CaptureFrame.Request(
            format=options.format, timeout_sec=options.timeout))
        rclpy.spin_until_future_complete(node, future, timeout_sec=options.timeout + 5.0)
        if not future.done():
            raise RuntimeError('Capture response timed out; check camera node and network')
        image, info = save_capture(future.result(), options.format, options.output)
        print(f'Saved {image}\nCameraInfo: {info}')
    except (ValueError, RuntimeError, OSError) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
