"""Record received raw images and camera metadata, with timed lossless PNG samples."""

import csv
from datetime import datetime, timezone
from pathlib import Path
import time

import cv2

from .core import positive, save


def record(args):
    # Offline analysis does not need a sourced ROS environment.
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    import rosbag2_py
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image

    positive(args.square_mm)
    positive(args.interval)
    positive(args.duration)
    if not args.camera_id.strip() or not args.camera_settings.strip():
        raise ValueError('Camera identity and acquisition settings must be recorded')
    if args.image_topic == args.info_topic:
        raise ValueError('Image and CameraInfo topics must differ')
    root = Path(args.session).resolve()
    root.mkdir(parents=True)
    (root / 'frames').mkdir()
    save(root / 'session.yaml', {
        'schema_version': 1, 'stage': 'camera_intrinsics', 'purpose': args.purpose,
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'camera_id': args.camera_id, 'camera_settings': args.camera_settings,
        'board_inner_corners': [9, 6], 'square_mm': args.square_mm,
        'image_topic': args.image_topic, 'info_topic': args.info_topic,
        'sample_interval_sec': args.interval, 'duration_sec': args.duration,
        'notes': args.notes,
    })

    class Recorder(Node):
        def __init__(self):
            super().__init__('pinky_camera_calibration_recorder')
            self.bridge = CvBridge()
            self.bag = rosbag2_py.SequentialWriter()
            self.bag.open(rosbag2_py.StorageOptions(uri=str(root / 'raw_bag'), storage_id='sqlite3'),
                          rosbag2_py.ConverterOptions('', ''))
            for index, (topic, kind) in enumerate([
                    (args.image_topic, 'sensor_msgs/msg/Image'),
                    (args.info_topic, 'sensor_msgs/msg/CameraInfo')]):
                self.bag.create_topic(rosbag2_py.TopicMetadata(
                    id=index, name=topic, type=kind, serialization_format='cdr'))
            self.csv_file = (root / 'frames.csv').open('x', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['received_unix_ns', 'header_stamp_ns', 'frame_id',
                               'width', 'height', 'encoding', 'sample_png'])
            self.count = self.samples = self.infos = 0
            self.last_sample = -float('inf')
            self.geometry = None
            self.create_subscription(Image, args.image_topic, self.image, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, args.info_topic, self.info, qos_profile_sensor_data)

        def info(self, msg):
            self.bag.write(args.info_topic, serialize_message(msg), time.time_ns())
            self.infos += 1

        def image(self, msg):
            received = time.time_ns()
            self.bag.write(args.image_topic, serialize_message(msg), received)
            self.count += 1
            geometry = (msg.width, msg.height, msg.header.frame_id, msg.encoding)
            if self.geometry is not None and self.geometry != geometry:
                raise ValueError('Resolution, frame or encoding changed during recording')
            self.geometry = geometry
            name = ''
            now = time.monotonic()
            if now - self.last_sample >= args.interval:
                # Monochrome conversion is only for analysis; exact source messages stay in bag.
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                name = f'frames/{self.samples:06d}.png'
                if not cv2.imwrite(str(root / name), gray):
                    raise OSError(f'Cannot save {name}')
                self.samples += 1
                self.last_sample = now
                self.get_logger().info(f'Saved {self.samples} frames ({msg.width}x{msg.height})')
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.csv.writerow([received, stamp, msg.header.frame_id, msg.width,
                               msg.height, msg.encoding, name])
            self.csv_file.flush()

    rclpy.init(args=[])
    node = None
    failure = None
    try:
        node = Recorder()
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        failure = str(error)
        raise
    finally:
        if node is not None:
            node.csv_file.close()
            # Destroy the writer before returning, to flush sqlite and bag metadata.
            del node.bag
            geometry = node.geometry or (None, None, None, None)
            save(root / 'acquisition.yaml', {
                'completed': failure is None and node.samples > 0,
                'failure': failure, 'received_images': node.count,
                'sampled_images': node.samples, 'received_camera_info': node.infos,
                'image_size': list(geometry[:2]), 'frame_id': geometry[2],
                'encoding': geometry[3],
                'note': 'Bag contains received messages, not a guarantee of loss-free camera delivery.',
            })
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node is None or node.samples == 0:
        raise ValueError('No images received; check camera driver, topic, DDS domain and QoS')
    return str(root)
