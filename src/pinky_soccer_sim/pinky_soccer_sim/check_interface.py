#!/usr/bin/env python3
"""Passively check the same sensor contract in real, Gazebo and MuJoCo environments."""

import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, Imu, Range
from std_msgs.msg import Float32, UInt16MultiArray
import yaml


def main(args=None):
    """Collect samples and save a reviewable report; never command a robot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--output', required=True, help='YAML report path')
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--imu', action='store_true')
    parser.add_argument('--adc', action='store_true')
    parser.add_argument('--ir', action='store_true', help='Require calibrated IR ADC output too')
    argv = remove_ros_args(args=['check_interface', *args] if args is not None else None)[1:]
    options = parser.parse_args(argv)
    if not math.isfinite(options.duration) or options.duration <= 0:
        parser.error('duration must be positive')
    rclpy.init(args=args)
    node = Node('soccer_interface_check')
    expected = {'battery/voltage': Float32, 'battery/percent': Float32}
    if not options.no_camera:
        expected.update({'camera/image_raw': Image,
                         'camera/image_raw/compressed': CompressedImage,
                         'camera/camera_info': CameraInfo})
    if options.imu:
        expected.update({'imu_raw': Imu, 'imu_native': Imu})
    if options.adc:
        expected.update({'us_sensor/range': Range, 'batt_state': BatteryState})
    if options.ir:
        expected['ir_sensor/range'] = UInt16MultiArray
    samples = {topic: [] for topic in expected}
    subscriptions = []

    def observe(topic, msg):
        items = samples[topic]
        items.append(msg)
        # Bounded image memory; report counts separately from retained samples.
        counts[topic] += 1
        if len(items) > 3:
            del items[0]

    counts = dict.fromkeys(expected, 0)
    errors = []
    try:
        for topic, typ in expected.items():
            subscriptions.append(node.create_subscription(
                typ, topic, lambda msg, name=topic: observe(name, msg), qos_profile_sensor_data))
        deadline = time.monotonic() + options.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        for topic, values in samples.items():
            if not values:
                errors.append(f'No samples: {topic}')
            types = {endpoint.topic_type for endpoint in node.get_publishers_info_by_topic(topic)}
            expected_type = (expected[topic].__module__.split('.')[0]
                             + '/msg/' + expected[topic].__name__)
            if types != {expected_type}:
                errors.append(f'Type/publisher mismatch: {topic}: {sorted(types)}')
        for topic in ('camera/image_raw', 'camera/image_raw/compressed', 'camera/camera_info'):
            if topic not in samples:
                continue
            if any(msg.header.frame_id != 'front_camera_optical_frame' for msg in samples[topic]):
                errors.append(f'Wrong optical frame: {topic}')
            for endpoint in node.get_publishers_info_by_topic(topic):
                if endpoint.qos_profile.reliability != ReliabilityPolicy.BEST_EFFORT:
                    errors.append(f'Wrong camera reliability: {topic}')
        for msg in samples.get('camera/image_raw', []):
            if msg.encoding != 'bgr8' or msg.step != msg.width * 3:
                errors.append('Wrong raw image encoding/stride')
        for topic, frame in [('imu_raw', 'imu_link'), ('us_sensor/range', 'ultrasonic_link')]:
            if any(msg.header.frame_id != frame for msg in samples.get(topic, [])):
                errors.append(f'Wrong frame: {topic}')
        report = {'passed': not errors, 'errors': sorted(set(errors)),
                  'duration_wall_seconds': options.duration, 'samples': counts,
                  'topics': dict(node.get_topic_names_and_types()),
                  'services': dict(node.get_service_names_and_types())}
        Path(options.output).write_text(yaml.safe_dump(report, sort_keys=False))
        print(f"{'PASS' if report['passed'] else 'FAIL'}: {options.output}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
