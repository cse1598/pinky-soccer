#!/usr/bin/env python3
"""Normalize simulated observations to the physical robot's public ROS interface."""

from copy import deepcopy
import json
import math

import cv2
import numpy as np
from pinky_soccer_bringup.camera_support import image_messages
from pinky_soccer_bringup.frame_capture import FrameCaptureService
from pinky_soccer_sim.sensor_model import adc_from_distance, load_config, RANGE_LINKS
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import (BatteryState, CameraInfo, CompressedImage, Image, Imu,
                             LaserScan, MagneticField, Range)
from std_msgs.msg import Float32, String, UInt16MultiArray


def ideal_magnetic_field(orientation, world_field):
    """Rotate an explicit ideal ambient field into the simulated IMU sensor axes."""
    x, y, z, w = (getattr(orientation, k) for k in ('x', 'y', 'z', 'w'))
    norm = x*x+y*y+z*z+w*w
    if not math.isfinite(norm) or norm < .9:
        raise ValueError('Invalid simulated IMU orientation')
    q = np.array([x, y, z])/math.sqrt(norm)
    w /= math.sqrt(norm)
    cross = np.array([[0, -q[2], q[1]], [q[2], 0, -q[0]], [-q[1], q[0], 0]])
    rotation = np.eye(3)+2*w*cross+2*cross@cross
    return rotation.T @ np.asarray(world_field)


def bgr_frame(msg):
    """Respect Image encoding and row stride instead of assuming packed RGB."""
    if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width * 3:
        raise ValueError(f'Unsupported camera layout: {msg.encoding}')
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if data.size != msg.height * msg.step:
        raise ValueError('Image payload does not match height/step')
    frame = data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
        msg.height, msg.width, 3)
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else frame.copy()


class SensorAdapter(Node):
    """Publish the same names, types, units and camera QoS as the real launch."""

    def __init__(self, **kwargs):
        """Configure optional sensors without inventing unknown IR ADC calibration."""
        super().__init__('soccer_sim_sensors', **kwargs)
        self.cfg = load_config(self.declare_parameter('sensor_config', '').value)
        self.images, self.infos = {}, {}
        self.ir_samples = {}
        self.last_image = None
        self.capture_service = None
        qos = qos_profile_sensor_data
        if self.declare_parameter('camera_enabled', True).value:
            self.raw = self.create_publisher(Image, 'camera/image_raw', qos)
            self.jpeg = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos)
            self.info = self.create_publisher(CameraInfo, 'camera/camera_info', qos)
            self.capture_service = FrameCaptureService(self, self.cfg['camera']['jpeg_quality'])
            self.create_subscription(Image, 'sim/camera/image_raw', self.image_received, qos)
            self.create_subscription(CameraInfo, 'sim/camera/camera_info', self.info_received, qos)
        if self.declare_parameter('imu_enabled', False).value:
            self.imu = self.create_publisher(Imu, 'imu_raw', qos)
            self.native = self.create_publisher(Imu, 'imu_native', 10)
            self.mag = self.create_publisher(MagneticField, 'imu/mag', qos)
            self.mag_status = self.create_publisher(String, 'imu/status', qos)
            self.world_field = self.declare_parameter(
                'magnetic_field_world_tesla', [22e-6, 0., -42e-6]).value
            if len(self.world_field) != 3 or not all(math.isfinite(v) for v in self.world_field):
                raise ValueError('magnetic_field_world_tesla must be a finite 3-vector')
            self.create_subscription(Imu, 'sim/imu', self.imu_received, qos)
        self.adc = self.declare_parameter('adc_enabled', False).value
        if self.adc:
            self.us = self.create_publisher(Range, 'us_sensor/range', 10)
            self.ir = None
            if self.cfg['ir']['distances']:
                self.ir = self.create_publisher(UInt16MultiArray, 'ir_sensor/range', 10)
            else:
                self.get_logger().warning(
                    'IR ray distances enabled; ir_sensor/range disabled: '
                    'measured ADC table missing')
            self.batt = self.create_publisher(BatteryState, 'batt_state', 10)
            self.ir_distances = {}
            for link in RANGE_LINKS:
                if link != 'ultrasonic_link':
                    self.ir_distances[link] = self.create_publisher(
                        Range, f'sim/{link}/distance', qos)
                self.create_subscription(
                    LaserScan, f'sim/{link}/scan',
                    lambda msg, name=link: self.range_received(name, msg), qos)
            self.create_timer(0.05, self.publish_adc_battery)
        self.voltage = self.create_publisher(Float32, 'battery/voltage', 10)
        self.percent = self.create_publisher(Float32, 'battery/percent', 10)
        self.create_timer(5.0, self.publish_battery)

    @staticmethod
    def key(msg):
        """Use the acquisition timestamp to pair image and CameraInfo samples."""
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def image_received(self, msg):
        """Cache a bounded number of images until the matching intrinsics arrive."""
        self.images[self.key(msg)] = msg
        self.publish_image(self.key(msg))

    def info_received(self, msg):
        """Accept CameraInfo only for the matching acquisition timestamp."""
        self.infos[self.key(msg)] = msg
        self.publish_image(self.key(msg))

    def publish_image(self, key):
        """Convert to BGR, use an optical frame and preserve simulator timestamps."""
        if key in self.images and key in self.infos:
            msg, info = self.images.pop(key), self.infos.pop(key)
            try:
                c = self.cfg['camera']
                # Simulator time can reset. Resume immediately after a backwards jump.
                if (self.last_image is not None and key >= self.last_image
                        and key - self.last_image < 1e9 / c['fps'] - 1):
                    return
                frame = bgr_frame(msg)
                if (msg.width, msg.height) != (c['width'], c['height']):
                    raise ValueError('Native camera resolution differs from sensor_config')
                raw, jpeg, result = image_messages(
                    frame, msg.header.stamp, 'front_camera_optical_frame', info,
                    c['jpeg_quality'], self.raw.get_subscription_count() > 0,
                    self.jpeg.get_subscription_count() > 0)
                if raw is not None:
                    self.raw.publish(raw)
                if jpeg is not None:
                    self.jpeg.publish(jpeg)
                self.info.publish(result)
                self.capture_service.complete(
                    frame, msg.header.stamp, 'front_camera_optical_frame', info)
                self.last_image = key
            except ValueError as error:
                self.get_logger().error(str(error), throttle_duration_sec=5)
        for cache in (self.images, self.infos):
            while len(cache) > 5:
                del cache[next(iter(cache))]

    def destroy_node(self):
        """Finish pending capture requests before releasing subscriptions."""
        if self.capture_service is not None:
            self.capture_service.close()
            self.capture_service = None
        return super().destroy_node()

    def imu_received(self, msg):
        """Native simulators use SI; provide the legacy deg/s diagnostic stream too."""
        msg = deepcopy(msg)
        msg.header.frame_id = 'imu_link'
        self.imu.publish(msg)
        native = deepcopy(msg)
        for axis in ('x', 'y', 'z'):
            setattr(native.angular_velocity, axis,
                    math.degrees(getattr(native.angular_velocity, axis)))
        if native.angular_velocity_covariance[0] != -1:
            native.angular_velocity_covariance = [
                float(v) * (180 / math.pi)**2 for v in native.angular_velocity_covariance]
        self.native.publish(native)
        magnetic = MagneticField(header=deepcopy(msg.header))
        (magnetic.magnetic_field.x, magnetic.magnetic_field.y,
         magnetic.magnetic_field.z) = map(float, ideal_magnetic_field(msg.orientation, self.world_field))
        self.mag.publish(magnetic)
        self.mag_status.publish(String(data=json.dumps({
            'stamp': self.key(msg)/1e9, 'source': 'ideal_simulated_ambient_field',
            'mode': 'NDOF', 'system': 3, 'gyro': 3, 'accel': 3, 'mag': 3})))

    def range_received(self, link, msg):
        """Reduce cone rays and assemble three simultaneous IR channels L/M/R."""
        cfg = self.cfg['ultrasonic' if link == 'ultrasonic_link' else 'ir']
        values = [float(v) for v in msg.ranges if not math.isnan(v)]
        if not values:
            return
        distance = min(values)
        result = Range()
        result.header = deepcopy(msg.header)
        result.header.frame_id = link
        result.radiation_type = Range.ULTRASOUND if link == 'ultrasonic_link' else Range.INFRARED
        result.field_of_view = cfg['field_of_view'] if link == 'ultrasonic_link' else 0.0
        result.min_range, result.max_range = cfg['min_range'], cfg['max_range']
        result.range = (float('-inf') if distance < cfg['min_range']
                        else float('inf') if distance > cfg['max_range'] else distance)
        if link == 'ultrasonic_link':
            self.us.publish(result)
            return
        self.ir_distances[link].publish(result)
        if self.ir is None:
            return
        key = self.key(msg)
        self.ir_samples.setdefault(key, {})[link] = distance
        samples = self.ir_samples[key]
        if len(samples) == 3:
            # Official ADC driver sends channels in left, middle, right order.
            values = [adc_from_distance(max(0.0, samples[name]), self.cfg['ir'])
                      for name in RANGE_LINKS[1:]]
            self.ir.publish(UInt16MultiArray(data=values))
            del self.ir_samples[key]
        while len(self.ir_samples) > 5:
            del self.ir_samples[next(iter(self.ir_samples))]

    def publish_battery(self):
        """Expose an explicit ideal battery scenario, without claiming measured discharge."""
        self.voltage.publish(Float32(data=float(self.cfg['battery']['voltage'])))
        self.percent.publish(Float32(data=float(self.cfg['battery']['percent'])))

    def publish_adc_battery(self):
        """Match the official ADC BatteryState unknown fields and units."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self.cfg['battery']['voltage'])
        for key in ('temperature', 'current', 'charge', 'capacity', 'percentage'):
            setattr(msg, key, float('nan'))
        msg.design_capacity = 5.0
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.location, msg.serial_number = 'base_link', '0'
        self.batt.publish(msg)


def main(args=None):
    """Run using simulation time provided by the simulator's clock."""
    rclpy.init(args=args)
    node = SensorAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
