#!/usr/bin/env python3
"""Publish the robot's pinkylib camera directly to ROS, without Jupyter."""

from pinky_soccer_bringup.camera_support import camera_info, image_messages, positive
from pinky_soccer_bringup.frame_capture import FrameCaptureService
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CameraPublisher(Node):
    """Own the physical camera and publish only fresh, successfully captured images."""

    def __init__(self, camera_factory=None, **kwargs):
        """Start the robot SDK lazily; camera_factory is injectable for tests."""
        super().__init__('soccer_camera', **kwargs)
        self.camera = None
        self.capture_service = None
        try:
            width = self.declare_parameter('width', 640).value
            height = self.declare_parameter('height', 480).value
            fps = positive(self.declare_parameter('fps', 15.0).value, 'fps')
            positive(width, 'width')
            positive(height, 'height')
            self.quality = self.declare_parameter('jpeg_quality', 85).value
            if not 1 <= self.quality <= 100:
                raise ValueError('jpeg_quality must be in [1, 100]')
            self.frame_id = self.declare_parameter(
                'frame_id', 'front_camera_optical_frame').value
            if not self.frame_id:
                raise ValueError('frame_id must not be empty')
            url = self.declare_parameter('camera_info_url', '').value
            self.calibration = camera_info(url, width, height)
            if not url:
                self.get_logger().warning(
                    'Camera is uncalibrated: CameraInfo K/D/R/P remain zero.')
            self.raw_pub = self.create_publisher(
                Image, 'camera/image_raw', qos_profile_sensor_data)
            self.jpeg_pub = self.create_publisher(
                CompressedImage, 'camera/image_raw/compressed', qos_profile_sensor_data)
            self.info_pub = self.create_publisher(
                CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
            if camera_factory is None:
                from pinkylib import Camera
                camera_factory = Camera
            self.camera = camera_factory()
            self.camera.start(width=width, height=height)
            self.failures = 0
            self.capture_service = FrameCaptureService(self, self.quality)
            self.timer = self.create_timer(1.0 / fps, self.publish_frame)
            self.get_logger().info(
                f'Camera ready: {width}x{height}, capture limit {fps:g} Hz; '
                'pinkylib get_frame() includes a 180-degree image rotation. '
                'Timestamp is ROS time at read completion, not sensor exposure time.')
        except Exception:
            self.destroy_node()
            raise

    def publish_frame(self):
        """Capture once; avoid allocating raw/JPEG payloads without subscribers."""
        try:
            frame = self.camera.get_frame()
            stamp = self.get_clock().now().to_msg()
            raw, jpeg, info = image_messages(
                frame, stamp, self.frame_id, self.calibration, self.quality,
                include_raw=self.raw_pub.get_subscription_count() > 0,
                include_compressed=self.jpeg_pub.get_subscription_count() > 0)
            self.capture_service.complete(frame, stamp, self.frame_id, self.calibration)
            self.failures = 0
        except Exception as error:
            self.failures += 1
            self.get_logger().error(f'Camera read failed ({self.failures}/3): {error}')
            if self.failures >= 3:
                raise RuntimeError(
                    'Camera failed three times; stopping without stale frames') from error
            return
        if raw is not None:
            self.raw_pub.publish(raw)
        if jpeg is not None:
            self.jpeg_pub.publish(jpeg)
        self.info_pub.publish(info)

    def destroy_node(self):
        """Release the hardware on normal shutdown, capture error or partial startup."""
        if self.capture_service is not None:
            self.capture_service.close()
            self.capture_service = None
        if self.camera is not None:
            camera, self.camera = self.camera, None
            try:
                camera.close()
            except Exception as error:
                self.get_logger().error(f'Cannot close camera: {error}')
        return super().destroy_node()


def main(args=None):
    """Run a dedicated robot process; no remote code execution is used."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = CameraPublisher()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and node.capture_service is not None:
            node.capture_service.close()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
