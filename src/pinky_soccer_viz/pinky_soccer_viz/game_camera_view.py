"""Display synchronized robot view, gameplay detection and orange selection."""

import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TITLE = 'Pinky Soccer - Camera | Ball detection | Orange mask'


def compose_camera_view(panels, width=1440):
    """Render three panels without changing source pixels or stretching geometry.

    Each input is (title, BGR image or None, age seconds). Missing/stale data is
    explicit so an old annotation cannot look like a live observation.
    """
    tile_width = width//len(panels)
    sample = next((image for _, image, _ in panels if image is not None), None)
    aspect = sample.shape[0]/sample.shape[1] if sample is not None else .75
    height = max(160, min(720, round(tile_width*aspect)))
    tiles = []
    for title, frame, age in panels:
        tile = np.full((height+38, tile_width, 3), 28, np.uint8)
        label = title if frame is not None else title+' - WAITING'
        if frame is not None:
            scale = min(tile_width/frame.shape[1], height/frame.shape[0])
            size = (max(1, round(frame.shape[1]*scale)), max(1, round(frame.shape[0]*scale)))
            resized = cv2.resize(frame, size)
            h, w = resized.shape[:2]
            x, y = (tile_width-w)//2, 38+(height-h)//2
            tile[y:y+h, x:x+w] = resized
            if age > .5:
                label += f' - STALE {age:.1f}s'
        cv2.putText(tile, label, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .48,
                    (60, 60, 255) if age > .5 and frame is not None else (220, 220, 220),
                    1, cv2.LINE_AA)
        tiles.append(tile)
    return np.hstack(tiles)


class GameCameraView(Node):
    """Read-only monitor; annotations and mask come from the actual game detector."""

    def __init__(self):
        super().__init__('game_camera_view')
        self.bridge = CvBridge()
        self.width = int(self.declare_parameter('window_width', 1440).value)
        if not 600 <= self.width <= 3840:
            raise ValueError('window_width must be 600..3840')
        self.cache = {name: {} for name in ('camera', 'detection', 'mask')}
        self.subscriptions_kept = []
        for name, topic in [('camera', '/ball_detection/source_image'),
                            ('detection', '/ball_detection/debug_image'),
                            ('mask', '/ball_detection/debug_mask')]:
            self.subscriptions_kept.append(self.create_subscription(
                Image, topic, lambda msg, key=name: self.receive(key, msg),
                qos_profile_sensor_data))

    def receive(self, name, msg):
        """Keep a bounded cache keyed by exact exposure time and optical frame."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
            return
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id)
        cache = self.cache[name]
        cache[key] = frame, time.monotonic()
        while len(cache) > 12:
            del cache[next(iter(cache))]

    def render(self):
        """Prefer a recent synchronized triplet; never mix exposure timestamps."""
        now = time.monotonic()
        raw = self.cache['camera']
        common = raw.keys() & self.cache['detection'].keys() & self.cache['mask'].keys()
        matched = max(common, key=lambda key: raw[key][1]) if common else None
        if matched is not None and now-raw[matched][1] <= .5:
            selected = matched
        else:
            available = [(key, value[1]) for cache in self.cache.values()
                         for key, value in cache.items()]
            selected = max(available, key=lambda item: item[1])[0] if available else None
        panels = []
        for name, title in [('camera', 'CAMERA'), ('detection', 'BALL DETECTION'),
                            ('mask', 'ORANGE MASK')]:
            frame, seen = self.cache[name].get(selected, (None, now))
            panels.append((title, frame, now-seen))
        return compose_camera_view(panels, self.width)


def main(args=None):
    """Open the camera alongside RViz; closing this window does not stop the game."""
    rclpy.init(args=args)
    node = GameCameraView()
    opened = False
    try:
        cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(TITLE, node.width, 400)
        opened = True
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            cv2.imshow(TITLE, node.render())
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break
            if cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if opened:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
