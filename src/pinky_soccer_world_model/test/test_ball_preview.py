"""Exercise image callbacks without intrinsics or a TF connection."""

from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
import numpy as np
from pinky_soccer_world_model.ball_detector import BallDetector
import pytest


@pytest.mark.parametrize('ball', [True, False])
def test_uncalibrated_preview_without_metric_output(ball):
    """An uncalibrated image still gets annotated, with no invented metric point."""
    frame = np.zeros((240, 320, 3), np.uint8)
    if ball:
        cv2.circle(frame, (160, 140), 18, (0, 140, 255), -1)
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    msg.header.frame_id = 'front_camera_optical_frame'
    msg.header.stamp.sec = 42
    previews, metric_points = [], []
    node = SimpleNamespace(
        K=None, debug_pub=SimpleNamespace(publish=previews.append), bridge=bridge,
        allow_uncalibrated=False, approximate=False,
        pub=SimpleNamespace(publish=metric_points.append),
        ranges=[((5, 120, 80), (25, 255, 255))], min_area=12, min_circularity=0.4)
    # No TF member: reaching projection would fail this test.
    BallDetector.on_image(node, msg)
    assert not metric_points
    assert len(previews) == 1
    preview = previews[0]
    assert preview.header == msg.header
    assert preview.encoding == 'bgr8'
    pixels = bridge.imgmsg_to_cv2(preview, 'bgr8')
    assert np.any(pixels[:50] != frame[:50])  # status + uncalibrated banner
    if ball:
        assert tuple(pixels[140, 160]) == (0, 255, 0)  # centre marker on the ball
    else:
        assert not np.any(pixels[100:])


def test_no_preview_or_calibration_skips_processing():
    """The normal, opt-out path remains idle until intrinsics arrive."""
    BallDetector.on_image(SimpleNamespace(K=None, debug_pub=None, allow_uncalibrated=False), None)
