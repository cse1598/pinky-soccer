"""Review and actual gameplay callbacks must produce identical projection results."""

from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
from pinky_soccer_calibration.ball_review import GameplayReview, gameplay_settings, select_intrinsics
from pinky_soccer_world_model.ball_detector import BallDetector, transform_matrix
from pinky_soccer_world_model.ball_filter import BallFilter
import pytest
from test_compressed_capture import connected  # Reuse a real DDS camera/client fixture.


def scene():
    frame = np.zeros((240, 320, 3), np.uint8)
    cv2.circle(frame, (205, 155), 15, (0, 140, 255), -1)
    camera = TransformStamped()
    camera.transform.translation.z = .30
    camera.transform.rotation.x = 1.  # Optical camera looking vertically down.
    camera.transform.rotation.w = 0.
    output = TransformStamped()
    output.transform.rotation.w = 1.
    def lookup(target, source, stamp):
        return transform_matrix(camera if target == 'base_footprint' else output)
    return frame, camera, output, lookup


@pytest.mark.parametrize('rotated', [False, True])
@pytest.mark.parametrize('approximate', [False, True])
def test_review_matches_game_detector_point(rotated, approximate):
    frame, camera, output, lookup = scene()
    cfg, params = gameplay_settings()
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
    msg.header.frame_id = 'camera'
    msg.header.stamp.sec = 10
    k = np.array([[450., 0, 160], [0, 450., 120], [0, 0, 1]])
    d = np.array([.02, -.01, 0, 0, 0])
    points = []
    node = SimpleNamespace(K=None if approximate else k, D=d, approximate=False,
        allow_uncalibrated=approximate, approximate_hfov=66.,
        debug_pub=None, bridge=bridge, pub=SimpleNamespace(publish=points.append),
        ranges=cfg['ranges'], min_area=cfg['min_area_px'], min_circularity=cfg['min_circularity'],
        rotated=rotated, floor_frame='base_footprint', output_frame='base_link',
        radius=cfg['ball_radius'], max_range=cfg['max_range'],
        tf=SimpleNamespace(lookup_transform=lambda target, *a, **kw:
                           camera if target == 'base_footprint' else output))
    BallDetector.on_image(node, msg)
    engine = GameplayReview(cfg, params)
    external = None if approximate else (k, d, (320, 240))
    preview, report = engine.process(frame, 1, 10., 10., msg.header, lookup,
                                     external=external, approximate=approximate, rotated=rotated)
    assert len(points) == 1
    expected = points[0].point
    assert report['point'] == pytest.approx([expected.x, expected.y, expected.z])
    assert report['approximate'] == approximate
    assert report['tracking']['state'] == 'OBSERVED'
    assert isinstance(engine.filter, BallFilter)
    assert tuple(preview[155, 205]) == (0, 255, 0)


def test_project_calibration_precedence_and_resolution_mismatch():
    frame, _, _, _ = scene()
    k = np.array([[450., 0, 160], [0, 450., 120], [0, 0, 1]])
    session = SimpleNamespace(camera=(k, np.zeros(5), (320, 240), 30),
                              results={'camera': {'passed': True}}, stale=set())
    result = select_intrinsics(frame.shape, session, None, None, True, 66)
    assert np.array_equal(result[0], k) and not result[3]
    session.camera = k, np.zeros(5), (640, 480), 30
    with pytest.raises(ValueError, match='해상도'):
        select_intrinsics(frame.shape, session, None, None, True, 66)
    session.stale.add('camera')
    assert select_intrinsics(frame.shape, session, None, None, True, 66)[3]
    assert select_intrinsics(frame.shape, session, None, None, False, 66)[0] is None


def test_review_excludes_outside_area_using_gameplay_settings():
    """Review uses the gameplay ROI and shades it without changing the input."""
    frame, _, _, lookup = scene()
    frame[:] = 100
    cv2.circle(frame, (160, 40), 20, (0, 140, 255), -1)
    original = frame.copy()
    cfg, params = gameplay_settings()
    engine = GameplayReview(cfg, params)
    preview, report = engine.process(
        frame, 1, 10., 10., SimpleNamespace(frame_id='camera', stamp=None), lookup)
    assert report['blob'] is None and report['point'] is None
    assert np.array_equal(frame, original)
    assert tuple(preview[10, 300]) == (35, 35, 35)
    assert np.array_equal(preview[220, 300], frame[220, 300])


def test_missing_tf_and_lost_ball_never_reuse_old_observation():
    frame, _, _, lookup = scene()
    header = SimpleNamespace(frame_id='camera', stamp=None)
    engine = GameplayReview()
    _, report = engine.process(frame, 1, 10., 10., header, lambda *args: None)
    assert report['blob'] and report['point'] is None and report['tracking'] is None
    engine.reset()
    _, report = engine.process(frame, 1, 10., 10., header, lookup)
    assert report['point'] is not None
    # Repainting the same received frame must not create another filter observation.
    _, report = engine.process(frame, 1, 10., 10.4, header, lookup)
    assert engine.filter.last_seen == 10. and report['tracking']['state'] == 'ESTIMATED'
    blank = np.zeros_like(frame)
    _, report = engine.process(blank, 2, 10.5, 10.5, header, lookup)
    assert report['blob'] is None and report['point'] is None
    _, report = engine.process(blank, 3, 16., 16., header, lookup)
    assert report['tracking'] is None and report['tracking_reason'] == 'LOST'


def test_no_stable_frame_does_not_filter_in_moving_base_frame():
    frame, camera, output, _ = scene()
    def lookup(target, source, stamp):
        if target in ('map', 'odom'):
            return None
        return transform_matrix(camera if target == 'base_footprint' else output)
    _, report = GameplayReview().process(frame, 1, 1., 1.,
        SimpleNamespace(frame_id='camera', stamp=None), lookup)
    assert report['point'] is not None and report['tracking'] is None


def test_live_camera_info_overrides_approximation_and_clock_rewind_resets():
    frame, _, _, lookup = scene()
    info = SimpleNamespace(k=[500., 0, 160, 0, 500., 120, 0, 0, 1], d=[0.]*5,
                           width=320, height=240, distortion_model='plumb_bob')
    engine = GameplayReview()
    header = SimpleNamespace(frame_id='camera', stamp=None)
    _, report = engine.process(frame, 1, 10., 10., header, lookup, info=info)
    assert not report['approximate'] and 'CameraInfo' in report['intrinsics_source']
    _, report = engine.process(frame, 2, 1., 1., header, lookup, info=info)
    assert report['tracking']['state'] == 'OBSERVED' and engine.filter.last_seen == 1.


def test_game_camera_loader_supports_project_paths_and_rejects_cycles(tmp_path):
    import yaml
    from pinky_soccer_world_model.ball_detector import load_camera_yaml
    folder = tmp_path/'카메라 보정'
    folder.mkdir()
    camera = folder/'camera.yaml'
    camera.write_text(yaml.safe_dump({'camera_matrix': {'data': [500., 0, 160, 0, 500., 120, 0, 0, 1]},
                                     'image_width': 320, 'image_height': 240}))
    profile = tmp_path/'profile.yaml'
    profile.write_text(yaml.safe_dump({'launch': {'camera_info_url': camera.as_uri()}}))
    assert load_camera_yaml(profile)[0][0, 0] == 500.
    profile.write_text(yaml.safe_dump({'launch': {'camera_info_url': 'profile.yaml'}}))
    with pytest.raises(ValueError, match='Circular'):
        load_camera_yaml(profile)


def test_actual_jpeg_and_tf_feed_review_without_raw_subscription(connected):
    import time
    from tf2_ros import StaticTransformBroadcaster
    client, camera, source = connected
    cv2.circle(source, (500, 340), 18, (0, 140, 255), -1)
    transforms = []
    for parent, child, z, down in [('odom', 'base_footprint', 0., False),
                                  ('base_footprint', 'base_link', .027, False),
                                  ('base_footprint', 'front_camera_optical_frame', .3, True)]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.translation.z = z
        tf.transform.rotation.x = 1. if down else 0.
        tf.transform.rotation.w = 0. if down else 1.
        transforms.append(tf)
    broadcaster = StaticTransformBroadcaster(camera)
    broadcaster.sendTransform(transforms)
    engine = GameplayReview()
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        frame, header, info, key = client.review_frame()
        now = client.get_clock().now().nanoseconds * 1e-9
        observed = header.stamp.sec + header.stamp.nanosec * 1e-9
        _, report = engine.process(frame, key, observed, now, header,
                                   client.review_transform, info=info)
        if report['point'] and report['tracking']:
            break
        time.sleep(.03)
    assert report['point'] is not None and report['approximate']
    assert report['tracking']['stable_frame'] == 'odom'
    assert camera.raw_pub.get_subscription_count() == 0
