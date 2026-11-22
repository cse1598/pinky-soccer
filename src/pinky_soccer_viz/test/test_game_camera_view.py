"""Image monitor layout, exact-frame matching and stale/missing data handling."""

from types import SimpleNamespace
import time

import numpy as np
from pinky_soccer_viz.game_camera_view import compose_camera_view, GameCameraView


def image(value):
    return np.full((120, 160, 3), value, np.uint8)


def test_monitor_has_three_panels_and_preserves_input():
    raw, detection, mask = image(50), image(100), image(200)
    before = raw.copy()
    canvas = compose_camera_view([('CAMERA', raw, 0), ('DETECTION', detection, 0),
                                  ('MASK', mask, 0)], 960)
    assert canvas.shape == (278, 960, 3)
    assert canvas[100, 100, 0] == 50
    assert canvas[100, 420, 0] == 100
    assert canvas[100, 740, 0] == 200
    assert np.array_equal(raw, before)
    pending = compose_camera_view([('CAMERA', None, 0), ('DETECTION', None, 0),
                                   ('MASK', None, 0)], 960)
    assert pending.shape == canvas.shape


def test_monitor_never_pairs_new_image_with_old_annotation():
    now = time.monotonic()
    old, current = (1, 0, 'optical'), (2, 0, 'optical')
    cache = {'camera': {old: (image(10), now-1), current: (image(50), now)},
             'detection': {old: (image(100), now-1)}, 'mask': {old: (image(200), now-1)}}
    canvas = GameCameraView.render(SimpleNamespace(cache=cache, width=960))
    assert canvas[100, 100, 0] == 50
    assert canvas[100, 420, 0] == 28  # waiting, not the previous detection
    cache['detection'][current], cache['mask'][current] = (image(150), now), (image(250), now)
    canvas = GameCameraView.render(SimpleNamespace(cache=cache, width=960))
    assert canvas[100, 420, 0] == 150 and canvas[100, 740, 0] == 250
