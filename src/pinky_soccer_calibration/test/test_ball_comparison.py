"""Bumper reference, correction signs, and saved review evidence."""

from types import SimpleNamespace

import numpy as np

from pinky_soccer_calibration.ball_review import compare_bumper_ball
from pinky_soccer_calibration.ball_review_view import BallReviewView
from pinky_soccer_calibration.suite_model import Session

import pytest


def test_bumper_center_offset_and_lateral_error():
    """Corrections add to raw XY after subtracting the bumper center."""
    result = compare_bumper_ball([.54, -.08, -.00665], [.04, .02, .002], [.48, -.12], [-.02, -.02])
    assert result['observed_xy_m'] == pytest.approx([.5, -.1])
    assert result['error_xy_m'] == pytest.approx([.02, .02])
    assert result['suggested_offset_xy_m'] == pytest.approx([-.02, -.02])
    assert result['corrected_error_m'] == pytest.approx(0, abs=1e-12)
    assert result['observed_range_m'] == pytest.approx(np.hypot(.5, .1))


@pytest.mark.parametrize('actual,offset', [
    ([float('nan'), 0], [0, 0]),
    ([1, 0], [float('inf'), 0]),
    ([-.1, 0], [0, 0]), ([1], [0, 0]),
])
def test_invalid_measurements_rejected(actual, offset):
    """Reject nonfinite, malformed, and rearward reference coordinates."""
    with pytest.raises(ValueError):
        compare_bumper_ball([.4, 0, 0], [.04, 0, 0], actual, offset)


def test_review_save_roundtrip_preserves_calibration_profile(tmp_path):
    """Review records survive reopening without altering launch calibration."""
    session = Session(tmp_path, 'rehearsal', 'test')
    session.results['camera_mount'] = {'height_m': {'mean': .06}}
    profile = session.export(True)
    original = profile.read_bytes()
    comparison = compare_bumper_ball([.34, 0, 0], [.04, 0, 0], [.29, 0], [-.01, 0])
    view = SimpleNamespace(session=session, current_comparison=lambda: dict(comparison),
                           is_busy=lambda: False, report={'point': [.34, 0, 0]},
                           external_path='', engine=SimpleNamespace(settings={}), on_saved=lambda: None,
                           comparison_text=SimpleNamespace(configure=lambda **kw: None))
    BallReviewView.save_comparison(view)
    assert profile.read_bytes() == original
    root = session.root
    session.release()
    reopened = Session.open(root)
    row = reopened.measurements['ball_review'][0]
    assert row['offset_xy_m'] == [-.01, 0]
    assert row['target'] == 'ball_center'
    assert reopened.recalculate('ball_review')['error_m']['mean'] == pytest.approx(.01)
    assert profile.read_bytes() == original
    assert 'ball_review' not in __import__('yaml').safe_load(reopened.export(True).read_text())['results']
    exported = profile.read_bytes()
    reopened.remove_measurements('ball_review', {row['id']})
    assert profile.read_bytes() == exported
    reopened.release()
