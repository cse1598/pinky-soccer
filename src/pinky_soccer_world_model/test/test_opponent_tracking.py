"""Sparse scans, measurement-time tracking and independent avoidance regression."""
import numpy as np
import pytest
from pinky_soccer_field.field_geometry import Rect
from pinky_soccer_world_model.opponents import (
    Candidate, SingleOpponentTracker, indexed_candidates)


def candidate(x=.4, y=.1, count=4):
    return Candidate(x, y, .07, count)


def confirmed(**kwargs):
    tr = SingleOpponentTracker(**kwargs)
    for t in (1., 1.1, 1.2):
        tr.update([candidate()], t)
    return tr


def extract(r, amin=-.5, inc=.01, **kwargs):
    return indexed_candidates(r, amin, inc, .02, 10., np.eye(3), np.zeros(3),
                              Rect(-2, -2, 2, 2), **kwargs)


def test_three_hits_in_five_not_publication_ticks():
    tr = SingleOpponentTracker()
    tr.update([candidate()], 1)
    for _ in range(100):
        assert tr.sample(1.01) is None
        tr.update([candidate()], 1)  # duplicate timestamp
    tr.update([], 1.1)
    tr.update([candidate()], 1.2)
    tr.update([], 1.3)
    assert tr.sample(1.3) is None
    tr.update([candidate()], 1.4)
    assert tr.sample(1.4)['state'] == 0
    assert not tr.velocity_valid


def test_no_opponent_and_transient_noise():
    tr = SingleOpponentTracker()
    for i in range(30):
        tr.update([candidate()] if i == 5 else [], 1+i*.1)
        assert tr.sample(1+i*.1) is None


def test_order_stability_impossible_jump_and_reacquisition():
    tr = confirmed()
    for i in range(8):
        items = [candidate(.4+i*.005), candidate(-1, -1)]
        tr.update(items[::(-1 if i % 2 else 1)], 1.3+i*.1)
        assert np.linalg.norm(tr.sample(1.3+i*.1)['position']-[.4+i*.005, .1]) < .02
    last = tr.last_seen
    assert not tr.update([candidate(-1, -1)], 2.1)
    assert tr.last_seen == last
    assert tr.sample(2.1)['state'] == 1
    tr.update([candidate(.44)], 2.2)
    assert tr.last_seen == 2.2
    assert tr.sample(3.3) is None


@pytest.mark.parametrize('speed', [-.2, 0., .2])
def test_alpha_beta_speed_forward_reverse_stationary(speed):
    tr = SingleOpponentTracker()
    for i in range(90):
        t = 1+i*.1
        tr.update([candidate(.2+speed*(t-1))], t)
    sample = tr.sample(t)
    assert sample['velocity'][0] == pytest.approx(speed, abs=.015)
    assert sample['position'][0] == pytest.approx(.2+speed*(t-1), abs=.01)
    before = tr.position.copy()
    for _ in range(40):
        tr.sample(t+.02)
    np.testing.assert_array_equal(tr.position, before)


def test_stationary_noise_velocity_and_reset():
    rng = np.random.default_rng(41)
    tr = SingleOpponentTracker()
    speeds = []
    for i in range(150):
        t = 1+i*.1
        tr.update([candidate(.4+rng.normal(0, .002), .1+rng.normal(0, .002))], t)
        sample = tr.sample(t)
        if sample:
            speeds.append(np.linalg.norm(sample['velocity']))
    assert max(speeds) < .02
    tr.reset()
    assert tr.sample(t) is None


def test_invalid_duplicate_and_late_measurements():
    tr = confirmed()
    before = tr.position.copy()
    for stamp in (0., float('nan'), 1.2, 1.1):
        assert not tr.update([candidate(1.)], stamp)
    np.testing.assert_array_equal(before, tr.position)


@pytest.mark.parametrize('bad', [dict(alpha=0), dict(beta=2), dict(confirm_hits=6),
                                dict(hold_sec=0), dict(max_speed=-1)])
def test_parameter_validation(bad):
    with pytest.raises(ValueError):
        SingleOpponentTracker(**bad)


def test_sparse_three_points_and_tiny_noise():
    assert len(extract([.7]*3, inc=.04)) == 1
    assert not extract([.7]*2, inc=.04)
    assert not extract([.7]*3, inc=.001)


def test_index_holes_do_not_connect_unrelated_groups():
    assert not extract([.7, .7]+[np.inf]*30+[.7, .7], inc=.001)
    # Only two missing beams, close surface fragments can join.
    assert len(extract([.7, .7, np.inf, .7, .7], inc=.01)) == 1


def test_full_scan_wrap_but_not_partial_scan():
    r = np.full(360, np.inf)
    r[:2] = r[-2:] = .7
    assert len(extract(r, amin=-np.pi, inc=2*np.pi/360)) == 1
    assert not extract(r[:180], amin=-np.pi, inc=2*np.pi/360)


def test_walls_structure_and_self_returns():
    assert not extract([1.98]*9, amin=-.02, inc=.005)
    assert not extract([.05]*9)
    assert not extract([.7]*9, structures=[Rect(.5, -.5, .9, .5)])
    assert len(extract([1.85]*5, amin=-.01, inc=.005)) == 1  # close to wall


def test_map_frame_cancels_self_translation():
    # Same fixed surface, different moving sensor origins, no truth enters tracker.
    tr = SingleOpponentTracker()
    for i in range(50):
        sensor_x = i*.002
        ranges = [1.-sensor_x]*5
        c = indexed_candidates(ranges, -.02, .01, .05, 10., np.eye(3),
                               np.array([sensor_x, 0., .12]), Rect(-2, -2, 2, 2))
        tr.update(c, 1+i*.1)
    assert np.linalg.norm(tr.sample(5.9)['velocity']) < .02
