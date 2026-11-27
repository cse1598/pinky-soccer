"""Pure numerical checks (no ROS graph required)."""
import math
import numpy as np
import pytest
from pinky_soccer_viz.path_lqr_core import DT, Limits, plan, track, wrap


@pytest.fixture
def lim():
    return Limits(.028, .028, .0971, 9.7, 67.4, .25, 2., .028, .0971, .25, 2.)


@pytest.mark.parametrize('kind', ['straight', 'bezier', 'quintic'])
@pytest.mark.parametrize('start,goal', [
    ((0., 0., 0.), (.5, .25, .6)),
    ((0., 0., 0.), (-.4, 0., 0.)),
    ((0., 0., 0.), (0., 0., math.pi)),
    ((.1, .2, 3.1), (.1, .2, -3.1)),
    ((.1, .2, .3), (.1, .2, .3)),
])
def test_reference_constraints(lim, kind, start, goal):
    p = plan(start, goal, lim, kind)
    np.testing.assert_allclose(p.states[0, :2], start[:2], atol=1e-8)
    np.testing.assert_allclose(p.states[-1, :2], goal[:2], atol=1e-8)
    assert abs(wrap(p.states[0, 2]-start[2])) < 1e-8
    assert abs(wrap(p.states[-1, 2]-goal[2])) < 1e-8
    assert np.all(np.isfinite(p.gains))
    assert len(p.states) == len(p.inputs)+1
    assert lim.scale(p.inputs) <= 1.00001
    wheels = lim.wheels(p.inputs)
    accel = np.diff(np.vstack((np.zeros(2), wheels, np.zeros(2))), axis=0)/DT
    assert np.max(np.abs(accel)) <= lim.accel*1.00001


def test_reverse(lim):
    p = plan((0., 0., 0.), (-.4, 0., 0.), lim, 'straight')
    assert p.direction == -1
    assert np.min(p.inputs[:, 0]) < -.1


@pytest.mark.parametrize('kind', ['straight', 'bezier', 'quintic'])
def test_ideal_closed_loop(lim, kind):
    p = plan((0., 0., 0.), (.5, .25, .6), lim, kind)
    pose = np.array([.0, .008, .04])
    wheel = np.zeros(2)
    for i in range(len(p.inputs)):
        wheel = lim.command(track(p, i, pose), wheel, DT)
        v, w = lim.twist(wheel)
        mid = pose[2]+w*DT/2
        pose += DT*np.array([v*math.cos(mid), v*math.sin(mid), w])
    assert np.linalg.norm(pose[:2]-p.states[-1, :2]) < .02
    assert abs(wrap(pose[2]-p.states[-1, 2])) < math.radians(3)


def test_command_limits(lim):
    old = np.zeros(2)
    for _ in range(20):
        new = lim.command(np.array([10., -30.]), old, DT)
        assert np.max(np.abs(new-old)) <= lim.accel*DT+1e-8
        assert lim.scale(lim.twist(new)) <= 1.000001
        old = new
