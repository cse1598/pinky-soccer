"""Referee goal geometry and the obstacle shuttle (no ROS graph)."""

import math

from pinky_soccer_field.field_geometry import Field
from pinky_soccer_viz.game_referee import crossed
from pinky_soccer_viz.obstacle_driver import Shuttle


def test_goal_needs_the_whole_ball_past_the_line():
    field = Field.from_file(None, 'yellow')
    line, r = field.half_length, 0.02135
    assert not crossed(field, line + 0.01, 0.0, 'blue', r)        # centre past, ball not
    assert crossed(field, line + r + 0.001, 0.0, 'blue', r)
    assert not crossed(field, line + 0.05, field.spec['goal_width'], 'blue', r)  # beside post
    assert crossed(field, -(line + r + 0.001), 0.0, 'yellow', r)
    assert not crossed(field, -(line + r + 0.001), 0.0, 'blue', r)


def simulate(shuttle, delay_steps, steps=4000, dt=0.01):
    """Drive a delayed unicycle along x with the shuttle; return the swing after settling."""
    shuttle.begin(0.0, 0.0, 0.0)
    x, queue, xs = 0.0, [0.0] * delay_steps, []
    for k in range(steps):
        queue.append(shuttle.command(x, 0.0))
        x += queue.pop(0) * dt
        if k > steps // 2:
            xs.append(x)
    return min(xs), max(xs)


def test_shuttle_swing_converges_to_travel_despite_delay():
    for delay in (0, 10, 35):                       # 0, 0.1 s, 0.35 s
        lo, hi = simulate(Shuttle(0.15, 0.5), delay)
        assert math.isclose(hi, 0.25, abs_tol=0.012), (delay, hi)
        assert math.isclose(lo, -0.25, abs_tol=0.012), (delay, lo)


def test_shuttle_speed_and_direction():
    s = Shuttle(-0.2, 0.4)                          # sign of speed does not matter
    s.begin(1.0, 2.0, math.pi / 2)                  # heading +y
    assert s.command(1.0, 2.0) == 0.2
    assert s.command(1.0, 2.21) == -0.2              # past +travel/2 along +y: reverse


def test_shuttle_steers_back_onto_its_line():
    s = Shuttle(0.1, 0.4)
    s.begin(0.0, 0.0, 0.0)                          # line along +x
    assert s.steer(0.1, 0.05, 0.0, 0.1) < 0         # left of the line going forward: right
    assert s.steer(0.1, 0.05, 0.0, -0.1) > 0        # same offset while reversing
    assert s.steer(0.1, 0.0, 0.2, 0.1) < 0          # turned left: turn back
