"""Control law checks for reactive_controller (no ROS graph needed)."""

import math

from pinky_soccer.reactive_controller import compute_command, Gains
import pytest


def test_drives_forward_to_target_ahead():
    v, w = compute_command((0, 0, 0), (1.0, 0, 0), Gains())
    assert v == pytest.approx(Gains().max_lin)
    assert w == pytest.approx(0.0)


def test_turns_in_place_when_target_behind():
    v, w = compute_command((0, 0, 0), (-1.0, 0.1, 0), Gains())
    assert v == 0.0 and w > 0


def test_turns_to_heading_when_at_position():
    v, w = compute_command((0, 0, 0), (0.01, 0, math.pi / 2), Gains())
    assert v == 0.0 and w > 0
    assert compute_command((0, 0, 0), (0.0, 0, 0.01), Gains()) == (0.0, 0.0)


def test_respects_limits_and_goal_speed():
    g = Gains()
    v, w = compute_command((0, 0, 0), (5.0, 0.5, 0), g, max_speed=0.1)
    assert 0 < v <= 0.1
    assert abs(w) <= g.max_ang
    assert compute_command((0, 0, 0), (0, 0.0001, 3.0), g)[1] == pytest.approx(g.max_ang)


def test_slows_down_near_target():
    v_far, _ = compute_command((0, 0, 0), (1.0, 0, 0), Gains())
    v_near, _ = compute_command((0, 0, 0), (0.1, 0, 0), Gains())
    assert v_near < v_far
