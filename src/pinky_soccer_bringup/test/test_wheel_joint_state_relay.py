"""Unit tests for the wheel joint name relay."""
from pinky_soccer_bringup.wheel_joint_state_relay import rename_joints
import pytest

FROM = ['left_wheel_joint', 'right_wheel_joint']
TO = ['l_wheel_joint', 'r_wheel_joint']


def test_renames_driver_names_to_urdf_names():
    assert rename_joints(FROM, FROM, TO) == TO


def test_keeps_order_and_unknown_names():
    names = ['right_wheel_joint', 'other', 'left_wheel_joint']
    assert rename_joints(names, FROM, TO) == ['r_wheel_joint', 'other', 'l_wheel_joint']


def test_rejects_mismatched_lists():
    with pytest.raises(ValueError):
        rename_joints(FROM, FROM, TO[:1])
