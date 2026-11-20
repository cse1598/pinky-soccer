"""Regression checks for measured fitting, immutable provenance and ROS watchdog."""

import math
import time

import numpy as np
import pytest
from pinky_soccer_calibration.suite_model import Session, scalar_report, wheel_fit


def trials():
    result = []
    for left,right in [(40,40),(-40,-40),(-12,12),(12,-12),(39,40),(-39,-40)]:
        result.append(dict(left_rad=left,right_rad=right,
                           distance_m=(.027*left+.028*right)/2,
                           angle_rad=(.028*right-.027*left)/.0961))
    return result


def test_wheel_fit_recovers_asymmetric_radii():
    result = wheel_fit(trials())
    assert result['left_radius_m'] == pytest.approx(.027)
    assert result['right_radius_m'] == pytest.approx(.028)
    assert result['separation_m'] == pytest.approx(.0961)
    with pytest.raises(ValueError):
        wheel_fit(trials()[:2])


def test_missing_measurements_and_unreviewed_export_rejected(tmp_path):
    session = Session(tmp_path,'rehearsal','test')
    with pytest.raises(ValueError):
        session.export(True)
    with pytest.raises(ValueError):
        scalar_report([float('nan')]*20, 1)
    report = scalar_report([1.1]*20, 1)
    assert report['bias'] == pytest.approx(.1)
    session.results['wheels'] = wheel_fit(trials())
    with pytest.raises(ValueError):
        session.export(False)
    assert session.export(True).exists()


def test_ros_watchdog_stops_on_stale_odometry(tmp_path, monkeypatch):
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from pinky_soccer_calibration.suite_ros import Acquisition
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    rclpy.init(args=[], domain_id=87)
    node = Acquisition()
    try:
        session = Session(tmp_path,'rehearsal','watchdog')
        node.begin(session)
        msg = Odometry()
        msg.pose.pose.orientation.w = 1.
        joints = JointState(name=['l_wheel_joint','r_wheel_joint'],position=[0.,0.])
        node.receive('/odom',msg)
        node.receive('/joint_states',joints)
        node.start_motion('forward')
        time.sleep(.8)
        assert node.motion is None
        assert node.last_motion['reason'] == '센서 수신 중단'
    finally:
        node.close()
        rclpy.shutdown()


def test_rehearsal_profile_cannot_drive_real_launch(tmp_path):
    from pinky_soccer_bringup.calibration_profile import read_profile, configure_urdf
    session = Session(tmp_path, 'rehearsal', 'test')
    session.results['extrinsics'] = [.02, 0, .1, 0, .1, 0]
    path = session.export(True)
    with pytest.raises(ValueError, match='Rehearsal'):
        read_profile(path, False)
    urdf = '<robot><joint name="front_camera_fixed_joint"><parent link="mount"/></joint></robot>'
    result = configure_urdf(urdf, path, True)
    assert 'base_link' in result
    assert '0.02 0 0.1' in result
