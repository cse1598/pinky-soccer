"""Simulator actuator defaults tied to the Pinky Pro driver and XL330 manual."""
import math
import xml.etree.ElementTree as ET

NO_LOAD_RPM = 103.
DRIVER_RPM = 100.
PROFILE_ACCEL = 74.9
# Conservative existing simulation proxies; not measured hardware parameters.
TORQUE_PROXY = .15
VELOCITY_GAIN_PROXY = .08


def configure_mujoco(physics):
    """Preserve the velocity servo; cap actuator at the 5 V no-load speed."""
    physics['motors'].update(max_speed=NO_LOAD_RPM*2*math.pi/60,
                             wheel_accel=PROFILE_ACCEL,
                             max_torque=TORQUE_PROXY,velocity_gain=VELOCITY_GAIN_PROXY)
    return physics


def configure_gazebo(urdf):
    """Remove independent body clamps: the wheel motor adapter owns the limits."""
    root = ET.fromstring(urdf)
    for plugin in root.findall('.//plugin'):
        if 'DiffDrive' in plugin.get('name',''):
            for child in list(plugin):
                if child.tag.startswith(('min_','max_')) and any(
                        k in child.tag for k in ('velocity','acceleration','jerk')):
                    plugin.remove(child)
    for joint in root.findall('joint'):
        if joint.get('name','') in ('l_wheel_joint','r_wheel_joint'):
            limit = joint.find('limit')
            if limit is not None:
                limit.set('velocity',str(NO_LOAD_RPM*2*math.pi/60))
                limit.set('effort',str(TORQUE_PROXY))
    return ET.tostring(root,encoding='unicode')
