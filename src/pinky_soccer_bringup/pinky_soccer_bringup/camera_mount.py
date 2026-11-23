"""Apply measured floor height and tilt while retaining unmeasured model XY/yaw."""

import math

import numpy as np


OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def rotation_rpy(roll, pitch, yaw):
    """Return a fixed-axis RPY rotation."""
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def mount_values(summary):
    """Validate measured optical-centre height and mechanical roll/pitch."""
    height, roll, pitch = [float(summary[key]['mean'])
                           for key in ('height_m', 'roll_rad', 'pitch_rad')]
    if (not np.isfinite([height, roll, pitch]).all() or not .01 < height < 3
            or abs(roll) >= math.pi/2 or abs(pitch) >= math.pi/2):
        raise ValueError('카메라 장착 높이/각도 보정값을 확인하세요.')
    return height, roll, pitch


def apply_mount(camera_rt, summary):
    """Replace height/tilt in floor-to-optical TF; never mutate the input TF."""
    height, roll, pitch = mount_values(summary)
    rotation, origin = camera_rt
    mechanical = rotation @ OPTICAL.T
    yaw = math.atan2(mechanical[1, 0], mechanical[0, 0])
    corrected = np.array(origin, dtype=float, copy=True)
    corrected[2] = height
    return rotation_rpy(roll, pitch, yaw) @ OPTICAL, corrected


def fixed_transforms(root, reference):
    """Compose fixed URDF joints from a reference frame."""
    transforms = {reference: np.eye(4)}
    pending = list(root.findall("joint[@type='fixed']"))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None
                                else '0 0 0', sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None
                                else '0 0 0', sep=' ')
            transform = np.eye(4)
            transform[:3, :3] = rotation_rpy(*rpy)
            transform[:3, 3] = xyz
            transforms[joint.find('child').get('link')] = transforms[parent] @ transform
            pending.remove(joint)
            progress = True
        if not progress:
            break
    return transforms
