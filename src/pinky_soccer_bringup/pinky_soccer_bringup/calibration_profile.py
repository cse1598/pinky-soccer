"""Load reviewed calibration artifacts without changing official package files."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
import yaml

from .camera_mount import apply_mount, fixed_transforms, mount_values, OPTICAL


def selected(path):
    """Return the profile path, or '' when calibration is disabled ('' or 'none')."""
    path = str(path or '').strip()
    return '' if path.lower() in ('', 'none') else path


def read_profile(path, simulation):
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if data.get('schema_version') != 1 or data.get('human_verified') is not True:
        raise ValueError('Reviewed calibration profile schema 1 required')
    if data.get('environment') not in ('real', 'rehearsal'):
        raise ValueError('Calibration environment missing')
    if not simulation and data['environment'] != 'real':
        raise ValueError('Rehearsal profiles cannot be applied to physical hardware')
    mount = data.get('camera_mount', data.get('results', {}).get('camera_mount'))
    if mount:
        mount_values(mount)
    for key, value in data.get('launch', {}).items():
        if key not in ('wheel_radius', 'wheel_separation', 'camera_info_url'):
            raise ValueError(f'Unsupported profile launch key: {key}')
        if key != 'camera_info_url' and (not math.isfinite(float(value)) or float(value) <= 0):
            raise ValueError(f'Invalid profile value: {key}')
    return data


def arguments(simulation=False, default=''):
    """Declare calibration_profile (default: none, or a simulator's own profile)."""
    def apply(context):
        path = selected(LaunchConfiguration('calibration_profile').perform(context))
        if not path:
            return []
        data = read_profile(path, simulation)
        # Simulators generate their own CameraInfo from their renderer, never substitute real K/D.
        return [SetLaunchConfiguration(key, str(value)) for key, value in
                data.get('launch', {}).items() if not simulation or key != 'camera_info_url']
    return [DeclareLaunchArgument('calibration_profile', default_value=str(default),
                                  description="profile.yaml; 'none' disables calibration"),
            OpaqueFunction(function=apply)]


def drive_constants(path, simulation=False):
    """Return (wheel_radius, wheel_separation) from a profile's launch block, or Nones."""
    path = selected(path)
    if not path:
        return None, None
    launch = read_profile(path, simulation).get('launch', {})
    radius, separation = launch.get('wheel_radius'), launch.get('wheel_separation')
    return (None if radius is None else float(radius),
            None if separation is None else float(separation))


def configure_urdf(urdf, path, simulation=False):
    path = selected(path)
    if not path:
        return urdf
    data = read_profile(path, simulation)
    root = ET.fromstring(urdf)
    pose = data.get('camera_pose')
    if pose is not None:
        if len(pose) != 6 or not all(math.isfinite(float(v)) for v in pose):
            raise ValueError('Camera pose requires XYZ/RPY finite values')
        joint = root.find("joint[@name='front_camera_fixed_joint']")
        joint.find('parent').set('link', 'base_link')
        origin = joint.find('origin')
        if origin is None:
            origin = ET.SubElement(joint, 'origin')
        origin.set('xyz', ' '.join(map(str, pose[:3])))
        origin.set('rpy', ' '.join(map(str, pose[3:])))
    else:
        mount = data.get('camera_mount', data.get('results', {}).get('camera_mount'))
        if mount:
            import numpy as np
            transforms = fixed_transforms(root, 'base_footprint')
            if not {'base_link', 'front_camera_link'} <= transforms.keys():
                raise ValueError('Camera mount requires floor/base/camera fixed joints')
            camera = transforms['front_camera_link']
            rotation, xyz = apply_mount((camera[:3, :3] @ OPTICAL, camera[:3, 3]), mount)
            corrected = np.eye(4)
            corrected[:3, :3], corrected[:3, 3] = rotation @ OPTICAL.T, xyz
            corrected = np.linalg.inv(transforms['base_link']) @ corrected
            r = corrected[:3, :3]
            angles = [math.atan2(r[2, 1], r[2, 2]),
                      math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0])),
                      math.atan2(r[1, 0], r[0, 0])]
            joint = next(j for j in root.findall('joint')
                         if j.find('child').get('link') == 'front_camera_link')
            joint.find('parent').set('link', 'base_link')
            origin = joint.find('origin')
            if origin is None:
                origin = ET.SubElement(joint, 'origin')
            origin.set('xyz', ' '.join(map(str, corrected[:3, 3])))
            origin.set('rpy', ' '.join(map(str, angles)))
    # Wheel calibration changes only the drive constants (cmd_vel -> wheel speed conversion
    # and encoder odometry), exactly like the physical driver's wheel_radius/wheel_separation.
    # The wheel collision geometry is the simulated physics and is never changed, otherwise a
    # simulator could not be calibrated against itself (2026-11-24 decision).
    w = data.get('results', {}).get('wheels')
    if simulation and w and 'wheel_radius' in data.get('launch', {}):
        for plugin in root.findall('.//plugin'):
            if 'DiffDrive' in plugin.get('name', ''):
                plugin.find('wheel_radius').text = str(data['launch']['wheel_radius'])
                plugin.find('wheel_separation').text = str(data['launch']['wheel_separation'])
    return ET.tostring(root, encoding='unicode')
