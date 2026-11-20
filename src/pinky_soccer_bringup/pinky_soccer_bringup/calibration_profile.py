"""Load reviewed calibration artifacts without changing official package files."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
import yaml


def read_profile(path, simulation):
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if data.get('schema_version') != 1 or data.get('human_verified') is not True:
        raise ValueError('Reviewed calibration profile schema 1 required')
    if data.get('environment') not in ('real', 'rehearsal'):
        raise ValueError('Calibration environment missing')
    if not simulation and data['environment'] != 'real':
        raise ValueError('Rehearsal profiles cannot be applied to physical hardware')
    for key, value in data.get('launch', {}).items():
        if key not in ('wheel_radius', 'wheel_separation', 'camera_info_url'):
            raise ValueError(f'Unsupported profile launch key: {key}')
        if key != 'camera_info_url' and (not math.isfinite(float(value)) or float(value) <= 0):
            raise ValueError(f'Invalid profile value: {key}')
    return data


def arguments(simulation=False):
    def apply(context):
        path = LaunchConfiguration('calibration_profile').perform(context)
        if not path:
            return []
        data = read_profile(path, simulation)
        # Simulators generate their own CameraInfo from their renderer, never substitute real K/D.
        return [SetLaunchConfiguration(key, str(value)) for key, value in
                data.get('launch', {}).items() if not simulation or key != 'camera_info_url']
    return [DeclareLaunchArgument('calibration_profile', default_value=''),
            OpaqueFunction(function=apply)]


def configure_urdf(urdf, path, simulation=False):
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
    # Keep actual collision dimensions consistent with measured effective wheel geometry.
    w = data.get('results', {}).get('wheels')
    if simulation and w:
        for side, key in [('l', 'left_radius_m'), ('r', 'right_radius_m')]:
            link = root.find(f"link[@name='{side}_wheel']")
            cylinder = link.find('collision/geometry/cylinder')
            if cylinder is not None:
                cylinder.set('radius', str(w[key]))
        for plugin in root.findall('.//plugin'):
            if 'DiffDrive' in plugin.get('name', ''):
                plugin.find('wheel_radius').text = str(data['launch']['wheel_radius'])
                plugin.find('wheel_separation').text = str(data['launch']['wheel_separation'])
    return ET.tostring(root, encoding='unicode')
