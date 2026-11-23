"""Simulator model additions and shared, explicit sensor assumptions."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

RANGE_LINKS = ('ultrasonic_link', 'ir_l_link', 'ir_mid_link', 'ir_r_link')


def load_config(path):
    """Reject invalid model settings before spawning any simulation nodes."""
    cfg = yaml.safe_load(Path(path).read_text())
    cam = cfg['camera']
    for key in ('width', 'height'):
        if not isinstance(cam[key], int) or cam[key] <= 0:
            raise ValueError(f'camera {key} must be a positive integer')
    if not 0 < cam['horizontal_fov'] < math.pi or not 0 < cam['fps'] <= 100:
        raise ValueError('Invalid camera FOV/rate')
    if not 1 <= cam['jpeg_quality'] <= 100:
        raise ValueError('Invalid JPEG quality')
    for key in ('ultrasonic', 'ir'):
        item = cfg[key]
        if not 0 < item['min_range'] < item['max_range'] < math.inf:
            raise ValueError(f'Invalid {key} range')
    us = cfg['ultrasonic']
    if (not 0 < us['field_of_view'] < math.pi or not isinstance(us['samples'], int)
            or us['samples'] < 2):
        raise ValueError('Invalid ultrasonic beam')
    x, y = cfg['ir']['distances'], cfg['ir']['adc_values']
    if x or y:
        if (len(x) != len(y) or len(x) < 2 or not np.isfinite(x).all()
                or not np.isfinite(y).all() or np.any(np.diff(x) <= 0)
                or min(x) <= 0 or min(y) < 0 or max(y) > 4095):
            raise ValueError('IR requires increasing positive distances and 12-bit ADC samples')
    batt = cfg['battery']
    if not 0 < batt['voltage'] < math.inf or not 0 <= batt['percent'] <= 100:
        raise ValueError('Invalid simulated battery state')
    return cfg


def adc_from_distance(distance, cfg):
    """Interpolate a measured ADC table, saturating at its measured endpoints."""
    if not cfg['distances']:
        raise ValueError('IR ADC calibration table is missing')
    if math.isnan(distance) or distance < 0:
        raise ValueError('Invalid IR observation')
    return int(round(float(np.interp(distance, cfg['distances'], cfg['adc_values']))))


def configure_gazebo(urdf, cfg, camera=True, imu=False, adc=False):
    """Extend expanded wrapper URDF, leaving all official source files untouched."""
    robot = ET.fromstring(urdf)
    for gazebo in robot.findall('gazebo'):
        for sensor in list(gazebo.findall('sensor')):
            # All measured errors are injected by SensorAdapter in both engines.
            # Remove the official 20 mm lidar Gaussian rather than adding noise twice.
            for parent in sensor.iter():
                for noise in list(parent.findall('noise')):
                    parent.remove(noise)
            if sensor.get('type') == 'camera':
                if not camera:
                    gazebo.remove(sensor)
                    continue
                c = cfg['camera']
                sensor.find('update_rate').text = str(c['fps'])
                sensor.find('camera/image/width').text = str(c['width'])
                sensor.find('camera/image/height').text = str(c['height'])
                sensor.find('camera/horizontal_fov').text = str(c['horizontal_fov'])
                ET.SubElement(sensor.find('camera/image'), 'format').text = 'R8G8B8'
            if sensor.get('type') == 'imu':
                if not imu:
                    gazebo.remove(sensor)
                else:
                    sensor.find('topic').text = 'sim/imu'
    if adc:
        for link in RANGE_LINKS:
            us = link == 'ultrasonic_link'
            item = cfg['ultrasonic' if us else 'ir']
            gazebo = ET.SubElement(robot, 'gazebo', reference=link)
            sensor = ET.SubElement(gazebo, 'sensor', name=f'{link}_rays', type='gpu_lidar')
            ET.SubElement(sensor, 'always_on').text = '1'
            ET.SubElement(sensor, 'update_rate').text = '20'
            ET.SubElement(sensor, 'topic').text = f'sim/{link}/scan'
            ET.SubElement(sensor, 'gz_frame_id').text = link
            lidar = ET.SubElement(sensor, 'lidar')
            horizontal = ET.SubElement(ET.SubElement(lidar, 'scan'), 'horizontal')
            ET.SubElement(horizontal, 'samples').text = str(item['samples'] if us else 1)
            ET.SubElement(horizontal, 'resolution').text = '1'
            half = item['field_of_view'] / 2 if us else 0
            ET.SubElement(horizontal, 'min_angle').text = str(-half)
            ET.SubElement(horizontal, 'max_angle').text = str(half)
            bounds = ET.SubElement(lidar, 'range')
            for key, value in [('min', item['min_range']), ('max', item['max_range']),
                               ('resolution', 0.001)]:
                ET.SubElement(bounds, key).text = str(value)
    return ET.tostring(robot, encoding='unicode')


def add_mujoco_sensors(scene_path):
    """Add native IMU and sensor sites at existing official URDF link frames."""
    tree = ET.parse(scene_path)
    root = tree.getroot()
    sensors = root.find('sensor')
    if sensors is None:
        sensors = ET.SubElement(root, 'sensor')
    for link in ('imu_link', *RANGE_LINKS):
        body = root.find(f".//body[@name='{link}']")
        if body is None:
            raise ValueError(f'Missing sensor body {link}')
        ET.SubElement(body, 'site', name=f'soccer_{link}', size='0.001', rgba='0 0 0 0')
    for tag, name in [('accelerometer', 'acc'), ('gyro', 'gyro')]:
        ET.SubElement(sensors, tag, name=f'soccer_{name}', site='soccer_imu_link')
    ET.SubElement(sensors, 'framequat', name='soccer_quat', objtype='site',
                  objname='soccer_imu_link')
    tree.write(scene_path, encoding='unicode')
    return scene_path


def ray_ranges(physics, link, cfg):
    """Cast from the sensor origin against collision geometry, including moving balls."""
    item = cfg['ultrasonic' if link == 'ultrasonic_link' else 'ir']
    angles = np.linspace(-item['field_of_view'] / 2, item['field_of_view'] / 2,
                         item['samples']) if link == 'ultrasonic_link' else [0.0]
    site = physics.model.site(f'soccer_{link}').id
    rotation = physics.data.site_xmat[site].reshape(3, 3)
    group = np.array([1, 1, 0, 0, 1, 0], dtype=np.uint8)
    result = []
    for angle in angles:
        direction = rotation @ np.array([math.cos(angle), math.sin(angle), 0.0])
        distance = physics.mj.mj_ray(
            physics.model, physics.data, physics.data.site_xpos[site],
            direction, group, True, -1, np.zeros(1, dtype=np.int32))
        result.append(float('inf') if distance < 0 or distance > item['max_range']
                      else float('-inf') if distance < item['min_range'] else float(distance))
    return result
