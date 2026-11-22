"""Convert accessible bumper measurements using the running robot's fixed geometry."""

import hashlib
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def origin(element):
    """Return a URDF origin transform (fixed-axis roll, pitch, yaw)."""
    result = np.eye(4)
    if element is None:
        return result
    xyz = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    rpy = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
    if xyz.size != 3 or rpy.size != 3 or not np.isfinite(np.r_[xyz, rpy]).all():
        raise ValueError('URDF origin 좌표가 올바르지 않습니다.')
    r, p, y = rpy
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                      [-sp, cp*sr, cp*cr]]
    result[:3, 3] = xyz
    return result


def mesh_vertices(uri):
    """Read STL vertices without assuming the bumper origin is its box centre."""
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package, relative = uri[len('package://'):].split('/', 1)
        path = Path(get_package_share_directory(package)) / relative
    else:
        path = Path(uri.removeprefix('file://'))
        if not path.is_absolute():
            raise ValueError('범퍼 메시 경로는 package:// 또는 절대 경로여야 합니다.')
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84 + 50*count:
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                          ('attribute', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices']
        return vertices.reshape(-1, 3).astype(float)
    vertices = [list(map(float, line.split()[1:]))
                for line in data.decode('ascii').splitlines()
                if line.strip().startswith('vertex ')]
    if not vertices:
        raise ValueError('범퍼 STL 정점을 읽을 수 없습니다.')
    return np.asarray(vertices)


class MeasurementGeometry:
    """Static base_link transforms and the front outer face from robot_description."""

    def __init__(self, description):
        root = ET.fromstring(description)
        self.description = description
        self.sha256 = hashlib.sha256(description.encode()).hexdigest()
        self.transforms = {'base_link': np.eye(4)}
        pending = list(root.findall('joint'))
        while pending:
            progress = False
            for joint in pending[:]:
                parent = joint.find('parent').get('link')
                if parent in self.transforms and joint.get('type') == 'fixed':
                    self.transforms[joint.find('child').get('link')] = (
                        self.transforms[parent] @ origin(joint.find('origin')))
                    pending.remove(joint)
                    progress = True
            if not progress:
                break
        link = root.find("link[@name='bumper_link']")
        if link is None or 'bumper_link' not in self.transforms:
            raise ValueError('실행 모델에 범퍼가 없습니다. bumper_enabled 설정을 확인하세요.')
        points = []
        # Visual geometry reflects the complete outer surface, unlike simplified collision.
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            box, mesh = geometry.find('box'), geometry.find('mesh')
            if mesh is not None:
                vertices = mesh_vertices(mesh.get('filename'))
                scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                if scale.size != 3 or not np.isfinite(scale).all() or np.any(scale <= 0):
                    raise ValueError('범퍼 메시 scale이 올바르지 않습니다.')
                vertices *= scale
            elif box is not None:
                size = np.fromstring(box.get('size'), sep=' ')
                if size.size != 3 or not np.isfinite(size).all() or np.any(size <= 0):
                    raise ValueError('범퍼 box 치수가 올바르지 않습니다.')
                vertices = np.array([[x, y, z] for x in (-1, 1)
                                     for y in (-1, 1) for z in (-1, 1)]) * size/2
            else:
                raise ValueError('범퍼 외곽 기준은 STL 메시 또는 box 형상이 필요합니다.')
            transform = self.transforms['bumper_link'] @ origin(visual.find('origin'))
            points.extend(vertices @ transform[:3, :3].T + transform[:3, 3])
        points = np.asarray(points)
        if not points.size or not np.isfinite(points).all():
            raise ValueError('범퍼 외곽 형상을 확인할 수 없습니다.')
        low, high = points.min(axis=0), points.max(axis=0)
        self.front = np.array([high[0], (low[1]+high[1])/2, (low[2]+high[2])/2])

    def wheel_reference(self):
        """Locate the accessible front mark relative to the wheel axle midpoint."""
        root = ET.fromstring(self.description)
        centers = []
        for name in ('l_wheel_joint', 'r_wheel_joint'):
            joint = root.find(f"joint[@name='{name}']")
            if joint is None or joint.find('parent').get('link') not in self.transforms:
                raise ValueError('바퀴 축 모델 좌표가 없습니다. robot_description을 확인하세요.')
            transform = self.transforms[joint.find('parent').get('link')] @ origin(joint.find('origin'))
            centers.append(transform[:3, 3])
        if not np.allclose(np.asarray(centers)[0, [0, 2]], np.asarray(centers)[1, [0, 2]], atol=.001):
            raise ValueError('좌우 바퀴 축이 정렬되지 않은 모델입니다.')
        axle = np.mean(centers, axis=0)
        return {'bumper_from_axle_xyz_m': (self.front-axle).tolist(),
                'bumper_front_base_xyz_m': self.front.tolist(),
                'axle_center_base_xyz_m': axle.tolist(), 'robot_description_sha256': self.sha256}

    def sensor(self, frame):
        if frame not in self.transforms:
            raise ValueError(f'{frame}: base_link 기준 고정 좌표가 없습니다. 모델/프레임 확인 필요')
        return self.transforms[frame]

    def reference(self, distance, frame, kind):
        distance = float(distance)
        if not np.isfinite(distance) or distance <= 0:
            raise ValueError('범퍼 전면~벽 거리는 양수 m로 입력하세요.')
        transform = self.sensor(frame)
        offset = float(self.front[0] - transform[0, 3])
        expected = distance + offset
        if kind == 'ultrasonic':
            forward = float(transform[0, 0])
            if forward < np.cos(np.deg2rad(5)):
                raise ValueError('초음파 축이 로봇 정면에서 5° 이상 벗어납니다. 장착 확인 필요')
            expected /= forward
        elif kind != 'lidar':
            raise ValueError('지원하지 않는 거리 실험입니다.')
        if expected <= 0:
            raise ValueError('벽이 센서 뒤에 있습니다. 실측값·범퍼/센서 형상을 확인하세요.')
        return {'measurement_reference': 'bumper_front_center',
                'bumper_to_wall_m': distance, 'sensor_reference_m': float(expected),
                'bumper_front_base_xyz_m': self.front.tolist(), 'sensor_frame': frame,
                'sensor_base_xyz_m': transform[:3, 3].tolist(),
                'sensor_to_bumper_forward_offset_m': offset,
                'comparison': 'base_forward_projection' if kind == 'lidar' else 'sensor_axis_range',
                'robot_description_sha256': self.sha256,
                'geometry_source': 'running_robot_description_model_not_physical_measurement'}

    def scan_distance(self, scan):
        """Project only robot-forward rays onto the normal of a perpendicular wall."""
        rotation = self.sensor(scan.header.frame_id)[:3, :3]
        angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
        forward = rotation[0, 0]*np.cos(angles) + rotation[0, 1]*np.sin(angles)
        ranges = np.asarray(scan.ranges, dtype=float)
        mask = (np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
                & (forward >= np.cos(.035)))
        return float(np.mean(ranges[mask]*forward[mask])) if mask.any() else float('nan')

    def board_registration(self, x, y, yaw, base_height):
        """Translate the bumper's floor projection into the existing base registration."""
        return [float(x+self.front[0]), float(y+self.front[1]), float(yaw), float(base_height)]
