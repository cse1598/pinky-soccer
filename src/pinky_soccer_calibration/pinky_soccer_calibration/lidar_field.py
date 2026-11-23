"""Preserve one stationary LiDAR observation and an honest ray-traced reference map."""

import csv
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np
import yaml


def observation(scan, geometry=None):
    """Return all beam data and a static sensor transform without inventing a field pose."""
    count = len(scan.ranges)
    values = [scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min, scan.range_max]
    if (not scan.header.frame_id or not 12 <= count <= 10000
            or not np.isfinite(values).all() or scan.angle_increment == 0
            or not 0 <= scan.range_min < scan.range_max <= 100):
        raise ValueError('LiDAR 프레임·레이 개수·각도·감지 범위를 확인하세요.')
    angles = scan.angle_min+np.arange(count)*scan.angle_increment
    if (abs(angles[-1]-scan.angle_max) > 1.5*abs(scan.angle_increment)+1e-5
            or abs(angles[-1]-angles[0]) > 2*math.pi+.1):
        raise ValueError('LiDAR 각도 범위와 레이 개수가 일치하지 않습니다.')
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    hits = valid & (ranges < scan.range_max)
    if np.count_nonzero(hits) < 12:
        raise ValueError('벽을 관측한 유효 레이가 12개 미만입니다. 경기장 배치와 /scan을 확인하세요.')
    transform, frame = np.eye(4), scan.header.frame_id
    model_hash = getattr(geometry, 'sha256', None)
    if geometry is not None:
        try:
            transform = geometry.sensor(frame).copy()
            frame, model_hash = 'base_link', geometry.sha256
        except ValueError:
            pass
    points = np.full((count, 3), np.nan)
    points[valid] = np.c_[ranges[valid]*np.cos(angles[valid]),
                          ranges[valid]*np.sin(angles[valid]), np.zeros(valid.sum())]
    points[valid] = points[valid]@transform[:3, :3].T+transform[:3, 3]
    return angles, ranges, valid, hits, points, transform, frame, model_hash


def ray_cells(start, end):
    """Yield integer Bresenham cells, including both endpoints."""
    x, y = start
    ex, ey = end
    dx, dy = abs(ex-x), -abs(ey-y)
    sx, sy, error = (1 if x < ex else -1), (1 if y < ey else -1), dx+dy
    while True:
        yield x, y
        if x == ex and y == ey:
            break
        twice = 2*error
        if twice >= dy:
            error += dy
            x += sx
        if twice <= dx:
            error += dx
            y += sy


def rasterize(points, valid, hits, sensor_origin, resolution):
    """Free only traversed rays; mark returns occupied and unobserved cells unknown."""
    if not math.isfinite(resolution) or not .005 <= resolution <= .1:
        raise ValueError('지도 해상도는 0.005~0.1m/픽셀로 입력하세요.')
    bounds = np.vstack((points[valid, :2], np.asarray(sensor_origin[:2]), [0, 0]))
    lower = np.floor((bounds.min(axis=0)-.1)/resolution)*resolution
    upper = np.ceil((bounds.max(axis=0)+.1)/resolution)*resolution
    size = np.ceil((upper-lower)/resolution).astype(int)+1
    if size.prod() > 4_000_000 or size.max() > 4000:
        raise ValueError('지도 범위가 너무 큽니다. 이상 반사값을 확인하거나 해상도를 높이세요.')
    grid = np.full((size[1], size[0]), 205, np.uint8)
    start = tuple(np.floor((sensor_origin[:2]-lower)/resolution).astype(int))
    endpoints = np.floor((points[valid, :2]-lower)/resolution).astype(int)
    occupied = []
    for endpoint, hit in zip(endpoints, hits[valid]):
        cells = list(ray_cells(start, tuple(endpoint)))
        for x, y in cells[:-1] if hit else cells:
            grid[y, x] = 254
        if hit:
            occupied.append(tuple(endpoint))
    # A later ray must never erase an earlier occupied endpoint.
    for x, y in occupied:
        grid[y, x] = 0
    return np.flipud(grid), lower.tolist()


def save_field(folder, scan, *, received_monotonic, received_ros_ns, geometry=None,
               odom=None, odom_age_s=None, environment='rehearsal', robot_id='',
               resolution=.01, now_monotonic=None):
    """Atomically save an immutable snapshot, raw bag/CSV and a Nav2-compatible map."""
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.convert import message_to_yaml
    import rosbag2_py

    now = time.monotonic() if now_monotonic is None else now_monotonic
    age = now-received_monotonic
    if not math.isfinite(age) or not 0 <= age <= 1.5:
        raise ValueError('LiDAR 수신이 1.5초 넘게 끊겼습니다. 새 /scan을 받은 뒤 저장하세요.')
    if odom is not None and odom_age_s is not None and 0 <= odom_age_s <= 1.5:
        twist = odom.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        if (not np.isfinite([speed, twist.angular.z]).all()
                or speed > .02 or abs(twist.angular.z) > .03):
            raise ValueError('로봇이 움직이고 있습니다. 완전히 정지한 뒤 저장하세요.')
    angles, ranges, valid, hits, points, transform, frame, model_hash = observation(scan, geometry)
    grid, origin = rasterize(points, valid, hits, transform[:3, 3], resolution)
    root = Path(folder)/'lidar_fields'
    root.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8]
    destination = root/name
    with tempfile.TemporaryDirectory(prefix='.saving_', dir=root) as temporary:
        path = Path(temporary)
        pgm = f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode()+grid.tobytes()
        (path/'map.pgm').write_bytes(pgm)
        map_info = {'image': 'map.pgm', 'mode': 'trinary', 'resolution': resolution,
                    'origin': [*origin, 0.], 'negate': 0,
                    'occupied_thresh': .65, 'free_thresh': .196}
        (path/'map.yaml').write_text(yaml.safe_dump(map_info, sort_keys=False))
        (path/'scan.yaml').write_text(message_to_yaml(scan))
        with (path/'scan.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['beam', 'angle_rad', 'range_m', 'valid', 'obstacle_return',
                             'intensity', 'map_x_m', 'map_y_m', 'map_z_m'])
            for i in range(len(ranges)):
                intensity = scan.intensities[i] if i < len(scan.intensities) else ''
                writer.writerow([i, angles[i], ranges[i], bool(valid[i]), bool(hits[i]),
                                 intensity, *points[i]])
        messages = [('/scan', 'sensor_msgs/msg/LaserScan', scan)]
        if odom is not None:
            messages.append(('/odom', 'nav_msgs/msg/Odometry', odom))
        bag = rosbag2_py.SequentialWriter()
        try:
            bag.open(rosbag2_py.StorageOptions(uri=str(path/'raw_bag'), storage_id='sqlite3'),
                     rosbag2_py.ConverterOptions('', ''))
            for i, (topic, kind, message) in enumerate(messages):
                bag.create_topic(rosbag2_py.TopicMetadata(
                    id=i, name=topic, type=kind, serialization_format='cdr'))
                bag.write(topic, serialize_message(message), received_ros_ns)
        finally:
            del bag
        metadata = {
            'schema_version': 1, 'kind': 'single_stationary_lidar_observation',
            'recorded_utc': datetime.now(timezone.utc).isoformat(),
            'environment': environment, 'robot_id': robot_id, 'topic': '/scan',
            'source_frame': scan.header.frame_id, 'capture_coordinate_frame': frame,
            'map_frame_definition': 'capture-time '+frame+' axes and origin; not field centre',
            'sensor_to_capture_transform': transform.tolist(),
            'capture_base_pose_in_map': [0., 0., 0.] if frame == 'base_link' else None,
            'robot_description_sha256': model_hash,
            'header_stamp_ns': scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec,
            'header_age_s': (received_ros_ns-scan.header.stamp.sec*1_000_000_000
                             - scan.header.stamp.nanosec)/1e9,
            'received_ros_ns': received_ros_ns, 'receive_age_s': age,
            'odom_age_s': odom_age_s, 'resolution_m': resolution,
            'beams': len(ranges), 'valid_beams': int(valid.sum()),
            'obstacle_returns': int(hits.sum()),
            'ignored_beams': int((~valid).sum()), 'grid_size_px': list(reversed(grid.shape)),
            'occupied_cells': int(np.count_nonzero(grid == 0)),
            'free_cells': int(np.count_nonzero(grid == 254)),
            'unknown_cells': int(np.count_nonzero(grid == 205)),
            'observed_angular_span_rad': float(abs(angles[-1]-angles[0])),
            'map_yaml': 'map.yaml', 'raw_scan': 'scan.yaml', 'points_csv': 'scan.csv',
            'limitations': ['Single scan; no SLAM or motion compensation.',
                            'Occluded/unobserved areas remain unknown; no inferred rectangle.',
                            'People and obstacles are saved too; clear the field before capture.',
                            'Map axes follow the capture robot/sensor, not bumper/field centre.',
                            'A symmetric field still needs heading/initial-pose disambiguation.'],
            'sha256': {name: hashlib.sha256((path/name).read_bytes()).hexdigest()
                       for name in ('map.pgm', 'map.yaml', 'scan.yaml', 'scan.csv')}}
        if geometry is not None:
            (path/'robot_description.urdf').write_text(geometry.description)
        (path/'observation.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
        path.rename(destination)
    return destination, metadata
