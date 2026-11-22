"""Common real/Gazebo/MuJoCo AMCL and camera/compass launch construction."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_field.generate_field_map import default_lidar_height, rasterize, write_map


def localization_nodes(field_spec, use_sim_time, initial_pose=(0., 0., 0.),
                       allow_uncalibrated=False, approximate_hfov_deg=66., camera_yaml='',
                       heading_record='', image_rotated_180=False):
    """Generate the actual LiDAR-height map from the selected single field YAML."""
    field = Field.from_file(field_spec)
    output = Path(tempfile.mkdtemp(prefix='pinky_soccer_localization_'))
    height = default_lidar_height()
    grid, origin, _ = rasterize(field, .01, height)
    _, map_yaml = write_map(grid, origin, .01, output, field.spec.get('field_id', 'field'))
    cfg = Path(get_package_share_directory('pinky_soccer_world_model'), 'config')
    clock = {'use_sim_time': use_sim_time}
    map_config = {'yaml_filename': str(map_yaml), 'frame_id': 'map'}
    record = Path(heading_record).expanduser() if heading_record else Path.home().joinpath(
        '.ros', 'pinky_soccer', 'localization', output.name, 'heading_reference.yaml')
    return [
        Node(package='nav2_map_server', executable='map_server', name='field_map_server',
             output='screen', parameters=[clock, map_config]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[str(cfg/'amcl.yaml'), clock, {
                 'initial_pose.x': float(initial_pose[0]),
                 'initial_pose.y': float(initial_pose[1]),
                 'initial_pose.yaw': float(initial_pose[2])}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='localization_lifecycle', output='screen', parameters=[clock, {
                 'autostart': True, 'node_names': ['field_map_server', 'amcl'],
                 'bond_timeout': 4.0}]),
        Node(package='pinky_soccer_world_model', executable='goal_heading', name='goal_heading',
             output='screen', parameters=[str(cfg/'goal_heading.yaml'), clock, {
                 'field_spec': str(field_spec), 'allow_uncalibrated': allow_uncalibrated,
                 'approximate_hfov_deg': approximate_hfov_deg, 'camera_yaml': camera_yaml,
                 'image_rotated_180': image_rotated_180,
                 'heading_record': str(record)}]),
    ]
