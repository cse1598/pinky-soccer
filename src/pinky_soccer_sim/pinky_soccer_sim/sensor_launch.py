"""Shared simulator sensor launch arguments and normalization nodes."""

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def arguments():
    """Match the real launch's enable flags; keep unverified hardware opt-in."""
    return [
        DeclareLaunchArgument('sensor_config', default_value=PathJoinSubstitution([
            get_package_share_directory('pinky_soccer_sim'), 'config', 'sensors.yaml'])),
        DeclareLaunchArgument('sensor_noise_config', default_value=PathJoinSubstitution([
            get_package_share_directory('pinky_soccer_sim'), 'config', 'measured_sensor_noise.yaml'])),
        DeclareLaunchArgument('sensor_noise_enabled', default_value='true'),
        DeclareLaunchArgument('sensor_noise_seed', default_value='20261124'),
        *[DeclareLaunchArgument(name, default_value=default) for name, default in
          [('camera_enabled', 'true'), ('imu_enabled', 'false'), ('adc_enabled', 'false')]],
    ]


def nodes(camera=None):
    """Use the real camera's optical transform and normalized sensor outputs."""
    return [
        Node(package='pinky_soccer_bringup', executable='command_watchdog'),
        Node(package='pinky_soccer_sim', executable='sensor_adapter',
             parameters=[{'use_sim_time': True,
                          'sensor_config': LaunchConfiguration('sensor_config'),
                          'sensor_noise_config': LaunchConfiguration('sensor_noise_config'),
                          'sensor_noise_enabled': ParameterValue(
                              LaunchConfiguration('sensor_noise_enabled'), value_type=bool),
                          'sensor_noise_seed': ParameterValue(
                              LaunchConfiguration('sensor_noise_seed'), value_type=int),
                          **{name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                             for name in ('imu_enabled', 'adc_enabled')},
                          'camera_enabled': camera if camera is not None else ParameterValue(
                              LaunchConfiguration('camera_enabled'), value_type=bool)}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='front_camera_optical_tf',
             condition=IfCondition(LaunchConfiguration('camera_enabled') if camera is None
                                   else 'true' if camera else 'false'),
             arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '-1.5707963267948966',
                        '--pitch', '0', '--yaw', '-1.5707963267948966',
                        '--frame-id', 'front_camera_link',
                        '--child-frame-id', 'front_camera_optical_frame'],
             parameters=[{'use_sim_time': True}]),
    ]
