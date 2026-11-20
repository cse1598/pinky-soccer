"""Shared declarations and process descriptions for robot sensor launch files."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


SENSOR_ARGUMENTS = {
    'camera_enabled': ('true', 'Start the robot pinkylib camera publisher'),
    'camera_width': ('640', 'Camera output width in pixels'),
    'camera_height': ('480', 'Camera output height in pixels'),
    'camera_fps': ('15.0', 'Camera publication rate limit; measure actual throughput'),
    'camera_jpeg_quality': ('85', 'JPEG quality for image_raw/compressed (1..100)'),
    'camera_info_url': ('', 'Measured ROS calibration YAML, file:///absolute/path.yaml'),
    'camera_frame_id': ('front_camera_optical_frame', 'Optical frame of emitted images'),
    'imu_enabled': ('false', 'Enable only after confirming BNO055/I2C compatibility'),
    'imu_interface': ('/dev/i2c-0', 'Official BNO055 I2C interface'),
    'imu_rate': ('100.0', 'Official BNO055 sample rate'),
    'adc_enabled': ('false', 'Enable official ultrasonic/IR/ADC driver after verification'),
    'adc_interface': ('/dev/i2c-1', 'Official sensor ADC I2C interface'),
    'adc_rate': ('20.0', 'Official sensor ADC polling rate'),
}


def sensor_arguments():
    """Declare common sensor switches and configuration parameters."""
    return [DeclareLaunchArgument(name, default_value=default, description=description)
            for name, (default, description) in SENSOR_ARGUMENTS.items()]


def lc(name):
    """Return a launch configuration reference."""
    return LaunchConfiguration(name)


def enabled(name):
    """Keep hardware:=false independent of all per-sensor switches."""
    return IfCondition(AndSubstitution(lc('hardware'), lc(name)))


def generate_sensor_description():
    """Run the physical camera and opt-in official IMU/ADC drivers."""
    sim_time = {'use_sim_time': ParameterValue(lc('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *sensor_arguments(),
        Node(package='pinky_soccer_bringup', executable='camera_publisher',
             name='soccer_camera', output='screen', condition=enabled('camera_enabled'),
             parameters=[sim_time, {
                 'width': ParameterValue(lc('camera_width'), value_type=int),
                 'height': ParameterValue(lc('camera_height'), value_type=int),
                 'fps': ParameterValue(lc('camera_fps'), value_type=float),
                 'jpeg_quality': ParameterValue(lc('camera_jpeg_quality'), value_type=int),
                 'camera_info_url': ParameterValue(lc('camera_info_url'), value_type=str),
                 'frame_id': ParameterValue(lc('camera_frame_id'), value_type=str),
             }]),
        # Axis convention only: the wrapper URDF still supplies the physical pose.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='front_camera_optical_tf', condition=enabled('camera_enabled'),
             parameters=[sim_time], arguments=[
                 '--x', '0', '--y', '0', '--z', '0',
                 '--roll', '-1.5707963267948966', '--pitch', '0',
                 '--yaw', '-1.5707963267948966',
                 '--frame-id', 'front_camera_link', '--child-frame-id', lc('camera_frame_id'),
             ]),
        Node(package='pinky_imu_bno055', executable='main_node', name='soccer_imu_driver',
             output='screen', condition=enabled('imu_enabled'),
             parameters=[sim_time, {
                 'interface': lc('imu_interface'), 'frame_id': 'imu_link',
                 'rate': ParameterValue(lc('imu_rate'), value_type=float),
             }], remappings=[('imu_raw', 'imu_native')]),
        Node(package='pinky_soccer_bringup', executable='imu_units_relay',
             name='imu_units_relay', output='screen', parameters=[sim_time],
             condition=enabled('imu_enabled')),
        Node(package='pinky_sensor_adc', executable='main_node', name='soccer_adc_driver',
             output='screen', condition=enabled('adc_enabled'),
             parameters=[sim_time, {
                 'interface': lc('adc_interface'),
                 'rate': ParameterValue(lc('adc_rate'), value_type=float),
             }]),
    ])
