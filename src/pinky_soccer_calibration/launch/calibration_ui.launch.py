"""Select the camera source at process launch, never from a GUI control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    webcam = LaunchConfiguration('webcam').perform(context).lower()
    simulated_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    if webcam not in ('true', 'false'):
        raise ValueError('webcam must be true or false')
    arguments = []
    if webcam == 'true':
        if simulated_time:
            raise ValueError('실제 웹캠은 PC 시스템 시각을 사용합니다. use_sim_time:=false로 실행하세요.')
        arguments = ['-webcam', LaunchConfiguration('webcam_device').perform(context)]
        for name in ('width', 'height', 'fps'):
            arguments.extend(['--webcam-'+name, LaunchConfiguration('webcam_'+name).perform(context)])
        hfov = LaunchConfiguration('webcam_hfov_deg').perform(context)
        if hfov:
            arguments.extend(['--webcam-hfov-deg', hfov])
    return [Node(package='pinky_soccer_calibration', executable='calibration_ui', output='screen',
                 arguments=arguments, parameters=[{
                     'use_sim_time': simulated_time,
                     # 'auto': saved connection settings (연결·시간 탭); fixed at start.
                     'robot_namespace': LaunchConfiguration('robot_namespace', default='auto').perform(context),
                     'pc_ip': LaunchConfiguration('pc_ip', default='').perform(context)}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('webcam', default_value='false'),
        DeclareLaunchArgument('webcam_device', default_value='0'),
        DeclareLaunchArgument('webcam_width', default_value='640'),
        DeclareLaunchArgument('webcam_height', default_value='480'),
        DeclareLaunchArgument('webcam_fps', default_value='15'),
        DeclareLaunchArgument('webcam_hfov_deg', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_namespace', default_value='auto',
                              description="robot namespace ('' none, auto = saved settings)"),
        DeclareLaunchArgument('pc_ip', default_value='',
                              description='PC IPv4 the robot must use as chrony server '
                                          '(empty = saved settings)'),
        OpaqueFunction(function=start),
    ])
