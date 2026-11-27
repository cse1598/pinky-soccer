"""Run the overhead referee with its own RealSense, a video, or external ROS topics."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULTS = {
    "mode": "realsense",
    "video_path": "",
    "camera_namespace": "camera",
    "camera_name": "camera",
    "color_profile": "1280x720x30",
    "serial_no": "",
    "image_topic": "/camera/camera/color/image_raw",
    "camera_info_topic": "/camera/camera/color/camera_info",
    "show_ui": "true",
    "publish_image": "true",
    "field_length_m": "1.670",
    "field_width_m": "1.070",
    "pixels_per_meter": "550.0",
    "margin_m": "0.15",
    "ball_radius_m": "0.02135",
    "warmup_frames": "30",
    "require_camera_info": "false",
    "allow_approximate_rules": "false",
    "yellow_goal_y_min": "0.0",
    "yellow_goal_y_max": "0.0",
    "blue_goal_y_min": "0.0",
    "blue_goal_y_max": "0.0",
    "frame_timeout_s": "0.5",
    "robot_match_gate_m": "0.12",
    "robot_reacquire_frames": "2",
    "blue_led_min_light": "125",
    "velocity_arrow_min_mps": "0.035",
    "distortion_mode": "auto",
    "calibration_frames": "9",
    "calibration_max_error_px": "5.0",
    "debug_ui": "false",
    "debug_log": "false",
    "record_video": "false",
    "debug_dir": "~/pinky_referee_logs",
    "record_fps": "30.0",
}

BOOL_PARAMS = ("show_ui", "publish_image", "require_camera_info", "allow_approximate_rules", "debug_ui", "debug_log", "record_video")
INT_PARAMS = ("warmup_frames", "robot_reacquire_frames", "blue_led_min_light", "calibration_frames")
FLOAT_PARAMS = (
    "field_length_m", "field_width_m", "pixels_per_meter", "margin_m",
    "ball_radius_m", "yellow_goal_y_min", "yellow_goal_y_max",
    "blue_goal_y_min", "blue_goal_y_max", "frame_timeout_s",
    "robot_match_gate_m", "velocity_arrow_min_mps", "calibration_max_error_px", "record_fps",
)


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        v = value(name).lower()
        if v not in ("true", "false"):
            raise RuntimeError(name + " must be true or false")
        return v == "true"

    mode = value("mode")
    params = {k: boolean(k) for k in BOOL_PARAMS}
    params.update({k: int(value(k)) for k in INT_PARAMS})
    params.update({k: float(value(k)) for k in FLOAT_PARAMS})
    params.update(video_path="", image_topic=value("image_topic"),
                  camera_info_topic=value("camera_info_topic"), distortion_mode=value("distortion_mode"), debug_dir=value("debug_dir"))
    actions = []

    if mode == "realsense":
        # Resolve the driver only in this mode; video and topic mode work
        # on computers without librealsense or the RealSense ROS wrapper.
        try:
            share = get_package_share_directory("realsense2_camera")
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "RealSense mode needs realsense2_camera. Install "
                "ros-jazzy-realsense2-camera or source its workspace."
            ) from exc

        ns = value("camera_namespace").strip("/")
        name = value("camera_name").strip("/")
        if not name or "/" in name:
            raise RuntimeError("camera_name must be a single nonempty ROS name")
        prefix = "/" + "/".join(x for x in (ns, name) if x)
        params["image_topic"] = prefix + "/color/image_raw"
        params["camera_info_topic"] = prefix + "/color/camera_info"

        camera_args = {
            "camera_namespace": ns,
            "camera_name": name,
            "enable_color": "true",
            "enable_depth": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "rgb_camera.color_profile": value("color_profile"),
        }
        if value("serial_no"):
            camera_args["serial_no"] = value("serial_no")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share) / "launch" / "rs_launch.py")),
            launch_arguments=camera_args.items(),
        ))
    elif mode == "video":
        path = Path(value("video_path")).expanduser()
        if not value("video_path") or not path.is_file():
            raise RuntimeError("mode:=video requires an existing video_path")
        params["video_path"] = str(path.resolve())
    elif mode == "topic":
        pass
    else:
        raise RuntimeError("mode must be realsense, video, or topic")

    # The node waits for incoming frames and performs its own warm-up and
    # calibration; no fixed startup sleep is needed.
    actions.append(Node(
        package="pinky_soccer_referee",
        executable="referee_node",
        output="screen",
        parameters=[params],
    ))
    return actions


def generate_launch_description():
    arguments = []
    for name, default in DEFAULTS.items():
        kwargs = {"choices": ["realsense", "video", "topic"]} if name == "mode" else {}
        arguments.append(DeclareLaunchArgument(name, default_value=default, **kwargs))
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
