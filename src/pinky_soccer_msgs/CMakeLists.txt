cmake_minimum_required(VERSION 3.8)
project(pinky_soccer_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject.msg"
  "msg/WorldModel.msg"
  "msg/BehaviorGoal.msg"
  "msg/MotionResult.msg"
  "srv/CaptureFrame.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
