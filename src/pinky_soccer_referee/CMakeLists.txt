cmake_minimum_required(VERSION 3.8)
project(pinky_soccer_referee)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject.msg" "msg/RefereeState.msg" "msg/RefereeEvent.msg"
  "srv/GetState.srv"
  DEPENDENCIES std_msgs geometry_msgs)
install(PROGRAMS scripts/referee_node.py DESTINATION lib/${PROJECT_NAME} RENAME referee_node)
install(FILES scripts/referee_core.py scripts/referee_recorder.py scripts/referee_controls.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
