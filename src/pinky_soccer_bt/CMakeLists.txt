cmake_minimum_required(VERSION 3.16)
project(pinky_soccer_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinky_soccer_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ROS-free geometry, unit tested on its own.
add_library(soccer_logic src/soccer_logic.cpp)
target_include_directories(soccer_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(soccer_bt_nodes src/bt_nodes.cpp src/verify_mode.cpp)
target_link_libraries(soccer_bt_nodes PUBLIC
  soccer_logic
  behaviortree_cpp::behaviortree_cpp
  behaviortree_ros2::behaviortree_ros2
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${pinky_soccer_msgs_TARGETS})

add_executable(bt_node src/bt_node_main.cpp)
target_link_libraries(bt_node soccer_bt_nodes ament_index_cpp::ament_index_cpp)

install(TARGETS soccer_logic soccer_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY trees config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_soccer_logic test/test_soccer_logic.cpp)
  target_link_libraries(test_soccer_logic soccer_logic)
  ament_add_gtest(test_soccer_tree test/test_soccer_tree.cpp TIMEOUT 60)
  target_link_libraries(test_soccer_tree soccer_bt_nodes)
  target_compile_definitions(test_soccer_tree PRIVATE
    TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/soccer_1v1.xml")
  # The verifier catalog must match the tree XML and the C++ port models.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_verify_catalog test/test_verify_catalog.py TIMEOUT 60
    ENV BT_NODE=$<TARGET_FILE:bt_node> ROS_DOMAIN_ID=93
        TREE_FILE=${CMAKE_CURRENT_SOURCE_DIR}/trees/soccer_1v1.xml
        CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/config/verify_catalog.yaml)
endif()

ament_package()
