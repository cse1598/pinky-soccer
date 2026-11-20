cmake_minimum_required(VERSION 3.8)
project(pinky_soccer_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS ${PROJECT_NAME}/mujoco_soccer_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mujoco_soccer_bridge
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS ${PROJECT_NAME}/sensor_adapter.py
  DESTINATION lib/${PROJECT_NAME} RENAME sensor_adapter)
install(PROGRAMS ${PROJECT_NAME}/check_interface.py
  DESTINATION lib/${PROJECT_NAME} RENAME check_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mujoco_ball test/test_mujoco_ball.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
  ament_add_pytest_test(test_sensors test/test_sensors.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
endif()

ament_package()
