cmake_minimum_required(VERSION 3.8)
project(pinky_soccer_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS ${PROJECT_NAME}/wheel_joint_state_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_joint_state_relay
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_joint_state_relay test/test_wheel_joint_state_relay.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
