cmake_minimum_required(VERSION 3.8)
project(pinky_soccer_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/bno055_magnetic.py
  DESTINATION lib/${PROJECT_NAME} RENAME bno055_magnetic)
install(PROGRAMS ${PROJECT_NAME}/capture_image.py
  DESTINATION lib/${PROJECT_NAME} RENAME capture_image)
install(PROGRAMS ${PROJECT_NAME}/battery_state_relay.py
  DESTINATION lib/${PROJECT_NAME} RENAME battery_state_relay)
install(PROGRAMS ${PROJECT_NAME}/time_sync.py
  DESTINATION lib/${PROJECT_NAME} RENAME time_sync_status)
install(PROGRAMS ${PROJECT_NAME}/command_watchdog.py
  DESTINATION lib/${PROJECT_NAME} RENAME command_watchdog)
install(PROGRAMS ${PROJECT_NAME}/latency_echo.py
  DESTINATION lib/${PROJECT_NAME} RENAME latency_echo)
install(PROGRAMS ${PROJECT_NAME}/drive_diagnostics.py
  DESTINATION lib/${PROJECT_NAME} RENAME drive_diagnostics)

install(
  PROGRAMS ${PROJECT_NAME}/wheel_joint_state_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_joint_state_relay
)

install(
  PROGRAMS ${PROJECT_NAME}/camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_publisher
)

install(
  PROGRAMS ${PROJECT_NAME}/imu_units_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_units_relay
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md CAMERA_CAPTURE.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bno055_magnetic test/test_bno055_magnetic.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_time_sync test/test_time_sync.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_drive_diagnostics test/test_drive_diagnostics.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_battery_state_relay test/test_battery_state_relay.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_wheel_joint_state_relay test/test_wheel_joint_state_relay.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_sensor_support test/test_sensor_support.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_sensor_runtime test/test_sensor_runtime.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_calibration_profile test/test_calibration_profile.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 120)
  ament_add_pytest_test(test_frame_capture test/test_frame_capture.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
