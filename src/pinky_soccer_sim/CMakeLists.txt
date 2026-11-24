cmake_minimum_required(VERSION 3.8)
project(pinky_soccer_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(pinky_contact_kernel SHARED src/contact_kernel.cc)
target_compile_features(pinky_contact_kernel PUBLIC cxx_std_17)
add_library(pinky_foam_contact SHARED src/foam_contact_system.cc)
target_link_libraries(pinky_foam_contact PRIVATE pinky_contact_kernel gz-sim8::gz-sim8)
set_target_properties(pinky_foam_contact PROPERTIES INSTALL_RPATH "$ORIGIN")
install(TARGETS pinky_contact_kernel pinky_foam_contact LIBRARY DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})
install(FILES FIELD.md HARDBOARD_FIELD.md MATERIALS.md SENSOR_NOISE.md DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS ${PROJECT_NAME}/mujoco_soccer_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mujoco_soccer_bridge
)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS ${PROJECT_NAME}/gazebo_motor.py
  DESTINATION lib/${PROJECT_NAME} RENAME gazebo_motor)
install(PROGRAMS ${PROJECT_NAME}/sensor_adapter.py
  DESTINATION lib/${PROJECT_NAME} RENAME sensor_adapter)
install(PROGRAMS ${PROJECT_NAME}/check_interface.py
  DESTINATION lib/${PROJECT_NAME} RENAME check_interface)
install(PROGRAMS ${PROJECT_NAME}/check_material_physics.py
  DESTINATION lib/${PROJECT_NAME} RENAME check_material_physics)
install(PROGRAMS ${PROJECT_NAME}/calibrate_odometry.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/fit_sensor_noise.py
  DESTINATION lib/${PROJECT_NAME} RENAME fit_sensor_noise)
install(PROGRAMS ${PROJECT_NAME}/rcj_field.py
  DESTINATION lib/${PROJECT_NAME} RENAME generate_rcj_field)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_materials test/test_materials.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
  ament_add_pytest_test(test_rcj_field test/test_rcj_field.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
  ament_add_pytest_test(test_mujoco_ball test/test_mujoco_ball.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
  ament_add_pytest_test(test_sensors test/test_sensors.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
  ament_add_pytest_test(test_odometry_calibration test/test_odometry_calibration.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 120)
  ament_add_pytest_test(test_multi_robot test/test_multi_robot.py
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(test_sensor_noise test/test_sensor_noise.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 300)
endif()

ament_package()
