cmake_minimum_required(VERSION 3.8)
project(pinky_soccer_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Only the linters listed in package.xml run (flake8, pep257, xmllint, lint_cmake).
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro test/test_xacro.py TIMEOUT 120)
endif()

ament_package()
