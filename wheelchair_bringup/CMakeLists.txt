cmake_minimum_required(VERSION 3.8)
project(wheelchair_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The project is Apache-2.0 (see ../LICENSE), but source files carry no per-file
  # copyright headers yet, so skip the copyright check -- matching the ament_python
  # packages whose generated test_copyright is also skipped. Enable both together.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
