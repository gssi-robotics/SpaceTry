cmake_minimum_required(VERSION 3.14)
project(spacetry_requirements)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(fretish_agent)

# Install config, examples, docs
install(DIRECTORY config examples docs
  DESTINATION share/${PROJECT_NAME}
)

# Install entry-point scripts
install(PROGRAMS
  fretish_agent/cli.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
