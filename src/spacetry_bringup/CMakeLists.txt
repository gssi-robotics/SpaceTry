cmake_minimum_required(VERSION 3.8)
project(spacetry_bringup)

# Silence "unused variable" warning from colcon's CATKIN_INSTALL_INTO_PREFIX_ROOT
set(CATKIN_INSTALL_INTO_PREFIX_ROOT TRUE CACHE BOOL "" FORCE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
