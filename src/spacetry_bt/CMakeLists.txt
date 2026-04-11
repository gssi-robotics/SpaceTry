cmake_minimum_required(VERSION 3.8)
project(spacetry_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(spacetry_bt_runner
  src/bt_runner.cpp
  src/nodes_ros.cpp
)

target_include_directories(spacetry_bt_runner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(spacetry_bt_runner
  rclcpp
  behaviortree_cpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS spacetry_bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(FILES bt_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
