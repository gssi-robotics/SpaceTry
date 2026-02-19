cmake_minimum_required(VERSION 3.8)
project(spacetry_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(spacetry_bt_runner
  src/bt_runner.cpp
  src/nodes_ros.cpp
)

target_include_directories(spacetry_bt_runner PRIVATE include)

ament_target_dependencies(spacetry_bt_runner
  rclcpp std_srvs sensor_msgs behaviortree_cpp
)

install(TARGETS spacetry_bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
