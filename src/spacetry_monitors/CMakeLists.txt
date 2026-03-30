cmake_minimum_required(VERSION 3.8)
project(spacetry_monitors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(copilot
  src/copilot_monitor.cpp
  src/copilot.c
)

target_include_directories(copilot PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(copilot rclcpp std_msgs geometry_msgs)

# Make headers visible to dependents (optional)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs)

# Install the binary so 'ros2 run spacetry_monitors copilot' works
install(TARGETS
  copilot
  DESTINATION lib/${PROJECT_NAME})

# Install the manifest so the package is discoverable by 'ros2'
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

# Install public headers (copilot.h, copilot_types.h)
install(DIRECTORY include/
  DESTINATION include)

ament_package()
