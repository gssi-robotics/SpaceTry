cmake_minimum_required(VERSION 3.8)
project(marti_mission)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
