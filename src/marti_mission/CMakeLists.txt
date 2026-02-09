cmake_minimum_required(VERSION 3.8)
project(marti_mission)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  marti_mission
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
