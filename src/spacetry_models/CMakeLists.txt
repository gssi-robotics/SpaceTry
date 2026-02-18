cmake_minimum_required(VERSION 3.8)
project(spacetry_models)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
