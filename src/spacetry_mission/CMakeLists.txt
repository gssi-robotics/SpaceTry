cmake_minimum_required(VERSION 3.8)
project(spacetry_mission)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python module package (spacetry_mission/)
ament_python_install_package(${PROJECT_NAME})

# Install configs (student-editable)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install the executable script so `ros2 run spacetry_mission spacetry_mission_server` works
install(PROGRAMS
  spacetry_mission/spacetry_mission_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
