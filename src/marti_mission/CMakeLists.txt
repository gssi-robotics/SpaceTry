cmake_minimum_required(VERSION 3.8)
project(marti_mission)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python module package (marti_mission/)
ament_python_install_package(${PROJECT_NAME})

# Install configs (student-editable)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install the executable script so `ros2 run marti_mission marti_mission_server` works
install(PROGRAMS
  marti_mission/marti_mission_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
