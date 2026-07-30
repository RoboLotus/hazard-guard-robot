cmake_minimum_required(VERSION 3.8)
project(hazard_guard_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
