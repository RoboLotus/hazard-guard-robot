cmake_minimum_required(VERSION 3.8)
project(hazard_guard_simulation)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_library(hazard_guard_random_walk_system SHARED
  src/RandomWalkSystem.cc
)
target_link_libraries(hazard_guard_random_walk_system PRIVATE
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

add_library(hazard_guard_heat_transfer_system SHARED
  src/HeatTransferSystem.cc
)
target_link_libraries(hazard_guard_heat_transfer_system PRIVATE
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

install(TARGETS
  hazard_guard_random_walk_system
  hazard_guard_heat_transfer_system
  LIBRARY DESTINATION lib
)

install(DIRECTORY config meshes models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)
install(PROGRAMS scripts/rtabmap_scan_demo.py scripts/camera_info_relay.py
  scripts/cloud_stamp_relay.py scripts/adaptive_cloud_guard.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_adaptive_cloud_guard
    test/test_adaptive_cloud_guard.py
  )
endif()

install(PROGRAMS
  scripts/build_demo_people_heat_world.py
  scripts/run_demo_people_heat_navigation.sh
  scripts/run_demo_people_heat_simulation.sh
  scripts/trailer_aware_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_people_heat_world
    test/test_people_heat_world.py
  )
endif()

ament_package()
