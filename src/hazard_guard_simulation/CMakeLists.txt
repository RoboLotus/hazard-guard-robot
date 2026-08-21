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
install(PROGRAMS scripts/rtabmap_scan_demo.py scripts/auto_explore.py
  scripts/camera_info_relay.py
  scripts/cloud_stamp_relay.py scripts/adaptive_cloud_guard.py
  scripts/thermal_colorize.py scripts/thermal_overlay.py
  scripts/thermal_camera_info.py scripts/thermal_camera_publisher.py
  scripts/thermal_cloud.py
  scripts/timestamp_diagnostics.py
  scripts/person_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_adaptive_cloud_guard
    test/test_adaptive_cloud_guard.py
  )
  ament_add_pytest_test(test_physical_patrol_launch
    test/test_physical_patrol_launch.py
  )
  ament_add_pytest_test(test_cloud_stamp_relay
    test/test_cloud_stamp_relay.py
  )
  ament_add_pytest_test(test_timestamp_diagnostics
    test/test_timestamp_diagnostics.py
  )
  ament_add_pytest_test(test_thermal_cloud
    test/test_thermal_cloud.py
  )
  ament_add_pytest_test(test_rtabmap_real_launch
    test/test_rtabmap_real_launch.py
  )
  ament_add_pytest_test(test_auto_explore
    test/test_auto_explore.py
  )
endif()

install(PROGRAMS
  scripts/build_demo_people_heat_world.py
  scripts/run_demo_people_heat_simulation.sh
  scripts/trailer_aware_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_people_heat_world
    test/test_people_heat_world.py
  )
  ament_add_pytest_test(test_camera_topics test/test_camera_topics.py)
  ament_add_pytest_test(test_thermal_mount test/test_thermal_mount.py)
  ament_add_pytest_test(test_person_detector test/test_person_detector.py)
endif()

ament_package()
