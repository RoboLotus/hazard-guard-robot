cmake_minimum_required(VERSION 3.8)
project(hazard_guard_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)
install(PROGRAMS scripts/rtabmap_scan_demo.py scripts/camera_info_relay.py
  scripts/cloud_stamp_relay.py scripts/adaptive_cloud_guard.py
  scripts/timestamp_diagnostics.py
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
endif()

ament_package()
