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
  scripts/thermal_colorize.py scripts/thermal_overlay.py
  scripts/thermal_camera_info.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_adaptive_cloud_guard
    test/test_adaptive_cloud_guard.py
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_camera_topics test/test_camera_topics.py)
  ament_add_pytest_test(test_thermal_mount test/test_thermal_mount.py)
endif()

ament_package()
