cmake_minimum_required(VERSION 3.8)
project(hazard_guard_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunPatrol.action"
  "msg/HazardDetection.msg"
  "msg/PatrolWaypoint.msg"
  "msg/PersonObservation.msg"
  "msg/PersonObservationArray.msg"
  "msg/PersonSafetyState.msg"
  "msg/RobotTelemetry.msg"
  "srv/RobotCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
