cmake_minimum_required(VERSION 3.10)
project(orb_slam3_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyframePose.msg"
  "msg/LoopEdge.msg"
  "msg/TrackedFrame.msg"
  "msg/GraphSnapshot.msg"
  "msg/RevisionedPath.msg"
  "msg/TrackingEvent.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
