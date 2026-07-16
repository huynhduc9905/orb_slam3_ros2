cmake_minimum_required(VERSION 3.16)
project(orb_lidar_mapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orb_slam3_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.15 REQUIRED COMPONENTS common registration kdtree filters)

# ── Core library (no ROS) ────────────────────────────────────────────────────
add_library(${PROJECT_NAME}_core
  src/pose2.cpp
  src/timed_pose_buffer.cpp
  src/trajectory_store.cpp
  src/scan_deskewer.cpp
  src/tiled_occupancy_grid.cpp
  src/map_rebuilder.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── Mapper node library (ROS) ────────────────────────────────────────────────
add_library(${PROJECT_NAME}_node src/mapper_node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  sensor_msgs nav_msgs geometry_msgs visualization_msgs
  diagnostic_msgs std_msgs orb_slam3_msgs)

# ── Offline calibration library ─────────────────────────────────────────────
add_library(${PROJECT_NAME}_calibration
  src/rotation_bag_reader.cpp
  src/calibration_deskew.cpp)
target_compile_features(${PROJECT_NAME}_calibration PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_calibration SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_calibration
  ${PROJECT_NAME}_core ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_calibration
  rclcpp rosbag2_cpp sensor_msgs nav_msgs tf2_msgs pcl_conversions)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME}_node_exe src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node_exe ${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME}_node_exe rclcpp)
set_target_properties(${PROJECT_NAME}_node_exe PROPERTIES
  OUTPUT_NAME orb_lidar_mapper_node)
install(TARGETS ${PROJECT_NAME}_node_exe DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(pose2_test test/pose2_test.cpp)
  target_include_directories(pose2_test PRIVATE include)
  target_link_libraries(pose2_test ${PROJECT_NAME}_core)

  ament_add_gtest(timed_pose_buffer_test test/timed_pose_buffer_test.cpp)
  target_include_directories(timed_pose_buffer_test PRIVATE include)
  target_link_libraries(timed_pose_buffer_test ${PROJECT_NAME}_core)

  ament_add_gtest(trajectory_store_test test/trajectory_store_test.cpp)
  target_include_directories(trajectory_store_test PRIVATE include)
  target_link_libraries(trajectory_store_test ${PROJECT_NAME}_core)

  ament_add_gtest(scan_deskewer_test test/scan_deskewer_test.cpp)
  target_include_directories(scan_deskewer_test PRIVATE include)
  target_link_libraries(scan_deskewer_test ${PROJECT_NAME}_core)

  ament_add_gtest(tiled_occupancy_grid_test test/tiled_occupancy_grid_test.cpp)
  target_include_directories(tiled_occupancy_grid_test PRIVATE include)
  target_link_libraries(tiled_occupancy_grid_test ${PROJECT_NAME}_core)

  ament_add_gtest(map_rebuilder_test test/map_rebuilder_test.cpp)
  target_include_directories(map_rebuilder_test PRIVATE include)
  target_link_libraries(map_rebuilder_test ${PROJECT_NAME}_core)

  ament_add_gtest(mapper_node_test test/mapper_node_test.cpp)
  target_include_directories(mapper_node_test PRIVATE include)
  target_link_libraries(mapper_node_test ${PROJECT_NAME}_node)
  ament_target_dependencies(mapper_node_test
    rclcpp tf2_ros sensor_msgs nav_msgs geometry_msgs
    visualization_msgs orb_slam3_msgs)

  ament_add_gtest(rotation_bag_reader_test test/rotation_bag_reader_test.cpp)
  target_include_directories(rotation_bag_reader_test PRIVATE include)
  target_link_libraries(rotation_bag_reader_test ${PROJECT_NAME}_calibration)
  ament_target_dependencies(rotation_bag_reader_test
    rclcpp rosbag2_cpp sensor_msgs nav_msgs tf2_msgs)

  ament_add_gtest(calibration_deskew_test test/calibration_deskew_test.cpp)
  target_include_directories(calibration_deskew_test PRIVATE include)
  target_link_libraries(calibration_deskew_test ${PROJECT_NAME}_calibration)
  ament_target_dependencies(calibration_deskew_test
    rclcpp rosbag2_cpp sensor_msgs nav_msgs tf2_msgs)
endif()

install(TARGETS ${PROJECT_NAME}_core EXPORT ${PROJECT_NAME}_targets)
install(TARGETS ${PROJECT_NAME}_node EXPORT ${PROJECT_NAME}_targets)
install(TARGETS ${PROJECT_NAME}_calibration EXPORT ${PROJECT_NAME}_targets)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp tf2 tf2_ros tf2_geometry_msgs
  sensor_msgs nav_msgs geometry_msgs visualization_msgs
  diagnostic_msgs std_msgs orb_slam3_msgs rosbag2_cpp tf2_msgs pcl_conversions)
ament_export_dependencies(PCL)
ament_package()
