cmake_minimum_required(VERSION 3.16)
project(orb_lidar_mapper)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}_core
  src/pose2.cpp
  src/timed_pose_buffer.cpp
  src/trajectory_store.cpp
  src/scan_deskewer.cpp
  src/tiled_occupancy_grid.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

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
endif()

install(TARGETS ${PROJECT_NAME}_core EXPORT ${PROJECT_NAME}_targets)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_package()
