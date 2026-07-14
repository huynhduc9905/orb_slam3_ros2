cmake_minimum_required(VERSION 3.16)
project(orb_slam3_wrapper)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orb_slam3_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(orb_slam3_wrapper_core src/calibration.cpp src/pose_conversion.cpp)
target_include_directories(orb_slam3_wrapper_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(orb_slam3_wrapper_core sensor_msgs)
target_link_libraries(orb_slam3_wrapper_core ORB_SLAM3::ORB_SLAM3 Eigen3::Eigen)

ament_add_gtest(calibration_test test/calibration_test.cpp)
target_include_directories(calibration_test PRIVATE include)
ament_target_dependencies(calibration_test sensor_msgs)
target_link_libraries(calibration_test orb_slam3_wrapper_core)

ament_add_gtest(pose_conversion_test test/pose_conversion_test.cpp)
target_include_directories(pose_conversion_test PRIVATE include)
ament_target_dependencies(pose_conversion_test orb_slam3_vendor)
target_link_libraries(pose_conversion_test orb_slam3_wrapper_core)

install(TARGETS orb_slam3_wrapper_core EXPORT orb_slam3_wrapper_targets)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_targets(orb_slam3_wrapper_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 orb_slam3_vendor sensor_msgs)
ament_package()
