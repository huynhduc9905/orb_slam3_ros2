cmake_minimum_required(VERSION 3.16)
project(orb_slam3_vendor)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)

set(UPSTREAM_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/ORB_SLAM3")
set(PATCHED_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3")
file(REMOVE_RECURSE "${PATCHED_SOURCE}")
file(COPY "${UPSTREAM_SOURCE}/" DESTINATION "${PATCHED_SOURCE}")
execute_process(
  COMMAND git apply --whitespace=error-all
          "${CMAKE_CURRENT_SOURCE_DIR}/patches/0001-modern-headless-install.patch"
  WORKING_DIRECTORY "${PATCHED_SOURCE}"
  COMMAND_ERROR_IS_FATAL ANY)

file(ARCHIVE_EXTRACT
  INPUT "${PATCHED_SOURCE}/Vocabulary/ORBvoc.txt.tar.gz"
  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/vocabulary")

set(ORB_SLAM3_HEADLESS ON CACHE BOOL "" FORCE)
set(ORB_SLAM3_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${PATCHED_SOURCE}" orb_slam3-build)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(vendor_link_test test/vendor_link_test.cpp)
  target_link_libraries(vendor_link_test ORB_SLAM3::ORB_SLAM3)
  add_test(NAME installed_consumer
    COMMAND ${CMAKE_COMMAND}
      -DTEST_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/installed_consumer
      -DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/installed_consumer
      -DINSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/run_installed_consumer.cmake)
endif()

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt"
  DESTINATION share/${PROJECT_NAME}/vocabulary)
install(EXPORT ORB_SLAM3Targets
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ORB_SLAM3::)

ament_export_targets(ORB_SLAM3Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3 OpenCV OpenSSL)
ament_package(CONFIG_EXTRAS "cmake/orb_slam3_vendor-extras.cmake.in")
