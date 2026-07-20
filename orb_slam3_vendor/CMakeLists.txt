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
execute_process(
  COMMAND git clone --no-hardlinks "${UPSTREAM_SOURCE}" "${PATCHED_SOURCE}"
  COMMAND_ERROR_IS_FATAL ANY)
file(GLOB PATCH_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/patches/*.patch")
list(SORT PATCH_FILES)
foreach(PATCH_FILE IN LISTS PATCH_FILES)
  execute_process(
    COMMAND git apply --3way --whitespace=error-all "${PATCH_FILE}"
    WORKING_DIRECTORY "${PATCHED_SOURCE}"
    COMMAND_ERROR_IS_FATAL ANY)
endforeach()

file(ARCHIVE_EXTRACT
  INPUT "${PATCHED_SOURCE}/Vocabulary/ORBvoc.txt.tar.gz"
  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/vocabulary")

set(ORB_SLAM3_HEADLESS ON CACHE BOOL "" FORCE)
set(ORB_SLAM3_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${PATCHED_SOURCE}" orb_slam3-build)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  target_compile_definitions(ORB_SLAM3 PRIVATE ORB_SLAM3_SNAPSHOT_TESTING)
  ament_add_gtest(vendor_link_test test/vendor_link_test.cpp)
  target_link_libraries(vendor_link_test ORB_SLAM3::ORB_SLAM3)
  
  ament_add_gtest(loop_revisit_policy_test test/loop_revisit_policy_test.cpp)
  target_link_libraries(loop_revisit_policy_test ORB_SLAM3::ORB_SLAM3)
  set_tests_properties(loop_revisit_policy_test PROPERTIES TIMEOUT 10)

  ament_add_gtest(snapshot_value_test test/snapshot_value_test.cpp)
  target_compile_definitions(snapshot_value_test PRIVATE ORB_SLAM3_SNAPSHOT_TESTING)
  target_link_libraries(snapshot_value_test ORB_SLAM3::ORB_SLAM3)
  set_tests_properties(snapshot_value_test PROPERTIES TIMEOUT 10)
  ament_add_gtest(loop_closing_notification_test test/loop_closing_notification_test.cpp)
  target_compile_definitions(loop_closing_notification_test PRIVATE ORB_SLAM3_SNAPSHOT_TESTING)
  target_link_libraries(loop_closing_notification_test ORB_SLAM3::ORB_SLAM3)
  set_tests_properties(loop_closing_notification_test PROPERTIES TIMEOUT 10 ENVIRONMENT "ORB_TEST_VOCAB=${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt;ORB_TEST_SETTINGS=${CMAKE_CURRENT_SOURCE_DIR}/../orb_slam3_wrapper/config/tasterobot_stereo.yaml")
  ament_add_gtest(system_shutdown_test test/system_shutdown_test.cpp)
  target_compile_definitions(system_shutdown_test PRIVATE ORB_SLAM3_SNAPSHOT_TESTING)
  target_link_libraries(system_shutdown_test ORB_SLAM3::ORB_SLAM3)
  set_tests_properties(system_shutdown_test PROPERTIES
    TIMEOUT 30
    ENVIRONMENT
      "ORB_TEST_VOCAB=${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt;ORB_TEST_SETTINGS=${CMAKE_CURRENT_SOURCE_DIR}/../orb_slam3_wrapper/config/tasterobot_stereo.yaml")
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
