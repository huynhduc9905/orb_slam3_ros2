cmake_minimum_required(VERSION 3.16)
project(orb_slam_dashboard)

find_package(ament_cmake REQUIRED)

# Require Node.js major version 22 for the frontend build.
find_program(NODE_EXECUTABLE node REQUIRED)
execute_process(
  COMMAND "${NODE_EXECUTABLE}" -p "process.versions.node.split('.')[0]"
  OUTPUT_VARIABLE NODE_MAJOR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE NODE_VERSION_RESULT
)
if(NOT NODE_VERSION_RESULT EQUAL 0)
  message(FATAL_ERROR
    "orb_slam_dashboard: failed to query Node.js version with ${NODE_EXECUTABLE}")
endif()
if(NOT NODE_MAJOR STREQUAL "22")
  message(FATAL_ERROR
    "orb_slam_dashboard: Node.js major version 22 is required "
    "(found major ${NODE_MAJOR}). Install Node 22 and reconfigure.")
endif()

find_program(NPM_EXECUTABLE npm REQUIRED)

set(WEB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/web")
set(WEB_DIST_DIR "${WEB_DIR}/dist")
set(WEB_STAMP "${CMAKE_CURRENT_BINARY_DIR}/web_build.stamp")

add_custom_command(
  OUTPUT "${WEB_STAMP}"
  COMMAND "${NPM_EXECUTABLE}" ci
  COMMAND "${NPM_EXECUTABLE}" run build
  COMMAND ${CMAKE_COMMAND} -E touch "${WEB_STAMP}"
  WORKING_DIRECTORY "${WEB_DIR}"
  COMMENT "Building orb_slam_dashboard web UI (npm ci && npm run build)"
  VERBATIM
)

add_custom_target(orb_slam_dashboard_web ALL DEPENDS "${WEB_STAMP}")

install(
  DIRECTORY "${WEB_DIST_DIR}/"
  DESTINATION share/${PROJECT_NAME}/web
  USE_SOURCE_PERMISSIONS
)

# Python static server entrypoint (no ROS publishers/services/params).
install(
  PROGRAMS orb_slam_dashboard/server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME orb_slam_dashboard_server
)

install(FILES resource/orb_slam_dashboard
  DESTINATION share/ament_index/resource_index/packages
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
