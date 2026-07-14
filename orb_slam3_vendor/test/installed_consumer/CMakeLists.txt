cmake_minimum_required(VERSION 3.16)
project(orb_slam3_vendor_installed_consumer LANGUAGES CXX)

find_package(orb_slam3_vendor REQUIRED)

add_executable(installed_consumer main.cpp)
target_link_libraries(installed_consumer PRIVATE ORB_SLAM3::ORB_SLAM3)
