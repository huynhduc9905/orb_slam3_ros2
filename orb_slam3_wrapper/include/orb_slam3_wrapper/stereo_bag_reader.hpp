#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "orb_slam3_wrapper/mount_xy_mapper.hpp"
#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {

struct StereoFrame {
  int64_t stamp_ns{};
  cv::Mat left_bgr_or_gray;
  cv::Mat right_bgr_or_gray;
};

struct StereoDataset {
  std::vector<StereoFrame> frames;  // approx-synced pairs
  sensor_msgs::msg::CameraInfo left_info;
  sensor_msgs::msg::CameraInfo right_info;
  StaticCameraMount recorded_mount;
  MountXy recorded_camera_link_xy;  // from T_base_camera_link
  std::vector<std::pair<int64_t, Pose2>> odom_se2;  // optional empty
  std::string left_optical_frame;
  std::string right_optical_frame;
};

// Convert geometry_msgs Transform → Eigen Isometry3d.
Eigen::Isometry3d isometryFromGeometryTransform(
    const geometry_msgs::msg::Transform& tf);

// Build StaticCameraMount from a flat list of static transforms.
// Requires a resolvable chain base_link → camera_link (fail closed).
// Composes camera_link → left_optical_frame (multi-hop OK via tf2 BufferCore).
StaticCameraMount buildStaticCameraMount(
    const std::vector<geometry_msgs::msg::TransformStamped>& transforms,
    const std::string& base_frame = "base_link",
    const std::string& camera_link_frame = "camera_link",
    const std::string& left_optical_frame = "camera_infra1_optical_frame");

// Pair left/right image stamps by nearest neighbor within max_skew_ns.
// Returns (left_index, right_index, pair_stamp_ns=left_stamp) in left order.
// Exposed for unit testing.
std::vector<std::tuple<std::size_t, std::size_t, int64_t>> syncStereoStamps(
    const std::vector<int64_t>& left_stamps,
    const std::vector<int64_t>& right_stamps,
    int64_t max_skew_ns = 5'000'000LL);

struct StereoBagReader {
  static StereoDataset read(const std::filesystem::path& bag_path);
};

}  // namespace orb_slam3_wrapper
