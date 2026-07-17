#include "orb_slam3_wrapper/stereo_bag_reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace orb_slam3_wrapper {
namespace {

constexpr const char* kLeftImageTopic = "/camera/camera/infra1/image_rect_raw";
constexpr const char* kRightImageTopic = "/camera/camera/infra2/image_rect_raw";
constexpr const char* kLeftInfoTopic = "/camera/camera/infra1/camera_info";
constexpr const char* kRightInfoTopic = "/camera/camera/infra2/camera_info";
constexpr const char* kStaticTfTopic = "/tf_static";
constexpr const char* kOdomTopic = "/odom";

constexpr const char* kBaseFrame = "base_link";
constexpr const char* kCameraLinkFrame = "camera_link";
constexpr const char* kLeftOpticalFrame = "camera_infra1_optical_frame";
constexpr const char* kRightOpticalFrame = "camera_infra2_optical_frame";

constexpr int64_t kMaxStereoSkewNs = 5'000'000LL;  // 5 ms

int64_t stampNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessage& serialized) {
  MessageT message;
  if (!serialized.serialized_data || !serialized.serialized_data->buffer) {
    throw std::runtime_error("bag message has no serialized data");
  }
  rclcpp::SerializedMessage serialized_message(
      serialized.serialized_data->buffer_length);
  auto& raw = serialized_message.get_rcl_serialized_message();
  raw.buffer_length = serialized.serialized_data->buffer_length;
  std::memcpy(raw.buffer, serialized.serialized_data->buffer, raw.buffer_length);
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized_message, &message);
  return message;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-12) {
    throw std::runtime_error("invalid quaternion in /odom");
  }
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void reportBagReadProgress(std::uint64_t processed, std::uint64_t total) {
  if (total == 0) {
    std::cerr << "\r[  0%] reading bag ... " << processed << " msgs" << std::flush;
    return;
  }
  const int percent = static_cast<int>((100ULL * processed) / total);
  const int filled = std::min(30, (30 * percent) / 100);
  std::cerr << "\r[" << std::setw(3) << percent << "%] reading bag [";
  for (int i = 0; i < 30; ++i) {
    std::cerr << (i < filled ? '#' : '-');
  }
  std::cerr << "] " << processed << "/" << total << " msgs" << std::flush;
}

cv::Mat toMonoGray(const sensor_msgs::msg::Image& image) {
  if (image.encoding == "mono8" || image.encoding == "8UC1") {
    cv::Mat view(static_cast<int>(image.height), static_cast<int>(image.width),
                 CV_8UC1, const_cast<unsigned char*>(image.data.data()),
                 image.step);
    return view.clone();
  }
  return cv_bridge::toCvCopy(image, "mono8")->image;
}

geometry_msgs::msg::TransformStamped lookupRequired(
    tf2::BufferCore& buffer, const std::string& target,
    const std::string& source, const char* detail) {
  try {
    return buffer.lookupTransform(target, source, tf2::TimePointZero);
  } catch (const tf2::TransformException& error) {
    throw std::runtime_error(std::string("missing TF ") + detail + " (" +
                             target + " ← " + source + "): " + error.what());
  }
}

}  // namespace

Eigen::Isometry3d isometryFromGeometryTransform(
    const geometry_msgs::msg::Transform& tf) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() << tf.translation.x, tf.translation.y, tf.translation.z;
  result.linear() =
      Eigen::Quaterniond(tf.rotation.w, tf.rotation.x, tf.rotation.y,
                         tf.rotation.z)
          .normalized()
          .toRotationMatrix();
  return result;
}

StaticCameraMount buildStaticCameraMount(
    const std::vector<geometry_msgs::msg::TransformStamped>& transforms,
    const std::string& base_frame, const std::string& camera_link_frame,
    const std::string& left_optical_frame) {
  tf2::BufferCore buffer;
  for (const auto& t : transforms) {
    // Authority string is arbitrary for static bag TF.
    if (!buffer.setTransform(t, "stereo_bag_reader", /*is_static=*/true)) {
      throw std::runtime_error("failed to set static transform " +
                               t.header.frame_id + " → " + t.child_frame_id);
    }
  }

  const auto base_to_camera =
      lookupRequired(buffer, base_frame, camera_link_frame,
                     "base_link→camera_link");
  const auto camera_to_optical =
      lookupRequired(buffer, camera_link_frame, left_optical_frame,
                     "camera_link→left_optical");

  StaticCameraMount mount;
  mount.T_base_camera_link =
      isometryFromGeometryTransform(base_to_camera.transform);
  mount.T_camera_link_left_optical =
      isometryFromGeometryTransform(camera_to_optical.transform);
  return mount;
}

std::vector<std::tuple<std::size_t, std::size_t, int64_t>> syncStereoStamps(
    const std::vector<int64_t>& left_stamps,
    const std::vector<int64_t>& right_stamps, int64_t max_skew_ns) {
  std::vector<std::tuple<std::size_t, std::size_t, int64_t>> pairs;
  if (left_stamps.empty() || right_stamps.empty() || max_skew_ns < 0) {
    return pairs;
  }

  std::set<std::size_t> used_right;
  std::size_t right_cursor = 0;
  for (std::size_t li = 0; li < left_stamps.size(); ++li) {
    const int64_t ls = left_stamps[li];
    // Advance cursor to near ls - max_skew (lower bound search).
    while (right_cursor < right_stamps.size() &&
           right_stamps[right_cursor] < ls - max_skew_ns) {
      ++right_cursor;
    }
    std::optional<std::size_t> best_ri;
    int64_t best_abs = max_skew_ns + 1;
    for (std::size_t ri = right_cursor; ri < right_stamps.size(); ++ri) {
      const int64_t rs = right_stamps[ri];
      if (rs > ls + max_skew_ns) {
        break;
      }
      if (used_right.count(ri) != 0) {
        continue;
      }
      const int64_t abs_dt = std::llabs(rs - ls);
      if (abs_dt < best_abs ||
          (abs_dt == best_abs && best_ri && ri < *best_ri)) {
        best_abs = abs_dt;
        best_ri = ri;
      }
    }
    if (best_ri && best_abs <= max_skew_ns) {
      used_right.insert(*best_ri);
      pairs.emplace_back(li, *best_ri, ls);
    }
  }
  return pairs;
}

StereoDataset StereoBagReader::read(const std::filesystem::path& bag_path) {
  if (!std::filesystem::exists(bag_path)) {
    throw std::runtime_error("bag path does not exist: " + bag_path.string());
  }

  rosbag2_cpp::Reader reader;
  reader.open({bag_path.string(), "mcap"}, {"cdr", "cdr"});
  const auto metadata = reader.get_metadata();
  const std::uint64_t total_messages = metadata.message_count;

  std::vector<sensor_msgs::msg::Image> left_images;
  std::vector<sensor_msgs::msg::Image> right_images;
  std::optional<sensor_msgs::msg::CameraInfo> left_info;
  std::optional<sensor_msgs::msg::CameraInfo> right_info;
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
  std::vector<std::pair<int64_t, Pose2>> odom_se2;
  std::map<std::string, bool> seen;

  std::uint64_t processed_messages = 0;
  std::uint64_t next_progress_at = 0;
  reportBagReadProgress(0, total_messages);

  while (reader.has_next()) {
    const auto message = reader.read_next();
    ++processed_messages;
    if (processed_messages >= next_progress_at || !reader.has_next()) {
      reportBagReadProgress(processed_messages, total_messages);
      const auto step = total_messages == 0
                            ? 1000ULL
                            : std::max<std::uint64_t>(1, total_messages / 100);
      next_progress_at = processed_messages + step;
    }

    const auto& topic = message->topic_name;
    seen[topic] = true;

    if (topic == kLeftImageTopic) {
      left_images.push_back(deserialize<sensor_msgs::msg::Image>(*message));
    } else if (topic == kRightImageTopic) {
      right_images.push_back(deserialize<sensor_msgs::msg::Image>(*message));
    } else if (topic == kLeftInfoTopic) {
      // Keep first camera_info (static intrinsics for rectified stream).
      if (!left_info) {
        left_info = deserialize<sensor_msgs::msg::CameraInfo>(*message);
      }
    } else if (topic == kRightInfoTopic) {
      if (!right_info) {
        right_info = deserialize<sensor_msgs::msg::CameraInfo>(*message);
      }
    } else if (topic == kStaticTfTopic) {
      auto tf = deserialize<tf2_msgs::msg::TFMessage>(*message);
      for (auto& t : tf.transforms) {
        static_transforms.push_back(std::move(t));
      }
    } else if (topic == kOdomTopic) {
      auto odom = deserialize<nav_msgs::msg::Odometry>(*message);
      Pose2 pose{odom.pose.pose.position.x, odom.pose.pose.position.y,
                 yawFromQuaternion(odom.pose.pose.orientation)};
      odom_se2.emplace_back(stampNs(odom.header.stamp), pose);
    }
  }

  for (const char* topic : {kLeftImageTopic, kRightImageTopic, kLeftInfoTopic,
                            kRightInfoTopic, kStaticTfTopic}) {
    if (!seen[topic]) {
      throw std::runtime_error(std::string("missing required topic ") + topic);
    }
  }
  if (!left_info || !right_info) {
    throw std::runtime_error("missing camera_info messages");
  }

  StaticCameraMount mount;
  try {
    mount = buildStaticCameraMount(static_transforms, kBaseFrame,
                                   kCameraLinkFrame, kLeftOpticalFrame);
  } catch (const std::runtime_error& error) {
    throw std::runtime_error(std::string("failed to resolve camera mount from "
                                         "/tf_static: ") +
                             error.what());
  }

  std::vector<int64_t> left_stamps;
  left_stamps.reserve(left_images.size());
  for (const auto& img : left_images) {
    left_stamps.push_back(stampNs(img.header.stamp));
  }
  std::vector<int64_t> right_stamps;
  right_stamps.reserve(right_images.size());
  for (const auto& img : right_images) {
    right_stamps.push_back(stampNs(img.header.stamp));
  }

  const auto pairs =
      syncStereoStamps(left_stamps, right_stamps, kMaxStereoSkewNs);

  StereoDataset data;
  data.left_info = *left_info;
  data.right_info = *right_info;
  data.recorded_mount = mount;
  data.recorded_camera_link_xy = {
      mount.T_base_camera_link.translation().x(),
      mount.T_base_camera_link.translation().y(),
  };
  data.odom_se2 = std::move(odom_se2);
  data.left_optical_frame = kLeftOpticalFrame;
  data.right_optical_frame = kRightOpticalFrame;

  // Prefer frame_id from camera_info / images when present.
  if (!data.left_info.header.frame_id.empty()) {
    data.left_optical_frame = data.left_info.header.frame_id;
  }
  if (!data.right_info.header.frame_id.empty()) {
    data.right_optical_frame = data.right_info.header.frame_id;
  }

  data.frames.reserve(pairs.size());
  for (const auto& [li, ri, stamp] : pairs) {
    StereoFrame frame;
    frame.stamp_ns = stamp;
    frame.left_bgr_or_gray = toMonoGray(left_images[li]);
    frame.right_bgr_or_gray = toMonoGray(right_images[ri]);
    data.frames.push_back(std::move(frame));
  }

  std::cerr << "\n[  5%] bag loaded: stereo_frames=" << data.frames.size()
            << " left_raw=" << left_images.size()
            << " right_raw=" << right_images.size()
            << " odom=" << data.odom_se2.size() << '\n';
  return data;
}

}  // namespace orb_slam3_wrapper
