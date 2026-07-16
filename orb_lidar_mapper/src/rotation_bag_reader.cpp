#include "orb_lidar_mapper/rotation_bag_reader.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace orb_lidar_mapper {
namespace {

constexpr const char* kRawTopic = "/scan_origin";
constexpr const char* kUndistortedTopic = "/scan";
constexpr const char* kOdomTopic = "/odom";
constexpr const char* kImuTopic = "/imu";
constexpr const char* kStaticTfTopic = "/tf_static";

std::int64_t stampNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

template<typename MessageT>
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
    throw std::runtime_error("invalid quaternion in /tf_static or /odom");
  }
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void requireFinite(double value, const std::string& detail) {
  if (!std::isfinite(value)) {
    throw std::runtime_error("non-finite " + detail);
  }
}

void requireMonotonic(std::int64_t stamp, std::int64_t& previous,
                      bool& have_previous, const char* topic) {
  if (have_previous && stamp <= previous) {
    throw std::runtime_error(std::string("non-monotonic stamps on ") + topic);
  }
  previous = stamp;
  have_previous = true;
}

ScanValue convertScan(const sensor_msgs::msg::LaserScan& message, std::uint64_t id) {
  requireFinite(message.angle_min, "scan angle_min");
  requireFinite(message.angle_increment, "scan angle_increment");
  requireFinite(message.range_min, "scan range_min");
  requireFinite(message.range_max, "scan range_max");
  if (message.ranges.empty()) {
    throw std::runtime_error("empty scan");
  }
  ScanValue result;
  result.id = id;
  result.stamp_ns = stampNs(message.header.stamp);
  result.angle_min = message.angle_min;
  result.angle_increment = message.angle_increment;
  result.time_increment = message.time_increment;
  result.range_min = message.range_min;
  result.range_max = message.range_max;
  result.ranges = message.ranges;
  return result;
}

}  // namespace

RotationDataset RotationBagReader::read(const std::filesystem::path& bag_path) {
  if (!std::filesystem::exists(bag_path)) {
    throw std::runtime_error("bag path does not exist: " + bag_path.string());
  }

  rosbag2_cpp::Reader reader;
  reader.open({bag_path.string(), "mcap"}, {"cdr", "cdr"});
  RotationDataset data;
  std::map<std::string, bool> seen;
  std::int64_t previous_raw = 0, previous_undistorted = 0, previous_odom = 0,
               previous_imu = 0;
  bool have_raw = false, have_undistorted = false, have_odom = false, have_imu = false;
  bool have_mount = false;
  std::uint64_t raw_id = 0, undistorted_id = 0;

  while (reader.has_next()) {
    const auto message = reader.read_next();
    const auto& topic = message->topic_name;
    seen[topic] = true;
    if (topic == kRawTopic) {
      auto scan = deserialize<sensor_msgs::msg::LaserScan>(*message);
      const auto stamp = stampNs(scan.header.stamp);
      requireMonotonic(stamp, previous_raw, have_raw, kRawTopic);
      data.raw_scans.push_back(convertScan(scan, raw_id++));
    } else if (topic == kUndistortedTopic) {
      auto scan = deserialize<sensor_msgs::msg::LaserScan>(*message);
      const auto stamp = stampNs(scan.header.stamp);
      requireMonotonic(stamp, previous_undistorted, have_undistorted, kUndistortedTopic);
      data.undistorted_scans.push_back(convertScan(scan, undistorted_id++));
    } else if (topic == kOdomTopic) {
      auto odom = deserialize<nav_msgs::msg::Odometry>(*message);
      const auto stamp = stampNs(odom.header.stamp);
      requireMonotonic(stamp, previous_odom, have_odom, kOdomTopic);
      Pose2 pose{odom.pose.pose.position.x, odom.pose.pose.position.y,
                 yawFromQuaternion(odom.pose.pose.orientation)};
      Twist2 twist{odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                   odom.twist.twist.angular.z};
      requireFinite(pose.x, "/odom pose.x");
      requireFinite(pose.y, "/odom pose.y");
      requireFinite(pose.yaw, "/odom pose.yaw");
      requireFinite(twist.vx, "/odom twist.vx");
      requireFinite(twist.vy, "/odom twist.vy");
      requireFinite(twist.omega, "/odom twist.omega");
      data.odom_poses.push_back({stamp, pose});
      data.odom_twists.push_back({stamp, twist});
    } else if (topic == kImuTopic) {
      auto imu = deserialize<sensor_msgs::msg::Imu>(*message);
      const auto stamp = stampNs(imu.header.stamp);
      requireMonotonic(stamp, previous_imu, have_imu, kImuTopic);
      requireFinite(imu.angular_velocity.z, "/imu angular_velocity.z");
      data.imu_yaw_rates.push_back({stamp, imu.angular_velocity.z});
    } else if (topic == kStaticTfTopic) {
      auto tf = deserialize<tf2_msgs::msg::TFMessage>(*message);
      for (const auto& transform : tf.transforms) {
        if (transform.header.frame_id != "base_link" ||
            transform.child_frame_id != "base_scan") {
          continue;
        }
        if (have_mount) {
          throw std::runtime_error("duplicate recorded base_link -> base_scan edge");
        }
        data.recorded_mount.x_m = transform.transform.translation.x;
        data.recorded_mount.y_m = transform.transform.translation.y;
        data.recorded_mount.z_m = transform.transform.translation.z;
        data.recorded_mount.yaw_rad = yawFromQuaternion(transform.transform.rotation);
        requireFinite(data.recorded_mount.x_m, "recorded mount x");
        requireFinite(data.recorded_mount.y_m, "recorded mount y");
        requireFinite(data.recorded_mount.z_m, "recorded mount z");
        requireFinite(data.recorded_mount.yaw_rad, "recorded mount yaw");
        have_mount = true;
      }
    }
  }

  for (const char* topic : {kRawTopic, kUndistortedTopic, kOdomTopic, kImuTopic, kStaticTfTopic}) {
    if (!seen[topic]) {
      throw std::runtime_error(std::string("missing required topic ") + topic);
    }
  }
  if (!have_mount) {
    throw std::runtime_error("missing recorded base_link -> base_scan edge on /tf_static");
  }
  if (std::abs(data.recorded_mount.y_m) > 1e-6) {
    throw std::runtime_error("recorded base_scan.y is not zero");
  }
  if (std::abs(Pose2::normalizeAngle(data.recorded_mount.yaw_rad - kPi)) > 1e-6) {
    throw std::runtime_error("recorded base_scan.yaw is not pi");
  }
  return data;
}

std::vector<MotionInterval> selectStableRotationIntervals(
  const RotationDataset& dataset, double min_abs_omega, double max_abs_omega,
  double max_abs_linear_speed, std::int64_t minimum_duration_ns) {
  if (!std::isfinite(min_abs_omega) || !std::isfinite(max_abs_omega) ||
      !std::isfinite(max_abs_linear_speed) || min_abs_omega < 0.0 ||
      max_abs_omega < min_abs_omega || max_abs_linear_speed < 0.0 ||
      minimum_duration_ns < 0) {
    throw std::invalid_argument("invalid stable-rotation selector limits");
  }
  std::vector<MotionInterval> intervals;
  std::optional<MotionInterval> current;
  for (const auto& sample : dataset.odom_twists) {
    const double abs_omega = std::abs(sample.twist.omega);
    const double linear_speed = std::hypot(sample.twist.vx, sample.twist.vy);
    const bool stable = std::isfinite(abs_omega) && std::isfinite(linear_speed) &&
                        abs_omega >= min_abs_omega && abs_omega <= max_abs_omega &&
                        linear_speed <= max_abs_linear_speed;
    if (!stable) {
      if (current && current->end_ns - current->start_ns >= minimum_duration_ns) {
        intervals.push_back(*current);
      }
      current.reset();
      continue;
    }
    if (!current) {
      current = MotionInterval{sample.stamp_ns, sample.stamp_ns};
    } else {
      current->end_ns = sample.stamp_ns;
    }
  }
  if (current && current->end_ns - current->start_ns >= minimum_duration_ns) {
    intervals.push_back(*current);
  }
  return intervals;
}

}  // namespace orb_lidar_mapper
