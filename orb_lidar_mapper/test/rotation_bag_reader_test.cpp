#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "orb_lidar_mapper/rotation_bag_reader.hpp"

namespace orb_lidar_mapper {
namespace {

class TempBag {
 public:
  explicit TempBag(std::filesystem::path path) : path_(std::move(path)) {}
  ~TempBag() { std::filesystem::remove_all(path_); }
  const std::filesystem::path& path() const { return path_; }
 private:
  std::filesystem::path path_;
};

void addHeader(std_msgs::msg::Header& header, std::int64_t stamp_ns) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  header.frame_id = "base_link";
}

TempBag writeCalibrationBag(bool include_imu) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("rotation-calibration-" + std::to_string(::getpid()));
  std::filesystem::remove_all(path);
  rosbag2_cpp::Writer writer;
  writer.open({path.string(), "mcap"}, {"cdr", "cdr"});

  sensor_msgs::msg::LaserScan raw;
  addHeader(raw.header, 1'000'000'000LL);
  raw.angle_min = -1.0F;
  raw.angle_increment = 0.1F;
  raw.range_min = 0.1F;
  raw.range_max = 12.0F;
  raw.ranges = {2.0F, 2.1F};
  writer.write(raw, "/scan_origin", rclcpp::Time(1'000'000'000LL));

  sensor_msgs::msg::LaserScan undistorted = raw;
  undistorted.ranges = {2.0F};
  writer.write(undistorted, "/scan", rclcpp::Time(1'000'000'000LL));

  nav_msgs::msg::Odometry odom;
  addHeader(odom.header, 1'000'000'000LL);
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.angular.z = 0.25;
  writer.write(odom, "/odom", rclcpp::Time(1'000'000'000LL));

  if (include_imu) {
    sensor_msgs::msg::Imu imu;
    addHeader(imu.header, 1'000'000'000LL);
    imu.angular_velocity.z = 0.25;
    writer.write(imu, "/imu", rclcpp::Time(1'000'000'000LL));
  }

  tf2_msgs::msg::TFMessage tf;
  geometry_msgs::msg::TransformStamped mount;
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "base_scan";
  mount.transform.translation.x = 0.26;
  mount.transform.translation.z = 0.10;
  mount.transform.rotation.z = 1.0;
  mount.transform.rotation.w = 0.0;
  tf.transforms.push_back(mount);
  writer.write(tf, "/tf_static", rclcpp::Time(1'000'000'000LL));
  return TempBag(path);
}

RotationDataset datasetWithMotionSamples() {
  RotationDataset data;
  data.odom_twists = {
    {0, {0.0, 0.0, 0.30}},
    {1'000'000'000LL, {0.0, 0.0, 0.30}},
    {2'000'000'000LL, {0.03, 0.0, 0.30}},
    {3'000'000'000LL, {0.0, 0.0, 0.30}},
  };
  return data;
}

TEST(RotationBagReader, ReadsTopicsAndRecordedMount) {
  rclcpp::init(0, nullptr);
  const auto bag = writeCalibrationBag(true);
  const auto data = RotationBagReader::read(bag.path());
  ASSERT_EQ(data.raw_scans.size(), 1U);
  ASSERT_EQ(data.undistorted_scans.size(), 1U);
  EXPECT_NEAR(data.recorded_mount.x_m, 0.26, 1e-12);
  EXPECT_NEAR(data.recorded_mount.yaw_rad, kPi, 1e-9);
  rclcpp::shutdown();
}

TEST(RotationBagReader, MissingImuFailsClosed) {
  rclcpp::init(0, nullptr);
  try {
    (void)RotationBagReader::read(writeCalibrationBag(false).path());
    FAIL() << "expected missing /imu to fail";
  } catch (const std::runtime_error& error) {
    EXPECT_THAT(error.what(), testing::HasSubstr("/imu"));
  }
  rclcpp::shutdown();
}

TEST(MotionSelector, RejectsTranslationAndKeepsStableRotation) {
  const auto intervals = selectStableRotationIntervals(
    datasetWithMotionSamples(), 0.15, 0.45, 0.02, 1'000'000'000LL);
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_EQ(intervals.front().start_ns, 0);
  EXPECT_EQ(intervals.front().end_ns, 1'000'000'000LL);
}

}  // namespace
}  // namespace orb_lidar_mapper
