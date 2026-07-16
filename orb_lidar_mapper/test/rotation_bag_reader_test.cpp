#include <cmath>
#include <filesystem>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
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

struct BagOptions {
  bool include_imu = true;
  bool include_mount = true;
  double mount_y = 0.0;
  double mount_yaw = kPi;
  float raw_angle_min = -1.0F;
  float scan_time_increment = 0.0F;
  float scan_time = 0.0F;
  std::vector<float> raw_ranges = {2.0F, 2.1F};
  std::vector<std::pair<std::int64_t, double>> imu_samples = {{1'000'000'000LL, 0.25}};
};

TempBag writeCalibrationBag(const BagOptions& options = {}) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("rotation-calibration-" + std::to_string(::getpid()));
  std::filesystem::remove_all(path);
  rosbag2_cpp::Writer writer;
  writer.open({path.string(), "mcap"}, {"cdr", "cdr"});

  sensor_msgs::msg::LaserScan raw;
  addHeader(raw.header, 1'000'000'000LL);
  raw.angle_min = options.raw_angle_min;
  raw.angle_max = -0.8F;
  raw.angle_increment = 0.1F;
  raw.time_increment = 0.001F;
  raw.scan_time = 0.1F;
  raw.range_min = 0.1F;
  raw.range_max = 12.0F;
  raw.ranges = options.raw_ranges;
  writer.write(raw, "/scan_origin", rclcpp::Time(1'000'000'000LL));

  sensor_msgs::msg::LaserScan undistorted = raw;
  undistorted.time_increment = options.scan_time_increment;
  undistorted.scan_time = options.scan_time;
  undistorted.ranges = options.raw_ranges;
  writer.write(undistorted, "/scan", rclcpp::Time(1'000'000'000LL));

  nav_msgs::msg::Odometry odom;
  addHeader(odom.header, 1'000'000'000LL);
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.angular.z = 0.25;
  writer.write(odom, "/odom", rclcpp::Time(1'000'000'000LL));

  if (options.include_imu) {
    std::size_t imu_index = 0;
    for (const auto& [stamp_ns, yaw_rate] : options.imu_samples) {
      sensor_msgs::msg::Imu imu;
      addHeader(imu.header, stamp_ns);
      imu.angular_velocity.z = yaw_rate;
      writer.write(imu, "/imu", rclcpp::Time(1'000'000'000LL + imu_index++));
    }
  }

  {
    tf2_msgs::msg::TFMessage tf;
    if (options.include_mount) {
      geometry_msgs::msg::TransformStamped mount;
      mount.header.frame_id = "base_link";
      mount.child_frame_id = "base_scan";
      mount.transform.translation.x = 0.26;
      mount.transform.translation.y = options.mount_y;
      mount.transform.translation.z = 0.10;
      mount.transform.rotation.z = std::sin(options.mount_yaw / 2.0);
      mount.transform.rotation.w = std::cos(options.mount_yaw / 2.0);
      tf.transforms.push_back(mount);
    }
    writer.write(tf, "/tf_static", rclcpp::Time(1'000'000'000LL));
  }
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
  const auto bag = writeCalibrationBag();
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
    BagOptions options;
    options.include_imu = false;
    (void)RotationBagReader::read(writeCalibrationBag(options).path());
    FAIL() << "expected missing /imu to fail";
  } catch (const std::runtime_error& error) {
    EXPECT_THAT(error.what(), testing::HasSubstr("/imu"));
  }
  rclcpp::shutdown();
}

TEST(RotationBagReader, AveragesContiguousDuplicateImuStamps) {
  rclcpp::init(0, nullptr);
  BagOptions options;
  options.imu_samples = {{1'000'000'000LL, 0.1},
                         {1'000'000'000LL, 0.3},
                         {1'000'000'000LL, 0.5},
                         {1'011'000'000LL, 0.7}};
  RotationDataset data;
  try {
    data = RotationBagReader::read(writeCalibrationBag(options).path());
  } catch (const std::runtime_error& error) {
    rclcpp::shutdown();
    FAIL() << error.what();
  }
  ASSERT_EQ(data.imu_yaw_rates.size(), 2U);
  EXPECT_EQ(data.imu_yaw_rates[0].stamp_ns, 1'000'000'000LL);
  EXPECT_DOUBLE_EQ(data.imu_yaw_rates[0].omega_rad_s, 0.3);
  EXPECT_EQ(data.imu_yaw_rates[1].stamp_ns, 1'011'000'000LL);
  EXPECT_DOUBLE_EQ(data.imu_yaw_rates[1].omega_rad_s, 0.7);
  rclcpp::shutdown();
}

TEST(RotationBagReader, RejectsDecreasingImuStamps) {
  rclcpp::init(0, nullptr);
  BagOptions options;
  options.imu_samples = {{1'000'000'000LL, 0.1},
                         {999'000'000LL, 0.3}};
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(options).path()),
               std::runtime_error);
  rclcpp::shutdown();
}

TEST(RotationBagReader, PreservesNoReturnRangeValuesAndZeroScanTiming) {
  rclcpp::init(0, nullptr);
  BagOptions options;
  options.raw_ranges = {2.0F, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()};
  const auto data = RotationBagReader::read(writeCalibrationBag(options).path());
  ASSERT_EQ(data.undistorted_scans.size(), 1U);
  EXPECT_TRUE(std::isnan(data.undistorted_scans.front().ranges[1]));
  EXPECT_TRUE(std::isinf(data.undistorted_scans.front().ranges[2]));
  EXPECT_EQ(data.undistorted_scans.front().time_increment, 0.0);
  rclcpp::shutdown();
}

TEST(RotationBagReader, RejectsNonzeroExistingScanTiming) {
  rclcpp::init(0, nullptr);
  BagOptions options;
  options.scan_time_increment = 0.001F;
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(options).path()),
               std::runtime_error);
  options.scan_time_increment = 0.0F;
  options.scan_time = 0.001F;
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(options).path()),
               std::runtime_error);
  rclcpp::shutdown();
}

TEST(RotationBagReader, RejectsNonFiniteScanMetadata) {
  rclcpp::init(0, nullptr);
  BagOptions options;
  options.raw_angle_min = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(options).path()),
               std::runtime_error);
  options = BagOptions{};
  options.scan_time = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(options).path()),
               std::runtime_error);
  rclcpp::shutdown();
}

TEST(RotationBagReader, RejectsInvalidRecordedMountConstraints) {
  rclcpp::init(0, nullptr);
  BagOptions lateral;
  lateral.mount_y = 0.01;
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(lateral).path()),
               std::runtime_error);
  BagOptions yaw;
  yaw.mount_yaw = 0.0;
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(yaw).path()),
               std::runtime_error);
  BagOptions missing;
  missing.include_mount = false;
  EXPECT_THROW(RotationBagReader::read(writeCalibrationBag(missing).path()),
               std::runtime_error);
  rclcpp::shutdown();
}

TEST(MotionSelector, RejectsTranslationAndKeepsStableRotation) {
  const auto intervals = selectStableRotationIntervals(
    datasetWithMotionSamples(), 0.15, 0.45, 0.02, 1'000'000'000LL);
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_EQ(intervals.front().start_ns, 0);
  EXPECT_EQ(intervals.front().end_ns, 1'000'000'000LL);
}

TEST(MotionSelector, DoesNotBridgeOdometryDropout) {
  RotationDataset data;
  data.odom_twists = {
    {0, {0.0, 0.0, 0.30}}, {100'000'000LL, {0.0, 0.0, 0.30}},
    {200'000'000LL, {0.0, 0.0, 0.30}},
    {10'000'000'000LL, {0.0, 0.0, 0.30}},
    {10'100'000'000LL, {0.0, 0.0, 0.30}},
  };
  const auto intervals = selectStableRotationIntervals(
    data, 0.15, 0.45, 0.02, 500'000'000LL);
  EXPECT_TRUE(intervals.empty());
}

}  // namespace
}  // namespace orb_lidar_mapper
