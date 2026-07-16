#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_deskew.hpp"

namespace orb_lidar_mapper {
namespace {

struct RotatingScanFixture {
  ScanValue raw;
  TimedPoseBuffer odom{1'000'000'000LL, 100'000'000LL};
  std::vector<TimedYawRate> imu;
  StaticLidarMount mount{0.26, 0.0, 0.0, kPi};
  std::vector<Point2> midpoint_points;
};

Point2 transform(const Pose2& pose, Point2 point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

RotatingScanFixture rotatingScanFixture(double offset_m, double omega_rad_s,
                                        double duration_s) {
  RotatingScanFixture fixture;
  fixture.mount.x_m = offset_m;
  fixture.raw.id = 17;
  fixture.raw.stamp_ns = 1'000'000'000LL;
  fixture.raw.angle_min = -1.1F;
  fixture.raw.angle_increment = 0.43F;
  fixture.raw.time_increment = static_cast<float>(duration_s / 5.0);
  fixture.raw.range_min = 0.1F;
  fixture.raw.range_max = 20.0F;
  fixture.raw.ranges = {2.4F, 3.1F, 1.8F, 4.2F, 2.7F, 3.6F};

  const std::int64_t start = fixture.raw.stamp_ns;
  const std::int64_t end = start + static_cast<std::int64_t>(std::llround(
      duration_s * 1'000'000'000.0));
  const std::int64_t midpoint = start + (end - start) / 2;
  for (const std::int64_t stamp : {start, midpoint, end}) {
    const double yaw = omega_rad_s *
      static_cast<double>(stamp - start) / 1'000'000'000.0;
    EXPECT_TRUE(fixture.odom.push({stamp, Pose2{0.0, 0.0, yaw}}));
  }
  for (std::int64_t stamp = start; stamp < end; stamp += 10'000'000LL) {
    fixture.imu.push_back({stamp, omega_rad_s});
  }
  fixture.imu.push_back({end, omega_rad_s});

  const Pose2 midpoint_lidar = Pose2{0.0, 0.0, omega_rad_s * duration_s / 2.0} *
                               Pose2{offset_m, 0.0, kPi};
  for (std::size_t index = 0; index < fixture.raw.ranges.size(); ++index) {
    const auto stamp = start + static_cast<std::int64_t>(std::llround(
        static_cast<double>(index) * duration_s / 5.0 * 1'000'000'000.0));
    const Pose2 ray_lidar = Pose2{0.0, 0.0, omega_rad_s *
        static_cast<double>(stamp - start) / 1'000'000'000.0} *
      Pose2{offset_m, 0.0, kPi};
    const double angle = fixture.raw.angle_min +
                         index * fixture.raw.angle_increment;
    const Point2 ray_point = transform(ray_lidar,
                                       {std::cos(angle) * fixture.raw.ranges[index],
                                        std::sin(angle) * fixture.raw.ranges[index]});
    fixture.midpoint_points.push_back(transform(midpoint_lidar.inverse(), ray_point));
  }
  return fixture;
}

ScanValue rawScan() {
  ScanValue scan;
  scan.id = 1;
  scan.stamp_ns = 0;
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.1F;
  scan.time_increment = 0.01F;
  scan.range_min = 0.1F;
  scan.range_max = 20.0F;
  scan.ranges = {std::numeric_limits<float>::quiet_NaN(),
                 std::numeric_limits<float>::infinity(), 0.10F, 2.0F, 13.0F};
  return scan;
}

TEST(CalibrationDeskew, OdomRecoversMidpointCloud) {
  const auto fixture = rotatingScanFixture(0.26, 0.30, 0.098558);
  const auto result = deskewWithOdom(fixture.raw, fixture.odom, fixture.mount, 12.0);
  ASSERT_TRUE(result);
  ASSERT_EQ(result->scan_id, fixture.raw.id);
  ASSERT_EQ(result->reference_stamp_ns, 1'049'279'000LL);
  ASSERT_EQ(result->method, DeskewMethod::kOdom);
  ASSERT_EQ(result->points.size(), fixture.midpoint_points.size());
  for (std::size_t i = 0; i < result->points.size(); ++i) {
    EXPECT_NEAR(result->points[i].x, fixture.midpoint_points[i].x, 0.002);
    EXPECT_NEAR(result->points[i].y, fixture.midpoint_points[i].y, 0.002);
  }
}

TEST(CalibrationDeskew, ImuModelsLidarArc) {
  const auto fixture = rotatingScanFixture(0.26, 0.30, 0.098558);
  const auto result = deskewWithImu(fixture.raw, fixture.imu, 0.26, kPi, 12.0);
  ASSERT_TRUE(result);
  ASSERT_EQ(result->method, DeskewMethod::kImu);
  ASSERT_EQ(result->reference_stamp_ns, 1'049'279'000LL);
  ASSERT_EQ(result->points.size(), fixture.midpoint_points.size());
  for (std::size_t i = 0; i < result->points.size(); ++i) {
    EXPECT_NEAR(result->points[i].x, fixture.midpoint_points[i].x, 0.002);
    EXPECT_NEAR(result->points[i].y, fixture.midpoint_points[i].y, 0.002);
  }
}

TEST(CalibrationDeskew, MissingCoverageAndInvalidRangesFailClosed) {
  TimedPoseBuffer empty(1'000'000'000LL, 100'000'000LL);
  const StaticLidarMount mount{0.0, 0.0, 0.0, kPi};
  EXPECT_FALSE(deskewWithOdom(rawScan(), empty, mount, 12.0));

  TimedPoseBuffer odom(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(odom.push({0, Pose2{}}));
  ASSERT_TRUE(odom.push({100'000'000, Pose2{}}));
  const auto result = deskewWithOdom(rawScan(), odom, mount, 12.0);
  ASSERT_TRUE(result);
  ASSERT_EQ(result->points.size(), 1U);
  EXPECT_NEAR(result->points.front().x, 2.0 * std::cos(0.3), 1e-8);
}

TEST(CalibrationDeskew, RejectsImuGapsOverTwentyMilliseconds) {
  const auto fixture = rotatingScanFixture(0.26, 0.30, 0.098558);
  auto imu = fixture.imu;
  imu[1].stamp_ns += 21'000'000;
  EXPECT_FALSE(deskewWithImu(fixture.raw, imu, 0.26, kPi, 12.0));
}

TEST(CalibrationDeskew, AssociatesExistingScanByOrderAndMedianDelay) {
  const auto make = [](std::uint64_t id, std::int64_t stamp) {
    ScanValue scan;
    scan.id = id;
    scan.stamp_ns = stamp;
    scan.angle_min = 0.0F;
    scan.angle_increment = 0.1F;
    scan.range_min = 0.1F;
    scan.range_max = 10.0F;
    scan.ranges = {2.0F};
    return scan;
  };
  const std::vector<ScanValue> raw = {make(1, 0), make(2, 100), make(3, 200)};
  const std::vector<ScanValue> filtered = {make(11, 90), make(12, 190), make(13, 290)};
  const auto matches = associateUndistortedScans(raw, filtered, 2);
  ASSERT_EQ(matches.size(), 3U);
  EXPECT_EQ(matches[1].raw_scan_id, 2U);
  EXPECT_EQ(matches[1].undistorted_scan_id, 12U);
  EXPECT_EQ(matches[1].timestamp_delay_ns, 90);
  const auto adapted = adaptUndistortedScan(filtered[1], matches[1], 12.0);
  EXPECT_EQ(adapted.method, DeskewMethod::kExistingScan);
  EXPECT_EQ(adapted.reference_stamp_ns, 190);
  EXPECT_EQ(adapted.points.size(), 1U);

  auto ambiguous = filtered;
  ambiguous[2].stamp_ns = 500;
  EXPECT_TRUE(associateUndistortedScans(raw, ambiguous, 2).empty());
}

}  // namespace
}  // namespace orb_lidar_mapper
