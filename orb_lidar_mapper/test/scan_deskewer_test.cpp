#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/scan_deskewer.hpp"

namespace orb_lidar_mapper {
namespace {

TimedPoseBuffer stationaryWheelBuffer() {
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  EXPECT_TRUE(wheels.push({0, Pose2{}}));
  EXPECT_TRUE(wheels.push({100'000'000, Pose2{}}));
  return wheels;
}

ScanValue oneRay(std::int64_t stamp_ns, float angle, float range) {
  ScanValue scan;
  scan.stamp_ns = stamp_ns;
  scan.angle_min = angle;
  scan.range_min = 0.1F;
  scan.range_max = 10.0F;
  scan.ranges = {range};
  return scan;
}

TEST(ScanDeskewer, AppliesForwardLidarOffsetBeforeGlobalPose) {
  const ScanValue scan = oneRay(0, 0.0F, 1.0F);
  const auto rays = ScanDeskewer::deskew(
      scan, Pose2{}, Pose2{0.26, 0.0, M_PI}, stationaryWheelBuffer());
  ASSERT_TRUE(rays.has_value());
  ASSERT_EQ(rays->size(), 1U);
  EXPECT_NEAR((*rays)[0].end.x, -0.74, 1e-9);
}

TEST(ScanDeskewer, UsesEachRayTimestampAndRotatesAroundBaseCenter) {
  ScanValue scan;
  scan.stamp_ns = 0;
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.0F;
  scan.time_increment = 0.1F;
  scan.range_min = 0.1F;
  scan.range_max = 10.0F;
  scan.ranges = {1.0F, 1.0F};
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({0, Pose2{}}));
  ASSERT_TRUE(wheels.push({100'000'001, Pose2{0.0, 0.0, M_PI_2}}));

  const auto rays = ScanDeskewer::deskew(scan, Pose2{}, Pose2{0.26, 0.0, 0.0}, wheels);

  ASSERT_TRUE(rays.has_value());
  ASSERT_EQ(rays->size(), 2U);
  EXPECT_NEAR((*rays)[0].origin.x, 0.26, 1e-9);
  EXPECT_NEAR((*rays)[1].origin.x, 0.0, 1e-9);
  EXPECT_NEAR((*rays)[1].origin.y, 0.26, 1e-9);
  EXPECT_NEAR((*rays)[1].end.x, 0.0, 1e-9);
  EXPECT_NEAR((*rays)[1].end.y, 1.26, 1e-9);
}

TEST(ScanDeskewer, RejectsEntireScanWhenAnyRayWheelInterpolationIsUnavailable) {
  ScanValue scan = oneRay(0, 0.0F, 1.0F);
  scan.time_increment = 0.2F;
  scan.ranges.push_back(1.0F);
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({0, Pose2{}}));
  ASSERT_TRUE(wheels.push({200'000'000, Pose2{}}));

  EXPECT_FALSE(ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, wheels).has_value());
}

TEST(ScanDeskewer, RejectsTimestampArithmeticOverflow) {
  ScanValue scan = oneRay(std::numeric_limits<std::int64_t>::max(), 0.0F, 1.0F);
  scan.time_increment = 1.0F;
  scan.ranges.push_back(1.0F);
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({std::numeric_limits<std::int64_t>::max() - 1'000'000'000LL,
                           Pose2{}}));
  ASSERT_TRUE(wheels.push({std::numeric_limits<std::int64_t>::max(), Pose2{}}));

  EXPECT_FALSE(ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, wheels).has_value());
}

TEST(ScanDeskewer, SkipsInvalidBeamBeforeItsStaleInterpolation) {
  ScanValue scan = oneRay(0, 0.0F, 1.0F);
  scan.time_increment = 0.2F;
  scan.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({0, Pose2{}}));
  ASSERT_TRUE(wheels.push({200'000'000, Pose2{}}));

  const auto rays = ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, wheels);
  ASSERT_TRUE(rays.has_value());
  ASSERT_EQ(rays->size(), 1U);
  EXPECT_NEAR(rays->front().end.x, 1.0, 1e-9);
}

TEST(ScanDeskewer, RejectsScanWhenAValidBeamHasStaleInterpolation) {
  ScanValue scan = oneRay(0, 0.0F, 1.0F);
  scan.time_increment = 0.2F;
  scan.ranges.push_back(1.0F);
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({0, Pose2{}}));
  ASSERT_TRUE(wheels.push({200'000'000, Pose2{}}));

  EXPECT_FALSE(ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, wheels).has_value());
}

TEST(ScanDeskewer, RepresentsInfinityAsFiniteRangeMaxClearingRay) {
  const ScanValue scan = oneRay(0, 0.0F, std::numeric_limits<float>::infinity());
  const auto rays = ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, stationaryWheelBuffer());
  ASSERT_TRUE(rays.has_value());
  ASSERT_EQ(rays->size(), 1U);
  EXPECT_FALSE(rays->front().has_hit);
  EXPECT_TRUE(std::isfinite(rays->front().end.x));
  EXPECT_TRUE(std::isfinite(rays->front().end.y));
  EXPECT_NEAR(rays->front().end.x, scan.range_max, 1e-6);
}

TEST(ScanDeskewer, RejectsInvalidScanFields) {
  const auto invalid = [](ScanValue scan) {
    EXPECT_FALSE(ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, stationaryWheelBuffer()).has_value());
  };
  ScanValue scan = oneRay(0, 0.0F, 1.0F);
  scan.range_min = -0.1F;
  invalid(scan);
  scan = oneRay(0, 0.0F, 1.0F);
  scan.range_max = std::numeric_limits<float>::infinity();
  invalid(scan);
  scan = oneRay(0, 0.0F, 1.0F);
  scan.angle_min = std::numeric_limits<float>::quiet_NaN();
  invalid(scan);
  scan = oneRay(0, 0.0F, 1.0F);
  scan.angle_increment = std::numeric_limits<float>::infinity();
  invalid(scan);
  scan = oneRay(0, 0.0F, 1.0F);
  scan.time_increment = std::numeric_limits<float>::quiet_NaN();
  invalid(scan);
}

}  // namespace
}  // namespace orb_lidar_mapper
