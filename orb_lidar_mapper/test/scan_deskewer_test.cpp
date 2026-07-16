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

ScanValue threeRayScan(std::int64_t stamp_ns, float time_increment) {
  ScanValue scan = oneRay(stamp_ns, 0.0F, 2.0F);
  scan.time_increment = time_increment;
  scan.ranges = {2.0F, 2.0F, 2.0F};
  return scan;
}

TimedPoseBuffer stationaryWheelBufferForSweep() {
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  EXPECT_TRUE(wheels.push({0, Pose2{}}));
  EXPECT_TRUE(wheels.push({50'000'000, Pose2{}}));
  EXPECT_TRUE(wheels.push({100'000'000, Pose2{}}));
  return wheels;
}

ScanMotionBracket stationaryBracketForSweep() {
  return {0, 100'000'000, Pose2{}, Pose2{}, Pose2{}, Pose2{}};
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

TEST(ScanDeskewer, SkipsRaysWithoutWheelInterpolationButKeepsCoveredRays) {
  // Ray 0 at t=0 is covered by the wheel buffer; ray 1 at t=200ms exceeds the
  // 100ms gap and cannot be interpolated. The scan must NOT be discarded — the
  // covered ray is kept and only the stale ray is skipped. (Real lidar scans
  // sweep ~99ms, so the sweep tail routinely outruns the live wheel buffer;
  // rejecting the whole scan starves the map of nearly all data.)
  ScanValue scan = oneRay(0, 0.0F, 1.0F);
  scan.time_increment = 0.2F;
  scan.ranges.push_back(1.0F);
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({0, Pose2{}}));
  ASSERT_TRUE(wheels.push({200'000'000, Pose2{}}));

  const auto rays = ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, wheels);
  ASSERT_TRUE(rays.has_value());
  ASSERT_EQ(rays->size(), 1U) << "covered ray 0 kept, stale ray 1 skipped";
  EXPECT_NEAR(rays->front().end.x, 1.0, 1e-9);
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

TEST(ScanDeskewer, ReturnsEmptyWhenEveryRayHasStaleInterpolation) {
  // If NO ray can be interpolated, deskew yields an empty ray set (the caller
  // treats an empty result as "nothing to map from this scan"); it still does
  // not return nullopt, reserving that for malformed scan fields / overflow.
  ScanValue scan = oneRay(0, 0.0F, 1.0F);
  scan.time_increment = 0.2F;          // single ray at t=0
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({300'000'000, Pose2{}}));
  ASSERT_TRUE(wheels.push({500'000'000, Pose2{}}));  // buffer starts after the ray

  const auto rays = ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, wheels);
  ASSERT_TRUE(rays.has_value());
  EXPECT_TRUE(rays->empty());
}

TEST(ScanDeskewer, SkipsInfinityNoReturnBeamWithoutClearingRay) {
  // No-return / blocked beams report +inf. They must be ignored entirely (no
  // clearing ray), so rotating a blocked angle over the map cannot erase
  // previously-observed occupied cells.
  const ScanValue scan = oneRay(0, 0.0F, std::numeric_limits<float>::infinity());
  const auto rays = ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, stationaryWheelBuffer());
  ASSERT_TRUE(rays.has_value());
  EXPECT_TRUE(rays->empty()) << "no-return beam must not produce a (clearing) ray";
}

TEST(ScanDeskewer, IgnoresBeamsBeyondTwentyMeterCap) {
  // range_max is high enough not to be the limiter; the hardcoded 20 m cap is.
  ScanValue scan = oneRay(0, 0.0F, 22.0F);
  scan.range_max = 30.0F;
  const auto far = ScanDeskewer::deskew(scan, Pose2{}, Pose2{}, stationaryWheelBuffer());
  ASSERT_TRUE(far.has_value());
  EXPECT_TRUE(far->empty()) << "beam beyond 20 m must be ignored";

  ScanValue near_scan = oneRay(0, 0.0F, 18.0F);
  near_scan.range_max = 30.0F;
  const auto near = ScanDeskewer::deskew(near_scan, Pose2{}, Pose2{}, stationaryWheelBuffer());
  ASSERT_TRUE(near.has_value());
  ASSERT_EQ(near->size(), 1U) << "beam within 20 m must be kept as a hit";
  EXPECT_TRUE(near->front().has_hit);
  EXPECT_NEAR(near->front().end.x, 18.0, 1e-6);
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

TEST(ScanDeskewer, BracketedDeskewDistributesVisualResidualAcrossSweep) {
  const ScanValue scan = threeRayScan(0, 0.05F);
  const ScanMotionBracket bracket{
    0, 100'000'000, Pose2{}, Pose2{1.0, 0.0, 0.0}, Pose2{}, Pose2{}};

  const auto result = ScanDeskewer::deskewBracketed(
    scan, Pose2{}, stationaryWheelBufferForSweep(), bracket);

  ASSERT_TRUE(result);
  ASSERT_EQ(result->rays.size(), 3U);
  ASSERT_EQ(result->ray_motions.size(), 3U);
  EXPECT_NEAR(result->ray_motions[0].alpha, 0.0, 1e-12);
  EXPECT_NEAR(result->ray_motions[1].alpha, 0.5, 1e-12);
  EXPECT_NEAR(result->ray_motions[2].alpha, 1.0, 1e-12);
  EXPECT_NEAR(result->rays[0].end.x, 2.0, 1e-12);
  EXPECT_NEAR(result->rays[1].end.x, 2.5, 1e-12);
  EXPECT_NEAR(result->rays[2].end.x, 3.0, 1e-12);
  EXPECT_NEAR(result->rays[0].end.y, 0.0, 1e-12);
  EXPECT_NEAR(result->rays[1].end.y, 0.0, 1e-12);
  EXPECT_NEAR(result->rays[2].end.y, 0.0, 1e-12);
}

TEST(ScanDeskewer, BracketedDeskewPreservesNonMicrosecondCadence) {
  const ScanValue scan = threeRayScan(0, 0.00000125F);
  TimedPoseBuffer wheels(1'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(wheels.push({0, Pose2{}}));
  ASSERT_TRUE(wheels.push({1'250, Pose2{}}));
  ASSERT_TRUE(wheels.push({2'500, Pose2{}}));
  const ScanMotionBracket bracket{
    0, 2'500, Pose2{}, Pose2{1.0, 0.0, 0.0}, Pose2{}, Pose2{}};

  const auto result = ScanDeskewer::deskewBracketed(scan, Pose2{}, wheels, bracket);

  ASSERT_TRUE(result);
  ASSERT_EQ(result->rays.size(), 3U);
  ASSERT_EQ(result->ray_motions.size(), 3U);
  EXPECT_NEAR(result->ray_motions[0].alpha, 0.0, 1e-12);
  EXPECT_NEAR(result->ray_motions[1].alpha, 0.5, 1e-12);
  EXPECT_NEAR(result->ray_motions[2].alpha, 1.0, 1e-12);
  EXPECT_NEAR(result->rays[0].end.x, 2.0, 1e-12);
  EXPECT_NEAR(result->rays[1].end.x, 2.5, 1e-12);
  EXPECT_NEAR(result->rays[2].end.x, 3.0, 1e-12);
}

TEST(ScanDeskewer, BracketedDeskewPreservesRawCadenceBeyondOneNanosecondTail) {
  ScanValue scan = threeRayScan(0, 0.05F);
  scan.ranges.push_back(2.0F);
  TimedPoseBuffer wheels(1'000'000'000LL, 200'000'000LL);
  ASSERT_TRUE(wheels.push({0, Pose2{}}));
  ASSERT_TRUE(wheels.push({150'000'002, Pose2{}}));
  const ScanMotionBracket bracket{
    0, 150'000'002, Pose2{}, Pose2{1.0, 0.0, 0.0}, Pose2{}, Pose2{}};

  const auto result = ScanDeskewer::deskewBracketed(scan, Pose2{}, wheels, bracket);

  ASSERT_TRUE(result);
  ASSERT_EQ(result->ray_motions.size(), 4U);
  EXPECT_NEAR(result->ray_motions.back().alpha, 1.0, 1e-12);
  EXPECT_NEAR(result->rays.back().end.x, 3.0, 1e-12);
}

TEST(ScanDeskewer, RplidarFilteredNanAndInfinityCreateNoClearingEvidence) {
  ScanValue scan = threeRayScan(0, 0.05F);
  scan.ranges = {2.0F, std::numeric_limits<float>::quiet_NaN(),
                 std::numeric_limits<float>::infinity()};
  const auto result = ScanDeskewer::deskewBracketed(
    scan, Pose2{}, stationaryWheelBufferForSweep(), stationaryBracketForSweep());
  ASSERT_TRUE(result);
  ASSERT_EQ(result->rays.size(), 1U);
  EXPECT_TRUE(result->rays.front().has_hit);
  ASSERT_EQ(result->ray_motions.size(), 1U);
}

}  // namespace
}  // namespace orb_lidar_mapper
