#include <cmath>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/rotation_center_estimator.hpp"

namespace orb_lidar_mapper {
namespace {

Point2 apply(const Pose2& pose, Point2 p) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y};
}

Pose2 about(double yaw, Point2 c) {
  return {c.x - std::cos(yaw) * c.x + std::sin(yaw) * c.y,
          c.y - std::sin(yaw) * c.x - std::cos(yaw) * c.y, yaw};
}

DeskewedScan scan(std::uint64_t id, std::int64_t stamp) {
  return {id, stamp, DeskewMethod::kOdom,
          {{-2.0, -1.0}, {-0.4, -1.0}, {1.8, -1.0}, {2.3, 0.4},
           {0.2, 1.7}, {-1.7, 1.1}, {-0.4, 0.15}, {1.1, -0.2}}};
}

DeskewedScan physicalScan(std::uint64_t id, std::int64_t stamp, double base_yaw) {
  const Pose2 lidar_pose = Pose2{0.0, 0.0, base_yaw} * Pose2{0.26, 0.0, kPi};
  const std::vector<Point2> world{
      {-2.0, -1.0}, {-1.3, 0.4}, {-0.5, -1.4}, {0.3, 1.8},
      {1.4, -0.7}, {2.0, 1.1}, {-1.8, 1.5}, {0.9, 0.2},
  };
  const Pose2 world_to_lidar = lidar_pose.inverse();
  std::vector<Point2> points;
  points.reserve(world.size());
  for (const auto point : world) points.push_back(apply(world_to_lidar, point));
  return {id, stamp, DeskewMethod::kOdom, points};
}

TEST(RotationCenter, LocksSourceToTargetSign) {
  const Pose2 motion = about(0.40, {0.26, 0.0});
  const auto center = centerFromTransform(motion);
  ASSERT_TRUE(center);
  EXPECT_NEAR(center->x, +0.26, 1e-9);
  EXPECT_NEAR(center->y, 0.0, 1e-9);
}

TEST(RotationCenter, RejectsNearlyIdentityRotation) {
  EXPECT_FALSE(centerFromTransform(about(1e-8, {0.26, 0.0})));
}

TEST(RotationCenter, SelectsStablePairsAcrossYawSectors) {
  std::vector<DeskewedScan> scans;
  TimedPoseBuffer odom(100'000'000'000LL, 100'000'000'000LL);
  for (int i = 0; i <= 16; ++i) {
    const auto stamp = static_cast<std::int64_t>(i) * 1'000'000'000LL;
    scans.push_back(scan(static_cast<std::uint64_t>(i), stamp));
    ASSERT_TRUE(odom.push({stamp, {0.0, 0.0, 0.04 * i}}));
  }
  const std::vector<MotionInterval> stable{{0, 16'000'000'000LL}};

  const auto pairs = selectCalibrationPairs(scans, odom, stable,
                                            10.0 * kPi / 180.0,
                                            30.0 * kPi / 180.0);
  ASSERT_FALSE(pairs.empty());
  for (const auto& pair : pairs) {
    EXPECT_GE(std::abs(pair.expected_source_to_target_yaw_rad), 10.0 * kPi / 180.0);
    EXPECT_LE(std::abs(pair.expected_source_to_target_yaw_rad), 30.0 * kPi / 180.0);
    EXPECT_LT(pair.source_index, pair.target_index);
    EXPECT_LT(pair.yaw_sector, 8U);
  }
}

TEST(RotationCenter, MultiTurnIntervalStillCoversAbsoluteYawSectors) {
  std::vector<DeskewedScan> scans;
  TimedPoseBuffer odom(100'000'000'000LL, 100'000'000'000LL);
  constexpr int kStepsPerTurn = 16;
  constexpr int kTurns = 2;
  for (int step = 0; step <= kStepsPerTurn * kTurns; ++step) {
    const auto stamp = static_cast<std::int64_t>(step) * 100'000'000LL;
    const double unwrapped_yaw = step * 2.0 * kPi / kStepsPerTurn;
    scans.push_back(scan(static_cast<std::uint64_t>(step), stamp));
    ASSERT_TRUE(odom.push(
        {stamp, {0.0, 0.0, Pose2::normalizeAngle(unwrapped_yaw)}}));
  }
  const std::vector<MotionInterval> stable{
      {0, kStepsPerTurn * kTurns * 100'000'000LL}};

  const auto pairs = selectCalibrationPairs(scans, odom, stable,
                                            20.0 * kPi / 180.0,
                                            25.0 * kPi / 180.0);

  ASSERT_FALSE(pairs.empty());
  std::set<std::size_t> sectors;
  for (const auto& pair : pairs) sectors.insert(pair.yaw_sector);
  EXPECT_EQ(sectors.size(), 8U);
}

TEST(RotationCenter, RejectsPairsOutsideStableIntervals) {
  std::vector<DeskewedScan> scans{scan(1, 0), scan(2, 1'000'000'000LL)};
  TimedPoseBuffer odom(10'000'000'000LL, 10'000'000'000LL);
  ASSERT_TRUE(odom.push({0, Pose2{}}));
  ASSERT_TRUE(odom.push({1'000'000'000LL, Pose2{0.0, 0.0, 0.4}}));
  EXPECT_TRUE(selectCalibrationPairs(scans, odom, {{2'000'000'000LL,
                                                    3'000'000'000LL}},
                                     0.1, 0.5).empty());
}

TEST(RotationCenter, RecordsAcceptedAndRejectedRawCenterSamples) {
  const auto source = scan(4, 0);
  auto target = scan(5, 1'000'000'000LL);
  const Pose2 truth = about(0.35, {0.26, 0.0});
  for (auto& point : target.points) point = apply(truth, point);
  const ScanPair pair{0, 1, 2, 0.35};
  PlanarIcp icp(IcpConfig{});

  const auto accepted = estimateRotationCenter(
      DeskewMethod::kOdom, pair, source, target, icp, 1.0, 0.25);
  EXPECT_TRUE(accepted.accepted) << accepted.rejection_reason;
  EXPECT_EQ(accepted.source_scan_id, 4U);
  EXPECT_EQ(accepted.target_scan_id, 5U);
  EXPECT_NEAR(accepted.center.x, 0.26, 0.01);

  const auto rejected = estimateRotationCenter(
      DeskewMethod::kImu, ScanPair{0, 1, 3, 0.0}, source, target, icp, 1.0,
      0.25);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.rejection_reason, "yaw_disagreement");
  EXPECT_EQ(rejected.icp.rejection_reason, "yaw_disagreement");
}

TEST(RotationCenter, PhysicalSourceToTargetYawAndCenterUseOdomDeltaSign) {
  for (const double target_yaw : {0.35, -0.35}) {
    std::vector<DeskewedScan> scans{
        physicalScan(10, 0, 0.0), physicalScan(11, 1'000'000'000LL, target_yaw)};
    TimedPoseBuffer odom(10'000'000'000LL, 10'000'000'000LL);
    ASSERT_TRUE(odom.push({0, Pose2{0.0, 0.0, 0.0}}));
    ASSERT_TRUE(odom.push({1'000'000'000LL, Pose2{0.0, 0.0, target_yaw}}));
    const auto pairs = selectCalibrationPairs(
        scans, odom, {{0, 1'000'000'000LL}}, 0.30, 0.40);
    ASSERT_EQ(pairs.size(), 1U);
    EXPECT_NEAR(pairs.front().expected_source_to_target_yaw_rad, -target_yaw, 1e-12);
    const auto sample = estimateRotationCenter(
        DeskewMethod::kOdom, pairs.front(), scans[0], scans[1],
        PlanarIcp(IcpConfig{}), 1.0, 0.25);
    ASSERT_TRUE(sample.accepted) << sample.rejection_reason;
    EXPECT_NEAR(sample.icp.source_to_target.yaw, -target_yaw, 0.01);
    EXPECT_NEAR(sample.center.x, 0.26, 0.005);
    EXPECT_NEAR(sample.center.y, 0.0, 0.005);
  }
}

}  // namespace
}  // namespace orb_lidar_mapper
