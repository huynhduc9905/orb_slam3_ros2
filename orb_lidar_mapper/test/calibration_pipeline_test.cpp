#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <cmath>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_pipeline.hpp"

namespace orb_lidar_mapper {
namespace {

Point2 applyPose(const Pose2& pose, Point2 point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

std::vector<Point2> syntheticRoom() {
  return {{-2.0, -1.0}, {-1.2, -1.0}, {-0.4, -1.0}, {0.7, -1.0}, {1.8, -1.0},
          {2.3, -0.3}, {2.3, 0.4}, {1.7, 1.4}, {0.2, 1.7}, {-0.8, 1.3},
          {-1.7, 1.1}, {-2.1, 0.2}, {-0.4, 0.15}, {0.7, 0.5}, {1.1, -0.2},
          {-1.6, 0.5}, {-1.0, 0.7}, {1.6, 0.8}, {0.9, 1.3}, {-1.4, -0.4}};
}

RotationDataset syntheticCalibrationDataset() {
  RotationDataset dataset;
  dataset.recorded_mount = {0.260, 0.0, 0.0, kPi};
  std::vector<Point2> world = {
      {-2.0, -1.0}, {-1.2, -1.0}, {-0.4, -1.0}, {0.7, -1.0}, {1.8, -1.0},
      {2.3, -0.3}, {2.3, 0.4}, {1.7, 1.4}, {0.2, 1.7}, {-0.8, 1.3},
      {-1.7, 1.1}, {-2.1, 0.2}, {-0.4, 0.15}, {0.7, 0.5}, {1.1, -0.2},
      {-1.6, 0.5}, {-1.0, 0.7}, {1.6, 0.8}, {0.9, 1.3}, {-1.4, -0.4}};
  constexpr std::int64_t scan_period = 500'000'000LL;
  constexpr std::int64_t end = 25'000'000'000LL;
  for (std::int64_t stamp = 0; stamp <= end; stamp += 100'000'000LL) {
    const double yaw = -0.4 * static_cast<double>(stamp) / 1e9;
    dataset.odom_poses.push_back({stamp, {0.0, 0.0, yaw}});
    dataset.odom_twists.push_back({stamp, {0.0, 0.0, -0.4}});
    dataset.imu_yaw_rates.push_back({stamp, -0.4});
  }
  for (std::uint64_t id = 0; id <= 50; ++id) {
    const auto stamp = static_cast<std::int64_t>(id) * scan_period;
    const Pose2 lidar_pose = Pose2{0.0, 0.0, -0.4 * stamp / 1e9} *
                             Pose2{0.245, 0.0, kPi};
    ScanValue scan{id, stamp, static_cast<float>(-kPi),
                   static_cast<float>(2.0 * kPi / 720.0), 0.0F, 0.1F, 12.0F,
                   std::vector<float>(720, std::numeric_limits<float>::quiet_NaN())};
    for (const auto point : world) {
      const auto local = lidar_pose.inverse() * Pose2{point.x, point.y, 0.0};
      const double range = std::hypot(local.x, local.y);
      const double angle = std::atan2(local.y, local.x);
      const auto bin = static_cast<std::size_t>(std::llround((angle + kPi) /
          (2.0 * kPi / 720.0)));
      if (bin < scan.ranges.size() && range >= 0.15 && range <= 12.0) {
        scan.ranges[bin] = static_cast<float>(range);
      }
    }
    dataset.raw_scans.push_back(scan);
    auto undistorted = scan;
    undistorted.id = id + 1000;
    dataset.undistorted_scans.push_back(undistorted);
  }
  return dataset;
}

CalibrationPreparedDataset preparedSyntheticCalibrationDataset() {
  CalibrationPreparedDataset prepared;
  prepared.dataset = syntheticCalibrationDataset();
  const auto room = syntheticRoom();
  std::vector<DeskewedScan> odom;
  std::vector<DeskewedScan> existing;
  std::vector<DeskewedScan> imu;
  constexpr double true_offset = 0.245;
  for (std::uint64_t id = 0; id <= 50; ++id) {
    const auto stamp = static_cast<std::int64_t>(id) * 500'000'000LL;
    const Pose2 base_pose{0.0, 0.0, -0.2 * static_cast<double>(id)};
    const Pose2 lidar_pose = base_pose * Pose2{true_offset, 0.0, kPi};
    const Pose2 world_to_lidar = lidar_pose.inverse();
    std::vector<Point2> points;
    for (const auto point : room) points.push_back(applyPose(world_to_lidar, point));
    odom.push_back({id, stamp, DeskewMethod::kOdom, points});
    existing.push_back({id, stamp, DeskewMethod::kExistingScan, points});
    imu.push_back({id, stamp, DeskewMethod::kImu, points});
  }
  prepared.odom_scans = odom;
  prepared.existing_scans = existing;
  prepared.imu_scans_by_iteration = {imu, imu};
  for (const auto& pose : prepared.dataset.odom_poses) {
    prepared.sharpness_odom_poses.push_back(pose);
  }
  return prepared;
}

TEST(CalibrationPipeline, MapsScientificResultsToExactExitCodes) {
  EXPECT_EQ(resultExitCode(ResultClass::kConsistent), 0);
  EXPECT_EQ(resultExitCode(ResultClass::kLikelyOffsetError), 2);
  EXPECT_EQ(resultExitCode(ResultClass::kInconclusive), 3);
}

TEST(CalibrationPipeline, UsesMedianReliableMethodHintAndRecordedFallback) {
  std::array<MethodEstimate, 3> estimates{};
  estimates[0].reliable = false;
  estimates[1].reliable = true;
  estimates[1].forward_offset_m = 0.230;
  estimates[2].reliable = true;
  estimates[2].forward_offset_m = 0.250;
  EXPECT_NEAR(preliminarySharpnessHint(estimates), 0.240, 1e-12);
  estimates[1].reliable = false;
  estimates[2].reliable = false;
  EXPECT_DOUBLE_EQ(preliminarySharpnessHint(estimates), 0.260);
}

TEST(CalibrationPipeline, BoundsAndBalancesCommonSchedule) {
  std::vector<DeskewedScan> scans;
  TimedPoseBuffer odom(20'000'000'000LL, 1'000'000'000LL);
  for (std::size_t i = 0; i < 96; ++i) {
    const auto stamp = static_cast<std::int64_t>(i) * 100'000'000LL;
    odom.push({stamp, {0.0, 0.0, static_cast<double>(i) * 0.08}});
    scans.push_back({i, stamp, DeskewMethod::kOdom, {{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}}});
  }
  const auto pairs = selectCommonCalibrationSchedule(
    scans, odom, {{0, 9'500'000'000LL}}, 64);
  ASSERT_LE(pairs.size(), 64U);
  std::array<std::size_t, 8> sectors{};
  for (const auto& pair : pairs) ++sectors[pair.yaw_sector];
  const auto covered = std::count_if(sectors.begin(), sectors.end(), [](std::size_t count) { return count > 0; });
  EXPECT_GE(covered, 6);
  const auto repeated = selectCommonCalibrationSchedule(scans, odom, {{0, 9'500'000'000LL}}, 64);
  ASSERT_EQ(pairs.size(), repeated.size());
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    EXPECT_EQ(pairs[i].source_index, repeated[i].source_index);
    EXPECT_EQ(pairs[i].target_index, repeated[i].target_index);
    EXPECT_EQ(pairs[i].yaw_sector, repeated[i].yaw_sector);
  }
}

TEST(CalibrationPipeline, RecomputesImuDiagnosticsAfterNonConvergence) {
  std::vector<CenterSample> samples;
  for (std::size_t index = 0; index < 48; ++index) {
    CenterSample sample;
    sample.method = DeskewMethod::kImu;
    sample.accepted = true;
    sample.yaw_sector = index % 8;
    sample.center = {0.260, 0.001};
    sample.icp.trimmed_rmse_m = 0.01;
    sample.icp.overlap_ratio = 0.8;
    samples.push_back(sample);
  }
  const auto estimate = finalizeNonConvergedImuEstimate(samples);
  EXPECT_EQ(estimate.attempted_pairs, 48U);
  EXPECT_EQ(estimate.accepted_pairs, 0U);
  EXPECT_EQ(estimate.rejection_counts.at("imu_offset_not_converged"), 48U);
  EXPECT_EQ(estimate.rejection_counts.at("no_accepted_pairs"), 1U);
  EXPECT_FALSE(estimate.reliable);
  EXPECT_TRUE(std::all_of(samples.begin(), samples.end(), [](const auto& sample) {
    return !sample.accepted && sample.rejection_reason == "imu_offset_not_converged";
  }));
}

TEST(CalibrationPipeline, RunsAllMethodsAndIteratesImuOffset) {
  CalibrationConfig config;
  config.bag_path = "/synthetic/fixture.mcap";
  config.output_dir = std::filesystem::temp_directory_path() / "task5-synthetic-pipeline";
  config.icp.max_correspondence_m = 2.0;
  const auto run = runCalibrationDataset(config, preparedSyntheticCalibrationDataset());
  ASSERT_EQ(run.methods.size(), 3U);
  for (const auto& method : run.methods) {
    EXPECT_TRUE(method.reliable) << static_cast<int>(method.method);
    EXPECT_NEAR(method.forward_offset_m, 0.245, 0.005);
    EXPECT_GE(method.accepted_pairs, 40U);
  }
  EXPECT_GT(run.sharpness.coarse.size(), 0U);
  EXPECT_GT(run.sharpness.refined.size(), 0U);
  EXPECT_GT(run.imu_iterations, 1U);
  EXPECT_EQ(run.aggregate.classification, ResultClass::kLikelyOffsetError);
}

TEST(CalibrationPipeline, RejectsUnreadableBagAsOperationalFailure) {
  CalibrationConfig config;
  config.bag_path = "/path/that/does/not/exist";
  config.output_dir = std::filesystem::temp_directory_path() / "missing-bag-output";
  EXPECT_THROW(runCalibration(config), std::runtime_error);
}

TEST(CalibrationPipeline, ValidatesConfigurationBeforeBagIo) {
  CalibrationConfig config;
  config.output_dir = std::filesystem::temp_directory_path() / "invalid-config-output";
  config.range_cap_m = -1.0;
  EXPECT_THROW(runCalibration(config), std::invalid_argument);
}

}  // namespace
}  // namespace orb_lidar_mapper
