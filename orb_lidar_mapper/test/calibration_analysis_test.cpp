#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_analysis.hpp"

namespace orb_lidar_mapper {
namespace {

Point2 applyPose(const Pose2& pose, Point2 point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

CenterSample acceptedCenter(double x, double y, std::size_t sector) {
  CenterSample sample;
  sample.method = DeskewMethod::kOdom;
  sample.source_scan_id = sector * 2;
  sample.target_scan_id = sector * 2 + 1;
  sample.yaw_sector = sector;
  sample.accepted = true;
  sample.center = {x, y};
  sample.icp.converged = true;
  sample.icp.trimmed_rmse_m = 0.01;
  sample.icp.overlap_ratio = 0.8;
  return sample;
}

std::vector<CenterSample> samplesAround(double center, double spread,
                                         std::size_t count) {
  std::vector<CenterSample> samples;
  samples.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    samples.push_back(acceptedCenter(
      center + spread * static_cast<double>(static_cast<int>(i % 7) - 3),
      0.001 * static_cast<double>(static_cast<int>(i % 5) - 2), i % 8));
  }
  return samples;
}

MethodEstimate methodFixture(double offset,
                             DeskewMethod method = DeskewMethod::kOdom) {
  MethodEstimate result;
  result.method = method;
  result.reliable = true;
  result.center_x_m = offset;
  result.forward_offset_m = offset;
  result.confidence_95_m = {offset - 0.002, offset + 0.002};
  result.accepted_pairs = 60;
  result.attempted_pairs = 60;
  result.covered_yaw_sectors = 8;
  result.median_rmse_m = 0.01;
  result.median_overlap = 0.8;
  return result;
}

TEST(CalibrationAnalysis, DuplicateMethodRowsDoNotSatisfyTwoMethodGate) {
  const auto result = classifyCalibration(
    {methodFixture(0.260), methodFixture(0.261)},
    SharpnessResult{true, 0.260, {}, {}, {}}, 0.260);

  EXPECT_EQ(result.classification, ResultClass::kInconclusive);
  EXPECT_EQ(result.reason, "duplicate_method_estimate");
}

TEST(CalibrationAnalysis, RejectsOutliersButPreservesRawRows) {
  auto samples = samplesAround(0.26, 0.002, 80);
  samples.push_back(acceptedCenter(0.90, 0.20, 3));
  const auto result = robustMethodEstimate(DeskewMethod::kOdom, samples);
  EXPECT_TRUE(result.reliable);
  EXPECT_NEAR(result.forward_offset_m, 0.26, 0.003);
  EXPECT_EQ(result.attempted_pairs, samples.size());
  EXPECT_EQ(result.accepted_pairs, samples.size());
}

TEST(CalibrationAnalysis, SectorDeficiencyIsUnreliable) {
  std::vector<CenterSample> samples;
  for (std::size_t i = 0; i < 100; ++i) {
    samples.push_back(acceptedCenter(0.26, 0.0, 0));
  }
  const auto result = robustMethodEstimate(DeskewMethod::kOdom, samples);
  EXPECT_FALSE(result.reliable);
  EXPECT_EQ(result.covered_yaw_sectors, 1u);
}

TEST(CalibrationAnalysis, BootstrapIsDeterministicAndCountsRejections) {
  auto samples = samplesAround(0.26, 0.001, 48);
  samples.push_back(acceptedCenter(0.26, 0.0, 0));
  samples.back().accepted = false;
  samples.back().rejection_reason = "poor_overlap";
  const auto first = robustMethodEstimate(DeskewMethod::kOdom, samples, 17);
  const auto second = robustMethodEstimate(DeskewMethod::kOdom, samples, 17);
  EXPECT_EQ(first.confidence_95_m.low_m, second.confidence_95_m.low_m);
  EXPECT_EQ(first.confidence_95_m.high_m, second.confidence_95_m.high_m);
  EXPECT_EQ(first.rejection_counts.at("poor_overlap"), 1u);
  EXPECT_EQ(first.attempted_pairs, 49u);
}

TEST(CalibrationAnalysis, SharpnessUsesExactGridsAndFindsUniqueMinimum) {
  constexpr double kTrueOffset = 0.261;
  RotationDataset dataset;
  std::vector<DeskewedScan> scans;
  TimedPoseBuffer odom(10000000000LL, 1000000000LL);
  const std::vector<Point2> world_points{
    {-2.4, -1.1}, {-1.7, 0.2}, {-1.2, 2.3}, {-0.4, -2.0},
    {0.1, 0.7}, {0.8, -1.4}, {1.3, 2.1}, {2.0, -0.3},
    {2.7, 1.4}, {3.2, -1.8}, {-2.8, 1.6}, {1.9, 0.9},
  };
  for (std::size_t i = 0; i < 16; ++i) {
    const auto stamp = static_cast<std::int64_t>(i) * 100000000LL;
    const Pose2 base_pose{0.0, 0.0, 2.0 * kPi * static_cast<double>(i) / 16.0};
    odom.push({stamp, base_pose});
    DeskewedScan scan;
    scan.scan_id = i;
    scan.reference_stamp_ns = stamp;
    scan.method = DeskewMethod::kOdom;
    const Pose2 lidar_to_world = base_pose * Pose2{kTrueOffset, 0.0, kPi};
    const Pose2 world_to_lidar = lidar_to_world.inverse();
    for (const auto& point : world_points) {
      const double c = std::cos(world_to_lidar.yaw);
      const double s = std::sin(world_to_lidar.yaw);
      scan.points.push_back({world_to_lidar.x + c * point.x - s * point.y,
                             world_to_lidar.y + s * point.x + c * point.y});
    }
    scans.push_back(scan);
  }
  const auto result = evaluateMapSharpness(dataset, scans, odom, kTrueOffset);
  EXPECT_EQ(result.coarse.size(), 81u);
  EXPECT_FALSE(result.refined.empty());
  EXPECT_TRUE(result.reliable) << result.rejection_reason;
  EXPECT_NEAR(result.best_offset_m, kTrueOffset, 0.0003);
}

TEST(CalibrationAnalysis, SharpnessPairsAreTemporalNonAdjacentBoundedAndDeterministic) {
  std::vector<DeskewedScan> scans;
  TimedPoseBuffer odom(100'000'000'000LL, 100'000'000'000LL);
  for (std::size_t i = 0; i < 80; ++i) {
    const auto stamp = static_cast<std::int64_t>(i) * 100'000'000LL;
    ASSERT_TRUE(odom.push({stamp, {0.0, 0.0, 2.0 * kPi * i / 80.0}}));
    scans.push_back({1000 + i * 2, stamp, DeskewMethod::kOdom,
                     {{-1.0, -0.5}, {0.0, 1.0}, {1.5, -0.2}, {2.0, 0.8}}});
  }
  const auto first = selectSharpnessPairs(scans, odom, 128);
  const auto second = selectSharpnessPairs(scans, odom, 128);
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i].source_index, second[i].source_index);
    EXPECT_EQ(first[i].target_index, second[i].target_index);
  }
  ASSERT_LE(first.size(), 128U);
  ASSERT_FALSE(first.empty());
  for (const auto& pair : first) {
    EXPECT_GE(pair.target_index, pair.source_index + 2U);
    EXPECT_GT(scans[pair.target_index].scan_id, scans[pair.source_index].scan_id + 1U);
  }
}

TEST(CalibrationAnalysis, SharpnessUnevenPointCountsDoNotDominatePairAverage) {
  constexpr double kTrueOffset = 0.261;
  RotationDataset dataset;
  std::vector<DeskewedScan> scans;
  TimedPoseBuffer odom(100'000'000'000LL, 100'000'000'000LL);
  const std::vector<Point2> world_points{
      {-2.4, -1.1}, {-1.7, 0.2}, {-1.2, 2.3}, {-0.4, -2.0},
      {0.1, 0.7}, {0.8, -1.4}, {1.3, 2.1}, {2.0, -0.3},
      {2.7, 1.4}, {3.2, -1.8}, {-2.8, 1.6}, {1.9, 0.9},
  };
  for (std::size_t i = 0; i < 16; ++i) {
    const auto stamp = static_cast<std::int64_t>(i) * 100'000'000LL;
    const Pose2 base_pose{0.0, 0.0, 2.0 * kPi * i / 16.0};
    ASSERT_TRUE(odom.push({stamp, base_pose}));
    const Pose2 world_to_lidar = (base_pose * Pose2{kTrueOffset, 0.0, kPi}).inverse();
    DeskewedScan scan{i, stamp, DeskewMethod::kOdom, {}};
    for (const auto point : world_points) scan.points.push_back(applyPose(world_to_lidar, point));
    scans.push_back(std::move(scan));
  }
  const auto baseline = evaluateMapSharpness(dataset, scans, odom, kTrueOffset);
  auto uneven = scans;
  uneven[0].points.insert(uneven[0].points.end(), 10'000, uneven[0].points.front());
  const auto result = evaluateMapSharpness(dataset, uneven, odom, kTrueOffset);
  EXPECT_TRUE(baseline.reliable) << baseline.rejection_reason;
  EXPECT_TRUE(result.reliable) << result.rejection_reason;
  EXPECT_NEAR(result.best_offset_m, kTrueOffset, 0.0003);
  EXPECT_NEAR(result.best_offset_m, baseline.best_offset_m, 0.0003);
}

TEST(CalibrationAnalysis, SharpnessFlatAndMultimodalMinimaAreRejectedDeterministically) {
  EXPECT_FALSE(sharpnessMinimumIsUnique({{0.20, 1.0}, {0.22, 1.0}, {0.24, 1.0}}));
  EXPECT_FALSE(sharpnessMinimumIsUnique({{0.20, 2.0}, {0.22, 1.0}, {0.24, 2.0},
                                         {0.26, 1.0}, {0.28, 2.0}}));
  EXPECT_TRUE(sharpnessMinimumIsUnique({{0.20, 2.0}, {0.22, 1.0}, {0.24, 2.0}}));
}

TEST(CalibrationAnalysis, ClassifiesConsistentOffsetErrorAndDisagreement) {
  const auto sharp = SharpnessResult{true, 0.260, {}, {}, {}};
  EXPECT_EQ(classifyCalibration(
              {methodFixture(0.257, DeskewMethod::kOdom),
               methodFixture(0.260, DeskewMethod::kImu),
               methodFixture(0.262, DeskewMethod::kExistingScan)},
              sharp, 0.260).classification,
            ResultClass::kConsistent);
  EXPECT_EQ(classifyCalibration(
              {methodFixture(0.230, DeskewMethod::kOdom),
               methodFixture(0.232, DeskewMethod::kImu),
               methodFixture(0.234, DeskewMethod::kExistingScan)},
              SharpnessResult{true, 0.232, {}, {}, {}}, 0.260).classification,
            ResultClass::kLikelyOffsetError);
  EXPECT_EQ(classifyCalibration(
              {methodFixture(0.230, DeskewMethod::kOdom),
               methodFixture(0.260, DeskewMethod::kImu),
               methodFixture(0.290, DeskewMethod::kExistingScan)},
              SharpnessResult{true, 0.260, {}, {}, {}}, 0.260).classification,
            ResultClass::kInconclusive);
}

TEST(CalibrationAnalysis, RecordsExactRecordedOffsetEquation) {
  const auto result = classifyCalibration(
    {methodFixture(0.280, DeskewMethod::kOdom),
     methodFixture(0.281, DeskewMethod::kImu)},
    SharpnessResult{true, 0.280, {}, {}, {}}, 0.260);
  EXPECT_NEAR(result.consensus_offset_m, 0.2805, 0.002);
  EXPECT_NEAR(result.consensus_offset_m - 0.260, 0.0205, 0.002);
}

}  // namespace
}  // namespace orb_lidar_mapper
