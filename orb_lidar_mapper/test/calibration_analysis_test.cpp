#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_analysis.hpp"

namespace orb_lidar_mapper {
namespace {

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

MethodEstimate methodFixture(double offset) {
  MethodEstimate result;
  result.method = DeskewMethod::kOdom;
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
  RotationDataset dataset;
  std::vector<DeskewedScan> scans;
  TimedPoseBuffer odom(10000000000LL, 1000000000LL);
  for (std::size_t i = 0; i < 12; ++i) {
    const auto stamp = static_cast<std::int64_t>(i) * 100000000LL;
    odom.push({stamp, {0.0, 0.0, 0.2 * static_cast<double>(i)}});
    DeskewedScan scan;
    scan.scan_id = i;
    scan.reference_stamp_ns = stamp;
    scan.method = DeskewMethod::kOdom;
    for (std::size_t j = 0; j < 20; ++j) {
      const double angle = 2.0 * kPi * static_cast<double>(j) / 20.0;
      scan.points.push_back({0.26 + std::cos(angle), std::sin(angle)});
    }
    scans.push_back(scan);
  }
  const auto result = evaluateMapSharpness(dataset, scans, odom, 0.26);
  EXPECT_EQ(result.coarse.size(), 81u);
  EXPECT_FALSE(result.refined.empty());
  EXPECT_GE(result.best_offset_m, 0.18);
  EXPECT_LE(result.best_offset_m, 0.34);
}

TEST(CalibrationAnalysis, ClassifiesConsistentOffsetErrorAndDisagreement) {
  const auto sharp = SharpnessResult{true, 0.260, {}, {}, {}};
  EXPECT_EQ(classifyCalibration(
              {methodFixture(0.257), methodFixture(0.260), methodFixture(0.262)},
              sharp, 0.260).classification,
            ResultClass::kConsistent);
  EXPECT_EQ(classifyCalibration(
              {methodFixture(0.230), methodFixture(0.232), methodFixture(0.234)},
              SharpnessResult{true, 0.232, {}, {}, {}}, 0.260).classification,
            ResultClass::kLikelyOffsetError);
  EXPECT_EQ(classifyCalibration(
              {methodFixture(0.230), methodFixture(0.260), methodFixture(0.290)},
              SharpnessResult{true, 0.260, {}, {}, {}}, 0.260).classification,
            ResultClass::kInconclusive);
}

TEST(CalibrationAnalysis, RecordsExactRecordedOffsetEquation) {
  const auto result = classifyCalibration(
    {methodFixture(0.280), methodFixture(0.281)},
    SharpnessResult{true, 0.280, {}, {}, {}}, 0.260);
  EXPECT_NEAR(result.consensus_offset_m, 0.2805, 0.002);
  EXPECT_NEAR(result.consensus_offset_m - 0.260, 0.0205, 0.002);
}

}  // namespace
}  // namespace orb_lidar_mapper
