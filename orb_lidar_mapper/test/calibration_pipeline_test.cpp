#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_pipeline.hpp"

namespace orb_lidar_mapper {
namespace {

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

TEST(CalibrationPipeline, RejectsUnreadableBagAsOperationalFailure) {
  CalibrationConfig config;
  config.bag_path = "/path/that/does/not/exist";
  config.output_dir = std::filesystem::temp_directory_path() / "missing-bag-output";
  EXPECT_THROW(runCalibration(config), std::runtime_error);
}

}  // namespace
}  // namespace orb_lidar_mapper
