#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_report.hpp"

namespace orb_lidar_mapper {
namespace {

CalibrationRun fixture(const std::filesystem::path& output) {
  CalibrationRun run;
  run.config.output_dir = output;
  run.config.bag_path = "/immutable/fixture.mcap";
  run.dataset.recorded_mount = {0.260, 0.0, 0.0, kPi};
  run.dataset.raw_scans.push_back({1, 0, -1.0F, 0.5F, 0.0F, 0.1F, 12.0F, {2.0F, 3.0F, 2.5F}});
  run.dataset.odom_poses = {{0, {0.0, 0.0, 0.0}}, {1'000'000'000LL, {0.1, 0.0, 0.2}}};
  for (std::size_t i = 0; i < run.methods.size(); ++i) {
    auto& method = run.methods[i];
    method.method = static_cast<DeskewMethod>(i);
    method.reliable = true;
    method.center_x_m = 0.245;
    method.center_y_m = 0.001;
    method.forward_offset_m = 0.245;
    method.confidence_95_m = {0.240, 0.250};
    method.accepted_pairs = 48;
    method.attempted_pairs = 50;
    method.covered_yaw_sectors = 8;
    method.median_rmse_m = 0.012;
    method.median_overlap = 0.72;
    method.rejection_counts["poor_overlap"] = 2;
  }
  CenterSample rejected;
  rejected.method = DeskewMethod::kOdom;
  rejected.source_scan_id = 11;
  rejected.target_scan_id = 12;
  rejected.accepted = false;
  rejected.rejection_reason = "poor_overlap";
  run.center_samples.push_back(rejected);
  for (std::size_t i = 0; i < 3; ++i) {
    CenterSample accepted;
    accepted.method = static_cast<DeskewMethod>(i);
    accepted.source_scan_id = 20 + i;
    accepted.target_scan_id = 30 + i;
    accepted.accepted = true;
    accepted.center = {0.245 + 0.001 * static_cast<double>(i), 0.002};
    accepted.icp.trimmed_rmse_m = 0.012;
    accepted.icp.overlap_ratio = 0.72;
    run.center_samples.push_back(accepted);
  }
  run.sharpness.reliable = true;
  run.sharpness.best_offset_m = 0.245;
  run.sharpness.coarse = {{0.180, 0.4}, {0.260, 0.2}, {0.340, 0.4}};
  run.sharpness.refined = {{0.244, 0.21}, {0.245, 0.20}, {0.246, 0.21}};
  run.aggregate.classification = ResultClass::kLikelyOffsetError;
  run.aggregate.consensus_offset_m = 0.245;
  run.aggregate.confidence_95_m = {0.240, 0.250};
  run.aggregate.reason = "fixture";
  return run;
}

TEST(CalibrationReport, WritesSelfContainedOutputsAtomically) {
  const auto output = std::getenv("TASK5_REPORT_FIXTURE_DIR") != nullptr
    ? std::filesystem::path(std::getenv("TASK5_REPORT_FIXTURE_DIR"))
    : std::filesystem::temp_directory_path() / "lidar-calibration-report-test";
  std::filesystem::remove_all(output);
  ASSERT_NO_THROW(writeCalibrationReport(fixture(output)));
  for (const auto& name : {"calibration.json", "centers.csv", "sharpness.csv", "report.html"}) {
    EXPECT_TRUE(std::filesystem::is_regular_file(output / name));
  }
  std::ifstream json(output / "calibration.json");
  const std::string json_text((std::istreambuf_iterator<char>(json)), {});
  EXPECT_NE(json_text.find("LIKELY_OFFSET_ERROR"), std::string::npos);
  EXPECT_NE(json_text.find("\"coarse\":["), std::string::npos);
  EXPECT_NE(json_text.find("\"refined\":["), std::string::npos);
  EXPECT_NE(json_text.find("\"center_samples\":["), std::string::npos);
  std::ifstream centers(output / "centers.csv");
  const std::string centers_text((std::istreambuf_iterator<char>(centers)), {});
  EXPECT_NE(centers_text.find("poor_overlap"), std::string::npos);
  std::ifstream sharpness(output / "sharpness.csv");
  const std::string sharpness_text((std::istreambuf_iterator<char>(sharpness)), {});
  EXPECT_NE(sharpness_text.find("0.180"), std::string::npos);
  std::ifstream html(output / "report.html");
  const std::string html_text((std::istreambuf_iterator<char>(html)), {});
  EXPECT_NE(html_text.find("Existing /scan"), std::string::npos);
  EXPECT_NE(html_text.find("Raw center scatter"), std::string::npos);
  EXPECT_NE(html_text.find("Rejected center samples"), std::string::npos);
  EXPECT_EQ(html_text.find("src=\"http"), std::string::npos);
  EXPECT_EQ(html_text.find("href=\"http"), std::string::npos);
  if (std::getenv("TASK5_REPORT_FIXTURE_DIR") == nullptr) std::filesystem::remove_all(output);
}

TEST(CalibrationReport, RefusesOverwriteUnlessExplicitlyEnabled) {
  const auto output = std::getenv("TASK5_REPORT_FIXTURE_DIR") != nullptr
    ? std::filesystem::path(std::getenv("TASK5_REPORT_FIXTURE_DIR"))
    : std::filesystem::temp_directory_path() / "lidar-calibration-overwrite-test";
  std::filesystem::remove_all(output);
  ASSERT_NO_THROW(writeCalibrationReport(fixture(output)));
  EXPECT_THROW(writeCalibrationReport(fixture(output)), std::runtime_error);
  auto overwrite = fixture(output);
  overwrite.config.overwrite = true;
  EXPECT_NO_THROW(writeCalibrationReport(overwrite));
  if (std::getenv("TASK5_REPORT_FIXTURE_DIR") == nullptr) std::filesystem::remove_all(output);
}

}  // namespace
}  // namespace orb_lidar_mapper
