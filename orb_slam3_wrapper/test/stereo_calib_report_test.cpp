#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "orb_slam3_wrapper/stereo_calib_pipeline.hpp"
#include "orb_slam3_wrapper/stereo_calib_report.hpp"
#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {
namespace {

StereoCalibRun fixture(const std::filesystem::path& output,
                       bool inconclusive = false) {
  StereoCalibRun run;
  run.config.bag_path = "/immutable/stereo-fixture.mcap";
  run.config.output_dir = output;
  run.config.overwrite = false;
  run.config.thresholds.min_accepted_pairs = 40;
  run.config.thresholds.min_sectors = 6;
  run.config.thresholds.max_ci_half_width_m = 0.015;
  run.config.thresholds.agreement_floor_m = 0.010;
  run.config.thresholds.max_abs_center_m = 1.0;
  run.config.thresholds.min_pair_yaw_rad = 10.0 * kPi / 180.0;
  run.config.thresholds.max_pair_yaw_rad = 30.0 * kPi / 180.0;
  run.config.max_tracking_loss_fraction = 0.25;
  run.config.min_tracked_frames = 200;

  run.dataset.recorded_mount.T_base_camera_link.setIdentity();
  run.dataset.recorded_mount.T_base_camera_link.translation() << 0.326, 0.0,
      0.173;
  run.dataset.recorded_mount.T_camera_link_left_optical.setIdentity();
  run.dataset.recorded_camera_link_xy = {0.326, 0.0};

  TrackedPose tp;
  tp.stamp_ns = 1'000'000'000LL;
  tp.T_world_optical.setIdentity();
  tp.T_world_optical.translation() << 0.32, 0.05, 0.17;
  tp.tracking_state = 2;
  tp.pose_valid = true;
  run.trajectory.push_back(tp);
  tp.stamp_ns = 2'000'000'000LL;
  tp.T_world_optical.translation() << 0.30, 0.10, 0.17;
  run.trajectory.push_back(tp);

  PlanarPose pp;
  pp.stamp_ns = 1'000'000'000LL;
  pp.pose = {0.32, 0.05, 0.0};
  pp.valid = true;
  run.planar.push_back(pp);
  pp.stamp_ns = 2'000'000'000LL;
  pp.pose = {0.30, 0.10, 0.2};
  run.planar.push_back(pp);

  StereoCenterSample accepted;
  accepted.source_index = 0;
  accepted.target_index = 1;
  accepted.yaw_sector = 0;
  accepted.accepted = true;
  accepted.center = {-0.32, -0.05};
  accepted.mount_xy = {0.320, 0.001};
  run.samples.push_back(accepted);

  StereoCenterSample rejected;
  rejected.source_index = 1;
  rejected.target_index = 0;
  rejected.yaw_sector = 1;
  rejected.accepted = false;
  rejected.center = {0.5, 0.0};
  rejected.mount_xy = {0.5, 0.0};
  rejected.rejection_reason = "quote, \"comma\" and newline\nreason";
  run.samples.push_back(rejected);

  run.aggregate.recorded_xy = {0.326, 0.0};
  run.aggregate.estimate.reliable = true;
  run.aggregate.estimate.median_xy = {0.320, 0.001};
  run.aggregate.estimate.ci_half_width = {0.005, 0.004};
  run.aggregate.estimate.accepted_pairs = 48;
  run.aggregate.estimate.sectors_used = 8;
  run.aggregate.delta_xy = {-0.006, 0.001};
  run.aggregate.result_class = ResultClass::kLikelyOffsetError;
  run.aggregate.summary = "likely_offset_error";

  run.tracked_ok = 240;
  run.tracked_total = 250;

  if (inconclusive) {
    run.aggregate.result_class = ResultClass::kInconclusive;
    run.aggregate.summary = "inconclusive";
    run.aggregate.estimate.reliable = false;
    run.aggregate.estimate.median_xy = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()};
    run.aggregate.estimate.ci_half_width = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()};
    run.aggregate.delta_xy = {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()};
    run.aggregate.estimate.unreliable_reasons = {"too_few_pairs",
                                                 "broad_ci"};
  }
  return run;
}

std::string readFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  return std::string((std::istreambuf_iterator<char>(input)), {});
}

TEST(StereoCalibReport, WritesSelfContainedOutputs) {
  const auto output =
      std::filesystem::temp_directory_path() / "stereo-calib-report-test";
  std::filesystem::remove_all(output);

  ASSERT_NO_THROW(writeStereoCalibrationReport(fixture(output)));

  for (const auto& name :
       {"calibration.json", "centers.csv", "trajectory.csv", "report.html"}) {
    EXPECT_TRUE(std::filesystem::is_regular_file(output / name)) << name;
  }

  // Must never write TF / URDF artifacts.
  EXPECT_FALSE(std::filesystem::exists(output / "tf_static.yaml"));
  EXPECT_FALSE(std::filesystem::exists(output / "tf.yaml"));
  EXPECT_FALSE(std::filesystem::exists(output / "urdf"));

  const auto json = readFile(output / "calibration.json");
  EXPECT_NE(json.find("LIKELY_OFFSET_ERROR"), std::string::npos);
  EXPECT_NE(json.find("\"exit_code_meaning\""), std::string::npos);
  EXPECT_NE(json.find("\"recorded_camera_link_xy\""), std::string::npos);
  EXPECT_NE(json.find("\"estimated_camera_link_xy\""), std::string::npos);
  EXPECT_NE(json.find("\"delta_xy\""), std::string::npos);
  EXPECT_NE(json.find("\"ci_half_width\""), std::string::npos);
  EXPECT_NE(json.find("\"accepted_pairs\":48"), std::string::npos);
  EXPECT_NE(json.find("\"sectors_used\":8"), std::string::npos);
  EXPECT_NE(json.find("\"tracked_ok\":240"), std::string::npos);
  EXPECT_NE(json.find("\"tracked_total\":250"), std::string::npos);
  EXPECT_NE(json.find("\"thresholds\""), std::string::npos);
  EXPECT_NE(json.find("stereo-fixture.mcap"), std::string::npos);
  EXPECT_NE(json.find("\"recorded_T_base_camera_link\""), std::string::npos);
  EXPECT_NE(json.find("\"summary\""), std::string::npos);
  EXPECT_NE(json.find("likely_offset_error"), std::string::npos);
  EXPECT_NE(json.find("0.326"), std::string::npos);
  EXPECT_NE(json.find("0.320"), std::string::npos);

  const auto centers = readFile(output / "centers.csv");
  EXPECT_NE(centers.find("source_index"), std::string::npos);
  EXPECT_NE(centers.find("\"\"comma\"\""), std::string::npos);

  const auto traj = readFile(output / "trajectory.csv");
  EXPECT_NE(traj.find("stamp_ns"), std::string::npos);
  EXPECT_NE(traj.find("tracking_state"), std::string::npos);
  EXPECT_NE(traj.find("1000000000"), std::string::npos);

  const auto html = readFile(output / "report.html");
  EXPECT_NE(html.find("Stereo rotation-center"), std::string::npos);
  EXPECT_NE(html.find("LIKELY_OFFSET_ERROR"), std::string::npos);
  EXPECT_NE(html.find("<svg"), std::string::npos);
  EXPECT_EQ(html.find("src=\"http"), std::string::npos);
  EXPECT_EQ(html.find("href=\"http"), std::string::npos);

  std::filesystem::remove_all(output);
}

TEST(StereoCalibReport, WritesParseableNullsForNonFinite) {
  const auto output = std::filesystem::temp_directory_path() /
                      "stereo-calib-report-inconclusive";
  std::filesystem::remove_all(output);

  ASSERT_NO_THROW(writeStereoCalibrationReport(fixture(output, true)));

  const auto json = readFile(output / "calibration.json");
  EXPECT_EQ(json.find("nan"), std::string::npos);
  EXPECT_EQ(json.find("inf"), std::string::npos);
  EXPECT_NE(json.find(":null"), std::string::npos);
  EXPECT_NE(json.find("INCONCLUSIVE"), std::string::npos);
  EXPECT_NE(json.find("too_few_pairs"), std::string::npos);
  EXPECT_NE(json.find("\"unreliable_reasons\""), std::string::npos);

  std::filesystem::remove_all(output);
}

TEST(StereoCalibReport, RefusesOverwriteUnlessExplicitlyEnabled) {
  const auto output = std::filesystem::temp_directory_path() /
                      "stereo-calib-report-overwrite";
  std::filesystem::remove_all(output);

  ASSERT_NO_THROW(writeStereoCalibrationReport(fixture(output)));
  EXPECT_THROW(writeStereoCalibrationReport(fixture(output)),
               std::runtime_error);

  auto overwrite = fixture(output);
  overwrite.config.overwrite = true;
  EXPECT_NO_THROW(writeStereoCalibrationReport(overwrite));

  std::filesystem::remove_all(output);
}

TEST(StereoCalibReport, CreatesOutputDirectory) {
  const auto output = std::filesystem::temp_directory_path() /
                      "stereo-calib-report-nested" / "deep" / "out";
  std::filesystem::remove_all(std::filesystem::temp_directory_path() /
                              "stereo-calib-report-nested");

  ASSERT_NO_THROW(writeStereoCalibrationReport(fixture(output)));
  EXPECT_TRUE(std::filesystem::is_regular_file(output / "calibration.json"));

  std::filesystem::remove_all(std::filesystem::temp_directory_path() /
                              "stereo-calib-report-nested");
}

}  // namespace
}  // namespace orb_slam3_wrapper
