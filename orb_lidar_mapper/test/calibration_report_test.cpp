#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_deskew.hpp"
#include "orb_lidar_mapper/calibration_report.hpp"

namespace orb_lidar_mapper {
namespace {

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

Point2 transform(const Pose2& pose, Point2 point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

std::string extractMapsSection(const std::string& json_text) {
  const auto start = json_text.find("\"maps\":{");
  EXPECT_NE(start, std::string::npos);
  if (start == std::string::npos) return {};
  const auto end = json_text.find("},\"center_samples\"", start);
  EXPECT_NE(end, std::string::npos);
  if (end == std::string::npos) return {};
  return json_text.substr(start, end - start);
}

CalibrationRun fixture(const std::filesystem::path& output, bool inconclusive = false) {
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
  rejected.center = {0.280, -0.006};
  rejected.rejection_reason = "quote, \"comma\" and newline\nreason";
  run.center_samples.push_back(rejected);
  for (std::size_t i = 0; i < 3; ++i) {
    CenterSample accepted;
    accepted.method = static_cast<DeskewMethod>(i);
    accepted.source_scan_id = 20 + i;
    accepted.target_scan_id = 30 + i;
    accepted.accepted = true;
    accepted.center = {i == 0 ? 0.210 : (i == 1 ? 0.235 : 0.300), 0.002};
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
  if (inconclusive) {
    run.methods[1].reliable = false;
    run.methods[1].center_x_m = std::numeric_limits<double>::quiet_NaN();
    run.methods[1].center_y_m = std::numeric_limits<double>::infinity();
    run.methods[1].forward_offset_m = std::numeric_limits<double>::quiet_NaN();
    run.methods[1].confidence_95_m = {std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::infinity()};
    run.methods[1].median_rmse_m = std::numeric_limits<double>::infinity();
    run.methods[1].median_overlap = std::numeric_limits<double>::quiet_NaN();
    run.aggregate.classification = ResultClass::kInconclusive;
    run.aggregate.consensus_offset_m = std::numeric_limits<double>::quiet_NaN();
    run.aggregate.confidence_95_m = {std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity()};
    run.sharpness.reliable = false;
    run.sharpness.best_offset_m = std::numeric_limits<double>::quiet_NaN();
    run.sharpness.coarse.push_back({std::numeric_limits<double>::quiet_NaN(),
                                    std::numeric_limits<double>::infinity()});
    run.sharpness.refined.push_back({std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::quiet_NaN()});
  }
  return run;
}

CalibrationRun rotatingMapFixture(const std::filesystem::path& output,
                                   double consensus_offset_m = 0.245,
                                   bool finite_consensus = true) {
  CalibrationRun run = fixture(output);
  run.dataset = {};
  run.dataset.recorded_mount = {0.260, 0.0, 0.0, kPi};
  run.aggregate.classification = ResultClass::kLikelyOffsetError;
  if (finite_consensus) {
    run.aggregate.consensus_offset_m = consensus_offset_m;
  } else {
    run.aggregate.classification = ResultClass::kInconclusive;
    run.aggregate.consensus_offset_m = std::numeric_limits<double>::quiet_NaN();
  }

  constexpr double omega = 0.40;
  constexpr double duration_s = 0.10;
  constexpr std::int64_t start = 1'000'000'000LL;
  const std::int64_t end =
      start + static_cast<std::int64_t>(std::llround(duration_s * 1e9));

  ScanValue raw;
  raw.id = 7;
  raw.stamp_ns = start;
  raw.angle_min = -1.0F;
  raw.angle_increment = 0.5F;
  raw.time_increment = static_cast<float>(duration_s / 4.0);
  raw.range_min = 0.1F;
  raw.range_max = 20.0F;
  raw.ranges = {2.0F, 2.5F, 3.0F, 2.2F, 2.8F};
  run.dataset.raw_scans.push_back(raw);

  for (const std::int64_t stamp : {start, (start + end) / 2, end}) {
    const double yaw =
        omega * static_cast<double>(stamp - start) / 1'000'000'000.0;
    run.dataset.odom_poses.push_back({stamp, {0.0, 0.0, yaw}});
  }

  // Keep method table fields valid for report writing.
  for (std::size_t i = 0; i < run.methods.size(); ++i) {
    run.methods[i].method = static_cast<DeskewMethod>(i);
    run.methods[i].reliable = finite_consensus;
  }
  return run;
}

std::vector<Point2> expectedDeskewedMapPoints(const RotationDataset& dataset,
                                              double offset_m) {
  std::vector<Point2> points;
  if (dataset.odom_poses.empty() || dataset.raw_scans.empty()) return points;
  const auto retention = dataset.odom_poses.back().stamp_ns -
                         dataset.odom_poses.front().stamp_ns + 1'000'000'000LL;
  TimedPoseBuffer odom(std::max<std::int64_t>(retention, 1'000'000'000LL),
                       1'000'000'000LL);
  for (const auto& pose : dataset.odom_poses) odom.push(pose);

  StaticLidarMount mount{offset_m, 0.0, 0.0, kPi};
  const std::size_t stride =
      std::max<std::size_t>(1, dataset.raw_scans.size() / 80);
  for (std::size_t scan_index = 0; scan_index < dataset.raw_scans.size();
       scan_index += stride) {
    const auto& scan = dataset.raw_scans[scan_index];
    const auto deskewed = deskewWithOdom(scan, odom, mount, 12.0);
    if (!deskewed || deskewed->points.empty()) continue;
    const auto pose = odom.interpolate(deskewed->reference_stamp_ns);
    if (!pose) continue;
    const Pose2 world = *pose * Pose2{offset_m, 0.0, kPi};
    const std::size_t ray_stride =
        std::max<std::size_t>(1, deskewed->points.size() / 180);
    for (std::size_t i = 0; i < deskewed->points.size(); i += ray_stride) {
      points.push_back(transform(world, deskewed->points[i]));
    }
  }
  return points;
}

TEST(CalibrationReport, MapPreviewsUseOdomDeskewedRays) {
  const auto output =
      std::filesystem::temp_directory_path() / "lidar-calibration-deskewed-maps";
  std::filesystem::remove_all(output);
  auto run = rotatingMapFixture(output, 0.245, true);
  ASSERT_NO_THROW(writeCalibrationReport(run));

  std::ifstream json(output / "calibration.json");
  const std::string json_text((std::istreambuf_iterator<char>(json)), {});
  const auto maps = extractMapsSection(json_text);
  ASSERT_FALSE(maps.empty());

  const auto recorded =
      expectedDeskewedMapPoints(run.dataset, run.dataset.recorded_mount.x_m);
  const auto estimated =
      expectedDeskewedMapPoints(run.dataset, run.aggregate.consensus_offset_m);
  ASSERT_FALSE(recorded.empty());
  ASSERT_FALSE(estimated.empty());

  // Deskewed recorded/estimated maps must both appear and must differ when
  // offsets differ (raw undeskewed previews collapse motion blur and hide this).
  for (const auto& point : recorded) {
    EXPECT_NE(maps.find('[' + jsonNumber(point.x) + ',' + jsonNumber(point.y) + ']'),
              std::string::npos);
  }
  for (const auto& point : estimated) {
    EXPECT_NE(maps.find('[' + jsonNumber(point.x) + ',' + jsonNumber(point.y) + ']'),
              std::string::npos);
  }
  EXPECT_NE(recorded.front().x, estimated.front().x);

  // Raw undeskewed projection at scan stamp must NOT be what the report uses.
  {
    const auto& scan = run.dataset.raw_scans.front();
    const auto retention = run.dataset.odom_poses.back().stamp_ns -
                           run.dataset.odom_poses.front().stamp_ns + 1'000'000'000LL;
    TimedPoseBuffer odom(std::max<std::int64_t>(retention, 1'000'000'000LL),
                         1'000'000'000LL);
    for (const auto& pose : run.dataset.odom_poses) odom.push(pose);
    const auto pose = odom.interpolate(scan.stamp_ns);
    ASSERT_TRUE(pose);
    const Pose2 world = *pose * Pose2{0.260, 0.0, kPi};
    const double angle = scan.angle_min;
    const double range = scan.ranges.front();
    const Point2 raw_point{world.x + std::cos(world.yaw + angle) * range,
                           world.y + std::sin(world.yaw + angle) * range};
    // First raw ray at scan start differs from deskewed midpoint cloud.
    const auto raw_token =
        '[' + jsonNumber(raw_point.x) + ',' + jsonNumber(raw_point.y) + ']';
    const auto deskewed_token = '[' + jsonNumber(recorded.front().x) + ',' +
                                jsonNumber(recorded.front().y) + ']';
    EXPECT_NE(raw_token, deskewed_token);
    EXPECT_EQ(maps.find(raw_token), std::string::npos);
  }

  std::filesystem::remove_all(output);
}

TEST(CalibrationReport, OmitsEstimatedMapWhenConsensusIsNonFinite) {
  const auto output =
      std::filesystem::temp_directory_path() / "lidar-calibration-empty-estimated-map";
  std::filesystem::remove_all(output);
  auto run = rotatingMapFixture(output, 0.0, false);
  ASSERT_NO_THROW(writeCalibrationReport(run));

  std::ifstream json(output / "calibration.json");
  const std::string json_text((std::istreambuf_iterator<char>(json)), {});
  EXPECT_NE(json_text.find("\"estimated\":[]"), std::string::npos);
  EXPECT_NE(json_text.find("\"consensus_offset_m\":null"), std::string::npos);

  // Recorded map still present and deskewed.
  const auto recorded =
      expectedDeskewedMapPoints(run.dataset, run.dataset.recorded_mount.x_m);
  ASSERT_FALSE(recorded.empty());
  EXPECT_NE(json_text.find('[' + jsonNumber(recorded.front().x) + ',' +
                           jsonNumber(recorded.front().y) + ']'),
            std::string::npos);

  std::filesystem::remove_all(output);
}

// Build ≥3 odom-deskewable scans + accepted Odom ICP edges with known pure
// translation so chained world points are deterministic (identity root, T).
CalibrationRun icpMapFixture(const std::filesystem::path& output) {
  CalibrationRun run = fixture(output);
  run.center_samples.clear();
  run.dataset = {};
  run.dataset.recorded_mount = {0.260, 0.0, 0.0, kPi};

  constexpr double omega = 0.40;
  constexpr double duration_s = 0.10;
  constexpr std::int64_t t0 = 1'000'000'000LL;
  const std::int64_t dt =
      static_cast<std::int64_t>(std::llround(duration_s * 1e9));

  for (std::uint64_t scan_id = 1; scan_id <= 3; ++scan_id) {
    const std::int64_t start = t0 + static_cast<std::int64_t>(scan_id - 1) * 2 * dt;
    const std::int64_t end = start + dt;
    ScanValue raw;
    raw.id = scan_id;
    raw.stamp_ns = start;
    raw.angle_min = 0.0F;
    raw.angle_increment = 0.5F;
    raw.time_increment = static_cast<float>(duration_s / 2.0);
    raw.range_min = 0.1F;
    raw.range_max = 20.0F;
    // Single hit along +x in lidar frame after deskew → easy expected coords.
    raw.ranges = {2.0F, 2.5F, 3.0F};
    run.dataset.raw_scans.push_back(raw);
    for (const std::int64_t stamp : {start, (start + end) / 2, end}) {
      const double yaw =
          omega * static_cast<double>(stamp - t0) / 1'000'000'000.0;
      run.dataset.odom_poses.push_back({stamp, {0.0, 0.0, yaw}});
    }
  }

  // Pure +1 m translation of source into target: pose[target]=pose[source]*T^{-1}.
  // Root scan 1 at identity → pose[2]=(-1,0,0), pose[3]=(-2,0,0).
  for (std::uint64_t source = 1; source <= 2; ++source) {
    CenterSample edge;
    edge.method = DeskewMethod::kOdom;
    edge.source_scan_id = source;
    edge.target_scan_id = source + 1;
    edge.accepted = true;
    edge.icp.source_to_target = {1.0, 0.0, 0.0};
    edge.icp.converged = true;
    edge.icp.trimmed_rmse_m = 0.01;
    edge.icp.overlap_ratio = 0.9;
    run.center_samples.push_back(edge);
  }
  // Non-Odom accepted edge must not participate in the ICP map.
  CenterSample imu_edge;
  imu_edge.method = DeskewMethod::kImu;
  imu_edge.source_scan_id = 1;
  imu_edge.target_scan_id = 3;
  imu_edge.accepted = true;
  imu_edge.icp.source_to_target = {99.0, 99.0, 0.0};
  run.center_samples.push_back(imu_edge);

  for (std::size_t i = 0; i < run.methods.size(); ++i) {
    run.methods[i].method = static_cast<DeskewMethod>(i);
    run.methods[i].reliable = true;
  }
  return run;
}

std::vector<Point2> expectedIcpChainedMapPoints(const CalibrationRun& run) {
  std::vector<Point2> points;
  const auto& dataset = run.dataset;
  if (dataset.odom_poses.empty() || dataset.raw_scans.empty()) return points;

  const auto retention = dataset.odom_poses.back().stamp_ns -
                         dataset.odom_poses.front().stamp_ns + 1'000'000'000LL;
  TimedPoseBuffer odom(std::max<std::int64_t>(retention, 1'000'000'000LL),
                       1'000'000'000LL);
  for (const auto& pose : dataset.odom_poses) odom.push(pose);
  const StaticLidarMount mount{dataset.recorded_mount.x_m, 0.0, 0.0, kPi};

  std::map<std::uint64_t, std::vector<Point2>> local_by_id;
  for (const auto& scan : dataset.raw_scans) {
    const auto deskewed = deskewWithOdom(scan, odom, mount, 12.0);
    if (!deskewed || deskewed->points.empty()) continue;
    local_by_id[scan.id] = deskewed->points;
  }

  // Mirror production chain: root=1 identity, T=(1,0,0) along 1→2→3.
  const Pose2 t{1.0, 0.0, 0.0};
  std::map<std::uint64_t, Pose2> poses;
  poses[1] = {};
  poses[2] = poses[1] * t.inverse();
  poses[3] = poses[2] * t.inverse();

  for (const auto& [scan_id, local_points] : local_by_id) {
    const auto pose_it = poses.find(scan_id);
    if (pose_it == poses.end()) continue;
    const std::size_t ray_stride =
        std::max<std::size_t>(1, local_points.size() / 180);
    for (std::size_t i = 0; i < local_points.size(); i += ray_stride) {
      points.push_back(transform(pose_it->second, local_points[i]));
    }
  }
  return points;
}

TEST(CalibrationReport, IcpMapViewChainsAcceptedOdomPairs) {
  const auto output =
      std::filesystem::temp_directory_path() / "lidar-calibration-icp-map";
  std::filesystem::remove_all(output);
  auto run = icpMapFixture(output);
  ASSERT_NO_THROW(writeCalibrationReport(run));

  std::ifstream json(output / "calibration.json");
  const std::string json_text((std::istreambuf_iterator<char>(json)), {});
  const auto maps = extractMapsSection(json_text);
  ASSERT_FALSE(maps.empty());
  EXPECT_NE(maps.find("\"icp\":["), std::string::npos);
  EXPECT_EQ(maps.find("\"icp\":[]"), std::string::npos);

  const auto expected = expectedIcpChainedMapPoints(run);
  ASSERT_FALSE(expected.empty());
  for (const auto& point : expected) {
    EXPECT_NE(maps.find('[' + jsonNumber(point.x) + ',' + jsonNumber(point.y) + ']'),
              std::string::npos)
        << "missing ICP map point " << point.x << "," << point.y;
  }
  // Meta for diagnostics.
  EXPECT_NE(json_text.find("\"icp_meta\""), std::string::npos);
  EXPECT_NE(json_text.find("\"scan_count\":3"), std::string::npos);
  EXPECT_NE(json_text.find("\"edge_count\":2"), std::string::npos);
  EXPECT_NE(json_text.find("\"root_scan_id\":1"), std::string::npos);

  std::ifstream html(output / "report.html");
  const std::string html_text((std::istreambuf_iterator<char>(html)), {});
  EXPECT_NE(html_text.find("icp-map"), std::string::npos);
  EXPECT_NE(html_text.find("ICP map view"), std::string::npos);
  EXPECT_NE(html_text.find("plotMap('icp-map','icp')"), std::string::npos);

  std::filesystem::remove_all(output);
}

TEST(CalibrationReport, IcpMapViewEmptyWithoutAcceptedOdomPairs) {
  const auto output =
      std::filesystem::temp_directory_path() / "lidar-calibration-icp-map-empty";
  std::filesystem::remove_all(output);
  // Deskewable scans exist, but no accepted Odom edges between them.
  auto run = rotatingMapFixture(output, 0.245, true);
  run.center_samples.clear();
  CenterSample rejected;
  rejected.method = DeskewMethod::kOdom;
  rejected.source_scan_id = 7;
  rejected.target_scan_id = 7;
  rejected.accepted = false;
  run.center_samples.push_back(rejected);
  CenterSample imu_only;
  imu_only.method = DeskewMethod::kImu;
  imu_only.source_scan_id = 7;
  imu_only.target_scan_id = 7;
  imu_only.accepted = true;
  imu_only.icp.source_to_target = {1.0, 0.0, 0.0};
  run.center_samples.push_back(imu_only);

  ASSERT_NO_THROW(writeCalibrationReport(run));

  std::ifstream json(output / "calibration.json");
  const std::string json_text((std::istreambuf_iterator<char>(json)), {});
  EXPECT_NE(json_text.find("\"icp\":[]"), std::string::npos);
  EXPECT_EQ(json_text.find("\"icp_meta\""), std::string::npos);

  std::filesystem::remove_all(output);
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
  EXPECT_NE(centers_text.find("\"\"comma\"\""), std::string::npos);
  std::ifstream sharpness(output / "sharpness.csv");
  const std::string sharpness_text((std::istreambuf_iterator<char>(sharpness)), {});
  EXPECT_NE(sharpness_text.find("0.180"), std::string::npos);
  std::ifstream html(output / "report.html");
  const std::string html_text((std::istreambuf_iterator<char>(html)), {});
  EXPECT_NE(html_text.find("Existing /scan"), std::string::npos);
  EXPECT_NE(html_text.find("Raw center scatter"), std::string::npos);
  EXPECT_NE(html_text.find("rejected samples are outlined"), std::string::npos);
  EXPECT_EQ(html_text.find("src=\"http"), std::string::npos);
  EXPECT_EQ(html_text.find("href=\"http"), std::string::npos);
  if (std::getenv("TASK5_REPORT_FIXTURE_DIR") == nullptr) std::filesystem::remove_all(output);
}

TEST(CalibrationReport, WritesParseableNullsForInconclusiveScientificValues) {
  const auto output = std::getenv("TASK5_REPORT_INCONCLUSIVE_FIXTURE_DIR") != nullptr
    ? std::filesystem::path(std::getenv("TASK5_REPORT_INCONCLUSIVE_FIXTURE_DIR"))
    : std::filesystem::temp_directory_path() / "lidar-calibration-report-inconclusive";
  if (std::getenv("TASK5_REPORT_INCONCLUSIVE_FIXTURE_DIR") == nullptr) {
    std::filesystem::remove_all(output);
  }
  ASSERT_NO_THROW(writeCalibrationReport(fixture(output, true)));
  std::ifstream json(output / "calibration.json");
  const std::string text((std::istreambuf_iterator<char>(json)), {});
  EXPECT_EQ(text.find("nan"), std::string::npos);
  EXPECT_EQ(text.find("inf"), std::string::npos);
  EXPECT_NE(text.find(":null"), std::string::npos);
  if (std::getenv("TASK5_REPORT_INCONCLUSIVE_FIXTURE_DIR") == nullptr) {
    std::filesystem::remove_all(output);
  }
}

TEST(CalibrationReport, RollsBackWholeSetWhenLatePublicationFails) {
  const auto output = std::filesystem::temp_directory_path() / "lidar-calibration-report-rollback";
  std::filesystem::remove_all(output);
  ASSERT_NO_THROW(writeCalibrationReport(fixture(output)));
  const std::string old_json = [] (const auto& path) {
    std::ifstream input(path);
    return std::string((std::istreambuf_iterator<char>(input)), {});
  }(output / "calibration.json");
  auto replacement = fixture(output);
  replacement.config.overwrite = true;
  setenv("TASK5_REPORT_FAIL_PUBLISH_INDEX", "2", 1);
  EXPECT_THROW(writeCalibrationReport(replacement), std::runtime_error);
  unsetenv("TASK5_REPORT_FAIL_PUBLISH_INDEX");
  std::ifstream restored(output / "calibration.json");
  EXPECT_EQ(std::string((std::istreambuf_iterator<char>(restored)), {}), old_json);
  for (const auto& name : {"calibration.json", "centers.csv", "sharpness.csv", "report.html"}) {
    EXPECT_TRUE(std::filesystem::is_regular_file(output / name));
  }
  for (const auto& entry : std::filesystem::directory_iterator(output)) {
    EXPECT_EQ(entry.path().filename().string().find(".tmp."), std::string::npos);
    EXPECT_EQ(entry.path().filename().string().find(".bak."), std::string::npos);
  }
  std::filesystem::remove_all(output);
}

TEST(CalibrationReport, CleansAllTempsWhenLateWriteFails) {
  const auto output = std::filesystem::temp_directory_path() / "lidar-calibration-report-write-failure";
  std::filesystem::remove_all(output);
  setenv("TASK5_REPORT_FAIL_WRITE_INDEX", "3", 1);
  EXPECT_THROW(writeCalibrationReport(fixture(output)), std::runtime_error);
  unsetenv("TASK5_REPORT_FAIL_WRITE_INDEX");
  if (std::filesystem::exists(output)) {
    for (const auto& entry : std::filesystem::directory_iterator(output)) {
      EXPECT_EQ(entry.path().filename().string().find(".tmp."), std::string::npos);
      EXPECT_EQ(entry.path().filename().string().find(".bak."), std::string::npos);
      EXPECT_FALSE(std::filesystem::is_regular_file(entry.path()));
    }
  }
  std::filesystem::remove_all(output);
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
