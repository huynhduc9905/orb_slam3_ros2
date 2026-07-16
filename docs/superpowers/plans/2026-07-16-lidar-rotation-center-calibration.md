# Lidar Rotation-Center Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a read-only offline tool that estimates the lidar-to-rotation-center vector, compares odom deskew, IMU deskew, and existing `/scan` undistortion, and produces raw measurements plus a visual report.

**Architecture:** Add an isolated calibration library and executable to `orb_lidar_mapper`. Read MCAP directly with `rosbag2_cpp`, reuse `Pose2` and `TimedPoseBuffer`, use PCL constrained 2D ICP, robustly estimate rotation centers, cross-check them with a one-dimensional map-sharpness sweep, and write JSON/CSV/self-contained HTML without modifying TF or replaying the bag.

**Tech Stack:** C++17, ROS 2 Kilted, `rosbag2_cpp`, `rclcpp::Serialization`, Eigen, PCL 1.15.1 registration/kdtree, GoogleTest, CMake/ament, Playwright.

## Global Constraints

- Open `/home/duc/robot/bag/inplace-rotate` read-only.
- Require `/scan_origin`, `/scan`, `/odom`, `/imu`, and `/tf_static`.
- Treat `base_scan.y = 0` and `base_scan.yaw = pi` as exact; estimate only `base_scan.x`.
- Always expose raw center X/Y, CI, pair counts, residual, and overlap per method.
- Compare exactly: `/scan_origin` deskewed by `/odom`, `/scan_origin` deskewed by `/imu`, and existing `/scan` without invented ray timing.
- Use `/odom`, not `/odom_wheel`, for the requested odometry comparison.
- Default motion gate: `0.15 <= |omega| <= 0.45 rad/s`, `|v| < 0.02 m/s`.
- Pair separation: 10-30 degrees; overlap >= 0.40; trimmed RMSE <= 0.05 m; yaw disagreement <= 2 degrees.
- Accept finite ranges within message bounds and `0.15 <= range <= 12.0 m`; ignore NaN/infinity.
- Reliable method: >= 40 accepted pairs, >= 6/8 yaw sectors, CI half-width <= 0.015 m, and `|median(center_y)| <= 0.020 m`.
- Reliable method median spread must be <= 0.015 m.
- Sharpness sweep: 0.180-0.340 m at 0.002 m, then 0.00025 m refinement. A reliable minimum is unique, 3% better than candidates +/-0.020 m, and within 0.010 m of ICP consensus.
- `CONSISTENT` requires `|consensus-0.260| <= max(0.010 m, consensus CI half-width)`; `LIKELY_OFFSET_ERROR` requires the strict opposite inequality after method/sharpness agreement; all other scientific outcomes are `INCONCLUSIVE`.
- Exit statuses: `0=CONSISTENT`, `1=operational failure`, `2=LIKELY_OFFSET_ERROR`, `3=INCONCLUSIVE`.
- Never edit TF, URDF, source configuration, or the bag automatically.
- Do not modify/stage dormant React/Foxglove changes or `semantic-review/`.
- Use strict red-green TDD and a fresh review after every task.

---

### Task 1: Data Model And Read-Only Bag Ingestion

**Files:**
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/calibration_types.hpp`
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/rotation_bag_reader.hpp`
- Create: `orb_lidar_mapper/src/rotation_bag_reader.cpp`
- Create: `orb_lidar_mapper/test/rotation_bag_reader_test.cpp`
- Modify: `orb_lidar_mapper/CMakeLists.txt`
- Modify: `orb_lidar_mapper/package.xml`

**Interfaces:**

```cpp
enum class DeskewMethod { kOdom, kImu, kExistingScan };
enum class ResultClass { kConsistent, kLikelyOffsetError, kInconclusive };
inline constexpr double kPi = 3.14159265358979323846;
struct TimedTwist2 { std::int64_t stamp_ns; Twist2 twist; };
struct TimedYawRate { std::int64_t stamp_ns; double omega_rad_s; };
struct StaticLidarMount { double x_m, y_m, z_m, yaw_rad; };
struct RotationDataset {
  std::vector<ScanValue> raw_scans, undistorted_scans;
  std::vector<TimedPose2> odom_poses;
  std::vector<TimedTwist2> odom_twists;
  std::vector<TimedYawRate> imu_yaw_rates;
  StaticLidarMount recorded_mount;
};
struct MotionInterval { std::int64_t start_ns, end_ns; };
class RotationBagReader {
 public:
  static RotationDataset read(const std::filesystem::path& bag_path);
};
std::vector<MotionInterval> selectStableRotationIntervals(
  const RotationDataset&, double min_abs_omega, double max_abs_omega,
  double max_abs_linear_speed, std::int64_t minimum_duration_ns);
```

- [ ] **Step 1: Write failing real-MCAP reader tests**

Use `rosbag2_cpp::Writer` to create a temporary MCAP with one message per required topic and recorded `base_link -> base_scan`.

```cpp
TEST(RotationBagReader, ReadsTopicsAndRecordedMount) {
  const auto bag = writeCalibrationBag(true);
  const auto data = RotationBagReader::read(bag.path());
  ASSERT_EQ(data.raw_scans.size(), 1U);
  ASSERT_EQ(data.undistorted_scans.size(), 1U);
  EXPECT_NEAR(data.recorded_mount.x_m, 0.26, 1e-12);
  EXPECT_NEAR(data.recorded_mount.yaw_rad, kPi, 1e-9);
}
TEST(RotationBagReader, MissingImuFailsClosed) {
  try {
    (void)RotationBagReader::read(writeBagWithoutImu());
    FAIL() << "expected missing /imu to fail";
  } catch (const std::runtime_error& error) {
    EXPECT_THAT(error.what(), testing::HasSubstr("/imu"));
  }
}
TEST(MotionSelector, RejectsTranslationAndKeepsStableRotation) {
  const auto intervals = selectStableRotationIntervals(
    datasetWithMotionSamples(), 0.15, 0.45, 0.02, 1'000'000'000LL);
  ASSERT_EQ(intervals.size(), 1U);
}
```

- [ ] **Step 2: Verify RED**

```bash
colcon build --packages-select orb_lidar_mapper --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select orb_lidar_mapper --ctest-args \
  -R rotation_bag_reader_test --output-on-failure
```

Expected: compile failure because the reader/types do not exist.

- [ ] **Step 3: Implement minimal ingestion**

Add `rosbag2_cpp`, `tf2_msgs`, `pcl_conversions`, and PCL 1.15 CMake/package dependencies. Create one `${PROJECT_NAME}_calibration` library initially containing `rotation_bag_reader.cpp`; later tasks append their sources to this same target. Use `rosbag2_cpp::Reader` and typed `rclcpp::Serialization`. Validate required topics, monotonic stamps, finite data, nonempty scans, and exact recorded edge. Reject `|y| > 1e-6` or wrapped `|yaw-pi| > 1e-6`; preserve `/scan` zero timing instead of synthesizing it.

```cmake
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.15 REQUIRED COMPONENTS common registration kdtree filters)
add_library(${PROJECT_NAME}_calibration src/rotation_bag_reader.cpp)
target_compile_features(${PROJECT_NAME}_calibration PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_calibration
  ${PROJECT_NAME}_core ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_calibration
  rclcpp rosbag2_cpp sensor_msgs nav_msgs tf2_msgs pcl_conversions)
```

- [ ] **Step 4: Verify GREEN and regressions**

Run the focused test, all `orb_lidar_mapper` tests, and `colcon test-result --verbose`. Expected: zero failures.

- [ ] **Step 5: Commit**

```bash
git add orb_lidar_mapper/CMakeLists.txt orb_lidar_mapper/package.xml \
  orb_lidar_mapper/include/orb_lidar_mapper/calibration_types.hpp \
  orb_lidar_mapper/include/orb_lidar_mapper/rotation_bag_reader.hpp \
  orb_lidar_mapper/src/rotation_bag_reader.cpp \
  orb_lidar_mapper/test/rotation_bag_reader_test.cpp
git commit -m "feat: read lidar calibration bags offline"
```

### Task 2: Three Comparable Scan Preprocessing Paths

**Files:**
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/calibration_deskew.hpp`
- Create: `orb_lidar_mapper/src/calibration_deskew.cpp`
- Create: `orb_lidar_mapper/test/calibration_deskew_test.cpp`
- Modify: `orb_lidar_mapper/CMakeLists.txt`

**Interfaces:**

```cpp
struct DeskewedScan {
  std::uint64_t scan_id;
  std::int64_t reference_stamp_ns;
  DeskewMethod method;
  std::vector<Point2> points;
};
struct ScanAssociation {
  std::uint64_t raw_scan_id, undistorted_scan_id;
  std::int64_t timestamp_delay_ns;
};
std::optional<DeskewedScan> deskewWithOdom(
  const ScanValue&, const TimedPoseBuffer&, const StaticLidarMount&, double range_cap_m);
std::optional<DeskewedScan> deskewWithImu(
  const ScanValue&, const std::vector<TimedYawRate>&, double candidate_offset_m,
  double fixed_yaw_rad, double range_cap_m);
std::vector<ScanAssociation> associateUndistortedScans(
  const std::vector<ScanValue>& raw, const std::vector<ScanValue>& undistorted,
  std::int64_t maximum_delay_deviation_ns);
DeskewedScan adaptUndistortedScan(
  const ScanValue&, const ScanAssociation&, double range_cap_m);
```

- [ ] **Step 1: Write failing synthetic deskew tests**

Generate a 98.558 ms scan of static asymmetric world points while the base rotates at 0.30 rad/s around a lidar offset of 0.26 m.

```cpp
TEST(CalibrationDeskew, OdomRecoversMidpointCloud) {
  auto f = rotatingScanFixture(0.26, 0.30, 0.098558);
  auto result = deskewWithOdom(f.raw, f.odom, f.mount, 12.0);
  ASSERT_TRUE(result);
  expectCloudNear(result->points, f.midpoint_points, 0.002);
}
TEST(CalibrationDeskew, ImuModelsLidarArc) {
  auto f = rotatingScanFixture(0.26, 0.30, 0.098558);
  auto result = deskewWithImu(f.raw, f.imu, 0.26, kPi, 12.0);
  ASSERT_TRUE(result);
  expectCloudNear(result->points, f.midpoint_points, 0.002);
}
TEST(CalibrationDeskew, MissingCoverageAndInvalidRangesFailClosed) {
  EXPECT_FALSE(deskewWithOdom(rawScan(), emptyPoseBuffer(), mount(), 12.0));
  EXPECT_EQ(finitePoints({NAN, INFINITY, 0.10F, 2.0F, 13.0F}).size(), 1U);
}
TEST(CalibrationDeskew, AssociatesExistingScanByOrderAndMedianDelay) {
  auto matches = associateUndistortedScans(rawAt({0,100,200}),
                                            filteredAt({90,190,290}), 2);
  ASSERT_EQ(matches.size(), 3U);
  EXPECT_EQ(matches[1].timestamp_delay_ns, 90);
}
```

- [ ] **Step 2: Verify RED** using `calibration_deskew_test` through CTest.

- [ ] **Step 3: Implement midpoint deskew**

For every ray use:

```cpp
Pose2 base_to_lidar{mount.x_m, mount.y_m, mount.yaw_rad};
Pose2 ray_to_mid = (mid_base * base_to_lidar).inverse() *
                   (ray_base * base_to_lidar);
Point2 corrected = transform(ray_to_mid, raw_point);
```

Interpolate `/odom` with `TimedPoseBuffer`. Trapezoidally integrate finite IMU Z rates, reject gaps over 20 ms, and create pure-rotation base poses before applying the candidate offset. For `/scan`, only filter/convert points; associate by order and robust median delay, rejecting ambiguous matches.

- [ ] **Step 4: Verify GREEN**, all mapper tests, and five deterministic repetitions.

- [ ] **Step 5: Commit** as `feat: add comparable lidar deskew paths` with only Task 2 files.

### Task 3: Constrained ICP And Raw Center Samples

**Files:**
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/planar_icp.hpp`
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/rotation_center_estimator.hpp`
- Create: `orb_lidar_mapper/src/planar_icp.cpp`
- Create: `orb_lidar_mapper/src/rotation_center_estimator.cpp`
- Create: `orb_lidar_mapper/test/planar_icp_test.cpp`
- Create: `orb_lidar_mapper/test/rotation_center_estimator_test.cpp`
- Modify: `orb_lidar_mapper/CMakeLists.txt`

**Interfaces:**

```cpp
struct IcpConfig {
  double max_correspondence_m{0.15};
  int max_iterations{60};
  double minimum_overlap{0.40};
  double maximum_trimmed_rmse_m{0.05};
  double maximum_yaw_error_rad{2.0 * kPi / 180.0};
};
struct IcpResult {
  bool converged;
  Pose2 source_to_target;
  double overlap_ratio, trimmed_rmse_m;
  std::size_t correspondence_count;
  std::string rejection_reason;
};
class PlanarIcp {
 public:
  explicit PlanarIcp(IcpConfig);
  IcpResult align(const std::vector<Point2>& source,
                  const std::vector<Point2>& target,
                  double expected_yaw_rad) const;
};
struct ScanPair {
  std::size_t source_index, target_index, yaw_sector;
  double odom_yaw_delta_rad;
};
struct CenterSample {
  DeskewMethod method;
  std::uint64_t source_scan_id, target_scan_id;
  std::size_t yaw_sector;
  bool accepted;
  Point2 center;
  IcpResult icp;
  std::string rejection_reason;
};
std::vector<ScanPair> selectCalibrationPairs(
  const std::vector<DeskewedScan>&, const TimedPoseBuffer&,
  const std::vector<MotionInterval>&, double min_yaw, double max_yaw);
std::optional<Point2> centerFromTransform(const Pose2& source_to_target);
CenterSample estimateRotationCenter(
  DeskewMethod, const ScanPair&, const DeskewedScan&, const DeskewedScan&,
  const PlanarIcp&, double max_center_x, double max_abs_center_y);
```

- [ ] **Step 1: Write failing sign and ICP tests**

```cpp
TEST(RotationCenter, LocksSourceToTargetSign) {
  Pose2 motion = rotationAboutCenter(0.40, {0.26, 0.0});
  auto center = centerFromTransform(motion);
  ASSERT_TRUE(center);
  EXPECT_NEAR(center->x, +0.26, 1e-9);
}
TEST(PlanarIcp, RecoversAsymmetricRoomMotion) {
  auto source = asymmetricRoomCloud();
  auto truth = rotationAboutCenter(0.35, {0.26, 0.0});
  auto result = PlanarIcp(IcpConfig{}).align(source,
                                              transformCloud(source, truth), 0.35);
  ASSERT_TRUE(result.converged);
  auto center = centerFromTransform(result.source_to_target);
  ASSERT_TRUE(center);
  EXPECT_NEAR(center->x, 0.26, 0.005);
}
TEST(PlanarIcp, RejectsBlankCircularAndWrongYawPairs) {
  EXPECT_EQ(rejection(blankCloud()), "insufficient_points");
  EXPECT_EQ(rejection(circleCloud()), "degenerate_geometry");
  EXPECT_EQ(rejection(wrongYawCloud()), "yaw_disagreement");
}
```

- [ ] **Step 2: Verify RED** with both focused test executables.

- [ ] **Step 3: Implement PCL planar ICP**

```cpp
using Point = pcl::PointXYZ;
using Estimator = pcl::registration::TransformationEstimation2D<Point, Point, float>;
pcl::IterativeClosestPoint<Point, Point> icp;
icp.setTransformationEstimation(std::make_shared<Estimator>());
```

Recompute deterministic correspondences, trim the largest 20%, calculate overlap, and reject covariance condition number above `1e4`. Convert the final transform to `Pose2`. Solve `c = (I-R)^-1 t` with Eigen and reject determinant below `1e-4`; never alter the sign based on the recorded TF.

- [ ] **Step 4: Verify GREEN**, repeat focused tests five times, then run all mapper tests.

- [ ] **Step 5: Commit** as `feat: estimate lidar rotation centers with planar ICP` with only Task 3 files.

### Task 4: Robust Estimates, Sharpness, And Classification

**Files:**
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/calibration_analysis.hpp`
- Create: `orb_lidar_mapper/src/calibration_analysis.cpp`
- Create: `orb_lidar_mapper/test/calibration_analysis_test.cpp`
- Modify: `orb_lidar_mapper/CMakeLists.txt`

**Interfaces:**

```cpp
struct ConfidenceInterval { double low_m, high_m; };
struct MethodEstimate {
  DeskewMethod method;
  bool reliable;
  double center_x_m, center_y_m, forward_offset_m;
  ConfidenceInterval confidence_95_m;
  std::size_t accepted_pairs, attempted_pairs, covered_yaw_sectors;
  double median_rmse_m, median_overlap;
  std::map<std::string, std::size_t> rejection_counts;
};
struct SharpnessPoint { double offset_m, score; };
struct SharpnessResult {
  bool reliable;
  double best_offset_m;
  std::vector<SharpnessPoint> coarse, refined;
  std::string rejection_reason;
};
struct AggregateResult {
  ResultClass classification;
  double consensus_offset_m;
  ConfidenceInterval confidence_95_m;
  std::string reason;
};
MethodEstimate robustMethodEstimate(DeskewMethod,
  const std::vector<CenterSample>&,
  std::uint64_t seed = 0x4f52424c49444152ULL);
SharpnessResult evaluateMapSharpness(const RotationDataset&,
  const std::vector<DeskewedScan>&, const TimedPoseBuffer&, double consensus_hint);
AggregateResult classifyCalibration(const std::vector<MethodEstimate>&,
  const SharpnessResult&, double recorded_offset_m);
```

- [ ] **Step 1: Write failing robust/classification tests**

```cpp
TEST(CalibrationAnalysis, RejectsOutliersButPreservesRawRows) {
  auto samples = centerSamplesAround(0.26, 0.002, 80);
  samples.push_back(acceptedCenter(0.90, 0.20));
  auto result = robustMethodEstimate(DeskewMethod::kOdom, samples);
  EXPECT_TRUE(result.reliable);
  EXPECT_NEAR(result.forward_offset_m, 0.26, 0.003);
}
TEST(CalibrationAnalysis, SectorDeficiencyIsUnreliable) {
  EXPECT_FALSE(robustMethodEstimate(DeskewMethod::kOdom,
    centerSamplesInOneSector(0.26, 100)).reliable);
}
TEST(CalibrationAnalysis, ClassifiesConsistentErrorAndDisagreement) {
  EXPECT_EQ(classifyFixture({0.257,0.260,0.262}, 0.260), ResultClass::kConsistent);
  EXPECT_EQ(classifyFixture({0.230,0.232,0.234}, 0.232), ResultClass::kLikelyOffsetError);
  EXPECT_EQ(classifyFixture({0.230,0.260,0.290}, 0.260), ResultClass::kInconclusive);
}
```

- [ ] **Step 2: Verify RED** with `calibration_analysis_test`.

- [ ] **Step 3: Implement deterministic analysis**

Use medians, MAD weighting, and 2,000 `std::mt19937_64` bootstrap samples. Preserve every rejected row and rejection count. For sharpness, transform candidate `Pose2{x,0,pi}` scans through `/odom`, split nonadjacent yaw sectors, query a PCL KD-tree across subsets, trim 20%, and run the exact coarse/refined grids. Classification requires at least two reliable methods, <= 0.015 m median spread, reliable sharpness, and the exact recorded-offset equation from the spec.

- [ ] **Step 4: Verify GREEN**, five deterministic repetitions, then all mapper tests.

- [ ] **Step 5: Commit** as `feat: cross-check lidar offset with map sharpness`.

### Task 5: Pipeline, CLI, Outputs, And Browser Report

**Files:**
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/calibration_pipeline.hpp`
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/calibration_report.hpp`
- Create: `orb_lidar_mapper/src/calibration_pipeline.cpp`
- Create: `orb_lidar_mapper/src/calibration_report.cpp`
- Create: `orb_lidar_mapper/src/lidar_rotation_center_check.cpp`
- Create: `orb_lidar_mapper/test/calibration_pipeline_test.cpp`
- Create: `orb_lidar_mapper/test/calibration_report_test.cpp`
- Create: `orb_slam_dashboard/web/e2e/lidar-calibration-report.spec.ts`
- Create: `orb_slam_dashboard/web/playwright.calibration.config.ts`
- Modify: `orb_lidar_mapper/CMakeLists.txt`

**Interfaces:**

```cpp
struct CalibrationConfig {
  std::filesystem::path bag_path, output_dir;
  bool overwrite{false};
  double min_abs_omega{0.15}, max_abs_omega{0.45};
  double max_abs_linear_speed{0.02}, range_cap_m{12.0};
  IcpConfig icp;
};
struct CalibrationRun {
  CalibrationConfig config;
  RotationDataset dataset;
  std::vector<CenterSample> center_samples;
  std::array<MethodEstimate, 3> methods;
  SharpnessResult sharpness;
  AggregateResult aggregate;
};
CalibrationRun runCalibration(const CalibrationConfig&);
void writeCalibrationReport(const CalibrationRun&);
int resultExitCode(ResultClass);
```

- [ ] **Step 1: Write failing pipeline/report tests**

```cpp
TEST(CalibrationPipeline, RunsAllMethodsAndIteratesImuOffset) {
  auto run = runSyntheticCalibration(0.245);
  for (const auto& method : run.methods) {
    EXPECT_TRUE(method.reliable);
    EXPECT_NEAR(method.forward_offset_m, 0.245, 0.005);
  }
  EXPECT_EQ(run.aggregate.classification, ResultClass::kLikelyOffsetError);
}
TEST(CalibrationReport, WritesSelfContainedOutputsAtomically) {
  auto out = temporaryOutputDirectory();
  writeCalibrationReport(calibrationRunFixture(out));
  EXPECT_TRUE(validJson(out / "calibration.json"));
  EXPECT_TRUE(csvHasRejectedRows(out / "centers.csv"));
  EXPECT_TRUE(csvHasSharpnessGrid(out / "sharpness.csv"));
  auto html = read(out / "report.html");
  EXPECT_THAT(html, HasSubstr("Existing /scan"));
  EXPECT_THAT(html, Not(HasSubstr("src=\"http")));
}
```

- [ ] **Step 2: Verify RED** with pipeline and report tests.

- [ ] **Step 3: Implement orchestration and CLI**

Run bag read, interval selection, all deskew paths, common pair schedule, ICP, robust method estimates, IMU offset iteration (start 0.260 m; stop below 0.5 mm; max 8 iterations), sharpness, and classification. CLI options are exactly `--bag`, `--output`, `--overwrite`, `--range-cap-m`, `--min-omega`, `--max-omega`, `--max-linear-speed`, and `--help`. Unknown/missing options return 1.

- [ ] **Step 4: Implement atomic JSON/CSV/HTML output**

Write sibling temporary files and rename only after all content closes successfully. Embed CSS, JavaScript, JSON, center rows, sharpness rows, and downsampled maps in `report.html`. Canvas plots have fixed responsive dimensions. The report makes no network requests and exposes no controls or write API.

- [ ] **Step 5: Install executable and verify GREEN**

```cmake
add_executable(lidar_rotation_center_check src/lidar_rotation_center_check.cpp)
target_link_libraries(lidar_rotation_center_check ${PROJECT_NAME}_calibration)
install(TARGETS ${PROJECT_NAME}_calibration lidar_rotation_center_check
  DESTINATION lib/${PROJECT_NAME})
```

Run focused tests, all mapper tests, and `ros2 run orb_lidar_mapper lidar_rotation_center_check --help`.

- [ ] **Step 6: Run Playwright at 1440x900, 768x1024, and 390x844**

Assert nonblank center scatter, sharpness curve, recorded/estimated maps, all method rows, no clipping, no console errors, and no network requests after initial HTML.

```bash
cd orb_slam_dashboard/web
npx playwright test --config playwright.calibration.config.ts
```

- [ ] **Step 7: Commit** as `feat: report lidar rotation-center calibration`, staging only Task 5 files.

### Task 6: Real Bag Acceptance, Documentation, And Review

**Files:**
- Modify: `README.md`
- Modify: `handoff-kiro.md`
- Create locally only: `artifacts/inplace-rotate-calibration-20260716/`
- Verify: `docs/superpowers/specs/2026-07-16-lidar-rotation-center-calibration-design.md`

**Interfaces:**
- Consumes: installed `lidar_rotation_center_check` and immutable bag.
- Produces: one fresh artifact and documentation containing raw three-method estimates.

- [ ] **Step 1: Run full build and tests**

```bash
MAKEFLAGS=-j4 CMAKE_BUILD_PARALLEL_LEVEL=4 COLCON_DEFAULTS_FILE=/dev/null \
  colcon build --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
git diff --check
git submodule status
```

Expected: zero failures and clean ORB pin `bd9c608`.

- [ ] **Step 2: Run one fresh real-bag calibration**

```bash
ros2 run orb_lidar_mapper lidar_rotation_center_check \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output "$PWD/artifacts/inplace-rotate-calibration-20260716"
```

Accept scientific exit 0, 2, or 3; exit 1 blocks. Require nonempty `calibration.json`, `centers.csv`, `sharpness.csv`, and `report.html`. Never delete/reuse an earlier output directory.

- [ ] **Step 3: Record raw results without threshold tuning**

Record recorded center; odom, IMU, and `/scan` center X/Y/CI/pairs/RMSE/overlap; sharpness minimum; consensus; classification; and every rejection counter. Preserve `INCONCLUSIVE` honestly and diagnose from counters rather than relaxing thresholds.

- [ ] **Step 4: Verify real report visually and through Playwright**

Check all three viewports. Confirm plot signs, labels, method colors, map framing, score curve, warnings, and numeric values match JSON exactly.

- [ ] **Step 5: Update operator documentation**

Document command, bag facts, output path, statuses, raw estimates, and interpretation. State that no TF/URDF changed. A suggested offset is a candidate requiring user approval.

- [ ] **Step 6: Run Task 6 and whole-change reviews**

Create scoped review packages containing only intended Task 1-6 files. Fix every Critical/Important finding and rerun affected tests. Invoke `superpowers:requesting-code-review` for the final whole-change review. Keep artifacts, dormant frontend changes, and `semantic-review/` unstaged.

- [ ] **Step 7: Commit reviewed documentation**

```bash
git add README.md handoff-kiro.md
git commit -m "docs: record lidar rotation-center estimate"
```

## Completion Gate

- [ ] Full build/tests pass with zero failures.
- [ ] Synthetic sign, distortion, degeneracy, and classification tests pass.
- [ ] ICP is constrained to SE(2) and deterministic across repetitions.
- [ ] Odom, IMU, and `/scan` paths expose separate raw estimates.
- [ ] NaN/infinity rays are ignored in every path.
- [ ] Real immutable bag produces complete JSON/CSV/HTML.
- [ ] Predeclared thresholds are not tuned after seeing the result.
- [ ] Playwright passes desktop, tablet, and mobile.
- [ ] No TF, URDF, source config, bag, dormant frontend, or semantic-review files are modified/staged.
- [ ] Fresh task reviews and final review have no Critical/Important findings.
