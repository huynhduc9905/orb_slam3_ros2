# Stereo ORB-SLAM Rotation-Center Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Offline CLI that runs ORB-SLAM3 stereo on an inplace-rotate bag, estimates planar `base_link → camera_link` xy from pairwise SE(2) rotation centers, and reports agreement with recorded `/tf_static`.

**Architecture:** New calibration library + executable inside `orb_slam3_wrapper`. Offline MCAP stereo bag reader → headless `OrbSlam3Backend` tracking → horizontal SE(2) projection → pairwise `c = (I−R)⁻¹ t` → mount xy mapping → JSON/HTML report. Math mirrors lidar tool; no TF auto-edit.

**Tech Stack:** C++17, ROS 2 (rclcpp, rosbag2_cpp, sensor_msgs, tf2_msgs, nav_msgs), Eigen, Sophus (via ORB-SLAM3), OpenCV, gtest, ORB-SLAM3 System stereo.

**Spec:** `docs/superpowers/specs/2026-07-17-stereo-rotation-center-calibration-design.md`

## Global Constraints

- Never modify bag, TF, URDF, or launch as a side effect of the tool.
- Single method v1: ORB stereo only; `/odom` optional for motion gates only.
- Planar SE(2) only; fix R and z from bag `/tf_static`.
- Agreement metric: **L∞** on mount xy.
- Exit codes: 0=CONSISTENT, 1=operational failure, 2=LIKELY_OFFSET_ERROR, 3=INCONCLUSIVE.
- Write full report before scientific exit codes 0/2/3.
- Package: `orb_slam3_wrapper` (not `orb_lidar_mapper`).
- TDD: failing test → implement → pass → commit per task.
- First local bag: `/home/duc/robot/bag/inplace-rotate`.

## File Structure

```text
orb_slam3_wrapper/
  include/orb_slam3_wrapper/
    stereo_calib_types.hpp          # Pose2, Point2, ResultClass, configs, samples
    stereo_rotation_center.hpp      # centerFromTransform, pair select, aggregate
    planar_pose_projector.hpp       # optical 6DoF → horizontal SE(2)
    mount_xy_mapper.hpp             # lever arm ↔ base_link→camera_link xy
    stereo_calib_analysis.hpp       # reliability + classify
    stereo_bag_reader.hpp           # offline MCAP ingest
    stereo_calib_pipeline.hpp       # end-to-end run
    stereo_calib_report.hpp         # JSON/CSV/HTML
  src/
    stereo_rotation_center.cpp
    planar_pose_projector.cpp
    mount_xy_mapper.cpp
    stereo_calib_analysis.cpp
    stereo_bag_reader.cpp
    stereo_calib_pipeline.cpp
    stereo_calib_report.cpp
    stereo_rotation_center_check.cpp  # CLI main
  test/
    stereo_rotation_center_test.cpp
    planar_pose_projector_test.cpp
    mount_xy_mapper_test.cpp
    stereo_calib_analysis_test.cpp
    stereo_bag_reader_test.cpp        # synthetic / lightweight if possible
    stereo_calib_report_test.cpp
CMakeLists.txt / package.xml          # lib + exe + tests + rosbag2 deps
tools/run_stereo_rotation_center_calibration.sh
.gitignore                            # tools/stereo-rotation-center-report/
```

Library split: `orb_slam3_wrapper_stereo_calib` (calibration sources, no wrapper_node) linked by tests and `stereo_rotation_center_check`. Reuse existing `orb_slam3_wrapper_core` for `OrbSlam3Backend`, `Calibration::fromCameraInfo`.

---

### Task 1: SE(2) types + `centerFromTransform`

**Files:**
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_calib_types.hpp`
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_rotation_center.hpp`
- Create: `orb_slam3_wrapper/src/stereo_rotation_center.cpp`
- Create: `orb_slam3_wrapper/test/stereo_rotation_center_test.cpp`
- Modify: `orb_slam3_wrapper/CMakeLists.txt`
- Modify: `orb_slam3_wrapper/package.xml` (only if Eigen already covered — it is)

**Interfaces:**
- Produces:
  ```cpp
  namespace orb_slam3_wrapper {
  constexpr double kPi = 3.14159265358979323846;
  struct Point2 { double x{}; double y{}; };
  struct Pose2 {
    double x{}; double y{}; double yaw{};
    static double normalizeAngle(double a);
    Pose2 inverse() const;
    Pose2 compose(const Pose2& b) const; // this * b
  };
  std::optional<Point2> centerFromTransform(const Pose2& source_to_target);
  }
  ```
- Formula (lock sign with lidar): for `p_target = R(yaw) p_source + t`,  
  `c = (I − R)⁻¹ t` with matrix `[[1−c, s], [−s, 1−c]]`.  
  Reject `|det| < 1e-4` or non-finite.

- [ ] **Step 1: Write failing tests**

```cpp
// test/stereo_rotation_center_test.cpp
#include <gtest/gtest.h>
#include "orb_slam3_wrapper/stereo_rotation_center.hpp"

namespace orb_slam3_wrapper {
namespace {

Pose2 about(double yaw, Point2 center) {
  // Relative source→target pure rotation about `center` by +yaw
  // source_to_target maps points: R*(p - c) + c = R*p + (c - R*c)
  // t = c - R*c
  const double c = std::cos(yaw), s = std::sin(yaw);
  Pose2 m;
  m.yaw = yaw;
  m.x = center.x - (c * center.x - s * center.y);
  m.y = center.y - (s * center.x + c * center.y);
  return m;
}

TEST(StereoRotationCenter, LocksSourceToTargetSign) {
  const auto center = centerFromTransform(about(0.40, {0.32, 0.05}));
  ASSERT_TRUE(center.has_value());
  EXPECT_NEAR(center->x, 0.32, 1e-9);
  EXPECT_NEAR(center->y, 0.05, 1e-9);
}

TEST(StereoRotationCenter, RejectsNearlyIdentityRotation) {
  Pose2 almost_id;
  almost_id.yaw = 1e-6;
  almost_id.x = 0.01;
  almost_id.y = 0.0;
  EXPECT_FALSE(centerFromTransform(almost_id).has_value());
}

}  // namespace
}  // namespace orb_slam3_wrapper
```

- [ ] **Step 2: Wire CMake library + test (minimal)**

In `CMakeLists.txt` after core library:

```cmake
find_package(rosbag2_cpp REQUIRED)  # needed later; add now or in Task 5
# For Task 1 only Eigen is required:

add_library(orb_slam3_wrapper_stereo_calib
  src/stereo_rotation_center.cpp)
target_include_directories(orb_slam3_wrapper_stereo_calib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(orb_slam3_wrapper_stereo_calib PUBLIC Eigen3::Eigen)

ament_add_gtest(stereo_rotation_center_test test/stereo_rotation_center_test.cpp)
target_link_libraries(stereo_rotation_center_test orb_slam3_wrapper_stereo_calib)
```

Implement headers/cpp with `Pose2` helpers and `centerFromTransform` matching lidar's matrix.

- [ ] **Step 3: Build and run test**

```bash
cd /home/duc/robot/src/orb_slam3_ros2
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash  # or jazzy/etc as installed
colcon build --packages-select orb_slam3_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select orb_slam3_wrapper --ctest-args -R stereo_rotation_center_test --event-handlers console_direct+
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_calib_types.hpp \
  orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_rotation_center.hpp \
  orb_slam3_wrapper/src/stereo_rotation_center.cpp \
  orb_slam3_wrapper/test/stereo_rotation_center_test.cpp \
  orb_slam3_wrapper/CMakeLists.txt
git commit -m "feat: add stereo SE(2) rotation-center math"
```

---

### Task 2: Planar pose projector

**Files:**
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/planar_pose_projector.hpp`
- Create: `orb_slam3_wrapper/src/planar_pose_projector.cpp`
- Create: `orb_slam3_wrapper/test/planar_pose_projector_test.cpp`
- Modify: `CMakeLists.txt` (add source + test; link Eigen)

**Interfaces:**
```cpp
struct PlanarPose {
  int64_t stamp_ns{};
  Pose2 pose;           // horizontal world: x,y,yaw
  double height_m{};    // vertical residual diagnostic
  bool valid{};
};

// T_world_optical: Eigen::Isometry3d, left optical in ORB world
// R_horizontal_optical: rotation mapping optical axes → horizontal (base-aligned)
//   typically from fixed T_base_optical.linear() so horizontal = base xy
PlanarPose projectToHorizontal(
    int64_t stamp_ns,
    const Eigen::Isometry3d& T_world_optical,
    const Eigen::Matrix3d& R_base_optical);
```

Semantics:
- Optical origin in world: `t = T_world_optical.translation()`.
- Horizontal world position: use world axes already assumed Z-up for pure spin bags after rotating optical pose into base-aligned intermediate, **or** simpler v1:  
  `T_world_base_aligned = T_world_optical * T_optical_base` where `T_base_optical` is fixed from TF, then take `x,y` and yaw-about-Z from that pose’s base frame.
- Prefer the **base-aligned** path (matches design): convert each optical pose to a base-link-orientation pose using fixed `T_base_optical`, then extract SE(2) from that pose’s xy + yaw(Z).

```cpp
// T_world_base = T_world_optical * T_optical_base
// PlanarPose.pose = {tx, ty, yaw_from_rotation_matrix_Z}
// height_m = tz
```

- [ ] **Step 1: Failing test** — pure yaw about base origin with known lever arm:

```cpp
TEST(PlanarPoseProjector, ExtractsYawAndXyFromBaseAlignedPose) {
  Eigen::Isometry3d T_base_optical = Eigen::Isometry3d::Identity();
  T_base_optical.translation() << 0.32, 0.05, 0.17;
  // 90° yaw of base about world Z; optical fixed relative to base
  Eigen::Isometry3d T_world_base = Eigen::Isometry3d::Identity();
  T_world_base.linear() = Eigen::AngleAxisd(kPi/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T_world_optical = T_world_base * T_base_optical;
  const auto planar = projectToHorizontal(0, T_world_optical, T_base_optical);
  ASSERT_TRUE(planar.valid);
  EXPECT_NEAR(planar.pose.yaw, kPi/2, 1e-9);
  EXPECT_NEAR(planar.pose.x, T_world_optical.translation().x(), 1e-9);
  // After converting to base pose, xy should be T_world_base.translation() = 0
  // Implementation: extract from T_world_base = T_world_optical * inv(T_base_optical)
}
```

Clarify implementation contract in code comments:  
`T_world_base = T_world_optical * T_base_optical.inverse()`, planar from `T_world_base`.

- [ ] **Step 2: Implement + CMake + pass tests + commit**

```bash
git commit -m "feat: project ORB optical poses to horizontal SE(2)"
```

---

### Task 3: Mount XY mapper

**Files:**
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/mount_xy_mapper.hpp`
- Create: `orb_slam3_wrapper/src/mount_xy_mapper.cpp`
- Create: `orb_slam3_wrapper/test/mount_xy_mapper_test.cpp`

**Interfaces:**
```cpp
struct StaticCameraMount {
  Eigen::Isometry3d T_base_camera_link{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_camera_link_left_optical{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_base_left_optical() const {
    return T_base_camera_link * T_camera_link_left_optical;
  }
};

// Lever arm c: planar vector from left-camera horizontal origin to rotation
// center, expressed in the horizontal left-camera frame (same as centerFromTransform).
// For pure spin about base origin, in base horizontal frame the optical origin is at
// p_base, and c_base = -p_base_xy (center at origin). Convert frames carefully.
struct MountXy {
  double x_m{};
  double y_m{};
};

// Given estimated c in horizontal-left frame and fixed mount orientations,
// return implied base_link→camera_link translation xy (z kept from recorded).
MountXy impliedCameraLinkXy(
    const Point2& center_in_horizontal_left,
    const StaticCameraMount& recorded);
```

**Math (implement exactly; lock with test):**

Assume horizontal-left axes equal base xy axes rotated by the yaw of
`T_base_left_optical` only (pitch folded into fixed R). Simpler locked approach for v1:

1. From pure rotation about base origin, the base-frame horizontal position of left optical origin is constant in the rotating body:  
   `p_base_xy = translation of T_base_left_optical projected to xy` when orientation is fixed.  
2. Lever arm in base: rotation center (0,0) relative to optical origin = `-p_base_xy`.  
3. Estimated `c` is in horizontal-left frame. Horizontal-left at identity yaw aligns with base when we define projector consistently.  
4. **Recommended locked mapping for tests:**  
   - Work entirely in base horizontal: relative SE(2) from projected **base** poses yields `c_base` = vector from base-frame origin?  
   - Cleaner: run `centerFromTransform` on **base** planar poses (`projectToHorizontal` already yields `T_world_base` SE2). Then `c` is rotation center in **base** frame at pair epoch — for body-fixed center at origin, each sample’s `c` expressed in body should be ~0 if we used base poses…  

**Important design clarification for implementers:**

If planar poses are **base_link** poses in world, pure spin about base origin implies `t≈0` and center estimate at origin — that does **not** observe camera lever arm.

Therefore pairwise center math must run on **left-camera horizontal** poses (position of optical origin in world horizontal, yaw = base yaw or optical yaw about vertical). Then `c` is lever arm in the camera’s horizontal body frame.

Mapping to `base_link → camera_link` xy:

```text
Let T_base_optical_fixed = recorded T_base_left_optical (full SE3).
Let R_bo = yaw-only or full linear part as used by projector.
Estimated c_cam (2D) = optical_origin → rotation_center in horizontal-left.

Rotation center in base = 0.
optical_origin_in_base_xy = - R_base_from_horizontal_left * c_cam

Solve for camera_link translation xy such that
  (T_base_camera_link.translation + R_bc * t_optical_in_camera_link)_xy
  matches optical_origin_in_base_xy, with R_bc and t_optical_in_camera_link
  fixed from recorded TF except camera_link xy free.
```

Minimal implementable version for v1 tests:

```cpp
// If T_camera_link_left_optical is pure known offset and R_base_camera fixed:
// optical_xy_base = f(camera_link_xy, fixed_R, fixed_optical_chain)
// camera_link_xy = inverse_f(optical_xy_base)
// optical_xy_base = -c_rotated_into_base
```

- [ ] **Step 1: Write synthetic test with identity optical chain** (camera_link == optical, R=I):

```cpp
TEST(MountXyMapper, IdentityOpticalChainMapsLeverArmToTranslation) {
  StaticCameraMount m;
  m.T_base_camera_link.setIdentity();
  m.T_camera_link_left_optical.setIdentity();
  // c = vector from camera to rotation center = -camera_position_in_base
  // If camera at (0.32, 0.05), c should be (-0.32, -0.05) in base=camera frame
  Point2 c{-0.32, -0.05};
  const auto xy = impliedCameraLinkXy(c, m);
  EXPECT_NEAR(xy.x_m, 0.32, 1e-9);
  EXPECT_NEAR(xy.y_m, 0.05, 1e-9);
}
```

- [ ] **Step 2: Implement + second test with non-identity optical offset + commit**

```bash
git commit -m "feat: map stereo lever arm to camera_link xy"
```

---

### Task 4: Robust aggregation + classification

**Files:**
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_calib_analysis.hpp`
- Create: `orb_slam3_wrapper/src/stereo_calib_analysis.cpp`
- Create: `orb_slam3_wrapper/test/stereo_calib_analysis_test.cpp`
- Extend: `stereo_calib_types.hpp` with ResultClass, thresholds, sample structs
- Extend: `stereo_rotation_center.hpp/.cpp` with pair selection + per-pair samples if not already

**Interfaces:**
```cpp
enum class ResultClass { kConsistent, kLikelyOffsetError, kInconclusive };

struct StereoCenterSample {
  std::size_t source_index{};
  std::size_t target_index{};
  std::size_t yaw_sector{};
  bool accepted{};
  Point2 center;           // horizontal-left lever arm sample
  MountXy mount_xy;        // mapped sample
  std::string rejection_reason;
};

struct StereoEstimate {
  bool reliable{};
  MountXy median_xy;
  MountXy ci_half_width;   // bootstrap 95% half-width on x and y
  std::size_t accepted_pairs{};
  std::size_t sectors_used{};
  std::vector<std::string> unreliable_reasons;
};

struct StereoAggregate {
  ResultClass result_class{ResultClass::kInconclusive};
  StereoEstimate estimate;
  MountXy recorded_xy;
  MountXy delta_xy;
  std::string summary;
};

// Defaults (provisional from spec)
struct StereoThresholds {
  std::size_t min_accepted_pairs{40};
  std::size_t min_sectors{6};
  double max_ci_half_width_m{0.015};
  double agreement_floor_m{0.010};
  double max_abs_center_m{1.0};
  double min_pair_yaw_rad{10.0 * kPi / 180.0};
  double max_pair_yaw_rad{30.0 * kPi / 180.0};
};

StereoEstimate robustEstimate(const std::vector<StereoCenterSample>&, uint64_t seed,
                              const StereoThresholds&);
StereoAggregate classify(const StereoEstimate&, const MountXy& recorded_xy,
                         const StereoThresholds&);
int resultExitCode(ResultClass); // 0, 2, 3
```

L∞ agreement:  
`max(|Δx|,|Δy|) <= max(agreement_floor_m, max(ci_x, ci_y))`.

Bootstrap: 2000 resamples of accepted mount_xy (or centers then map).

Pair selection from planar trajectory:

```cpp
std::vector<std::pair<size_t,size_t>> selectPosePairs(
  const std::vector<PlanarPose>& poses,
  const std::vector<MotionInterval>& intervals, // empty = all
  double min_yaw, double max_yaw);
// relative source_to_target from planar poses:
// T_st = pose_target.inverse().compose(pose_source)  -- verify sign vs center test
```

Relative pose convention must match `about()` in Task 1 tests. Document and unit-test:
`relative = target.inverse() * source` OR `source.inverse() * target` — **lock to whatever recovers lever arm in synthetic pure rotation of the camera origin**.

- [ ] **Step 1: Tests for relative pose convention + bootstrap reliable/unreliable + L∞ classify**
- [ ] **Step 2: Implement + pass + commit**

```bash
git commit -m "feat: stereo calibration aggregation and classification"
```

---

### Task 5: Stereo bag reader

**Files:**
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_bag_reader.hpp`
- Create: `orb_slam3_wrapper/src/stereo_bag_reader.cpp`
- Create: `orb_slam3_wrapper/test/stereo_bag_reader_test.cpp` (unit tests on TF chain resolution with synthetic messages if bag I/O hard; optional skip full bag in CI)
- Modify: `package.xml` add `rosbag2_cpp`, `tf2_msgs`, `tf2_eigen` if needed
- Modify: `CMakeLists.txt` link rosbag2_cpp, sensor_msgs, tf2_msgs, nav_msgs, cv_bridge, rclcpp

**Interfaces:**
```cpp
struct StereoFrame {
  int64_t stamp_ns{};
  cv::Mat left_bgr_or_gray;
  cv::Mat right_bgr_or_gray;
};

struct StereoDataset {
  std::vector<StereoFrame> frames;  // approx-synced pairs
  sensor_msgs::msg::CameraInfo left_info;
  sensor_msgs::msg::CameraInfo right_info;
  StaticCameraMount recorded_mount;
  MountXy recorded_camera_link_xy;  // from T_base_camera_link
  std::vector<std::pair<int64_t, Pose2>> odom_se2; // optional empty
  std::string left_optical_frame;
  std::string right_optical_frame;
};

struct StereoBagReader {
  static StereoDataset read(const std::filesystem::path& bag_path);
};
```

Topics (defaults, overridable later if needed):
- `/camera/camera/infra1/image_rect_raw`
- `/camera/camera/infra2/image_rect_raw`
- `/camera/camera/infra1/camera_info`
- `/camera/camera/infra2/camera_info`
- `/tf_static`
- `/odom` (optional)

Behavior:
- Open MCAP via `rosbag2_cpp::Reader` (same as lidar).
- Deserialize with `rclcpp::Serialization<T>`.
- Sync stereo by nearest stamp within 5 ms (match wrapper default).
- Build TF buffer from all `/tf_static` transforms; lookup:
  - `base_link` → `camera_link`
  - `camera_link` → `camera_infra1_optical_frame` (or composed chain)
- Fail closed if `base_link→camera_link` missing.
- Convert images to mono8/gray `cv::Mat` for ORB (cv_bridge).
- Progress to stderr while reading.

- [ ] **Step 1: Implement reader + lightweight test for TF composition helper**
- [ ] **Step 2: Manually smoke-read inplace-rotate bag in a tiny debug main or gtest guarded by env `STEREO_CALIB_BAG` if present**
- [ ] **Step 3: Commit**

```bash
git commit -m "feat: offline stereo bag reader for rotation-center tool"
```

---

### Task 6: Pipeline (ORB track + estimate)

**Files:**
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_calib_pipeline.hpp`
- Create: `orb_slam3_wrapper/src/stereo_calib_pipeline.cpp`
- Create: `orb_slam3_wrapper/test/stereo_calib_pipeline_test.cpp` (synthetic poses path without ORB)

**Interfaces:**
```cpp
struct StereoCalibConfig {
  std::filesystem::path bag_path;
  std::filesystem::path output_dir;
  std::filesystem::path vocabulary_file;
  std::filesystem::path settings_file;
  bool overwrite{false};
  StereoThresholds thresholds{};
  // ORB tracking gates
  double max_tracking_loss_fraction{0.25};
  std::size_t min_tracked_frames{200};
};

struct TrackedPose {
  int64_t stamp_ns{};
  Eigen::Isometry3d T_world_optical{Eigen::Isometry3d::Identity()};
  int tracking_state{};
  bool pose_valid{};
};

struct StereoCalibRun {
  StereoCalibConfig config;
  StereoDataset dataset;
  std::vector<TrackedPose> trajectory;
  std::vector<PlanarPose> planar;
  std::vector<StereoCenterSample> samples;
  StereoAggregate aggregate;
  std::size_t tracked_ok{};
  std::size_t tracked_total{};
};

StereoCalibRun runStereoCalibration(const StereoCalibConfig& config);
// stages:
// 1 read bag
// 2 Calibration::fromCameraInfo + OrbSlam3Backend configure + TrackStereo loop
// 3 project planar
// 4 select pairs + centerFromTransform + map xy
// 5 robustEstimate + classify
```

ORB integration:
- Reuse `OrbSlam3Backend(vocab, settings)`.
- `configureCalibration` with `Calibration::fromCameraInfo`.
- For each synced frame: `trackStereo(left, right, stamp_sec)` → `FrameSnapshot`.
- Convert `Sophus::SE3f T_world_camera` → `Eigen::Isometry3d`.
- Stage progress labels on stderr: bag, ORB, pairs, estimate, report.

Synthetic unit test: inject planar poses of a camera at (0.32,0.05) spinning; ensure aggregate median ~ recorded without calling ORB.

- [ ] **Step 1: Synthetic pipeline path test (no ORB)**
- [ ] **Step 2: Full ORB path in `runStereoCalibration`**
- [ ] **Step 3: Commit**

```bash
git commit -m "feat: stereo rotation-center calibration pipeline"
```

---

### Task 7: Report writer

**Files:**
- Create: `orb_slam3_wrapper/include/orb_slam3_wrapper/stereo_calib_report.hpp`
- Create: `orb_slam3_wrapper/src/stereo_calib_report.cpp`
- Create: `orb_slam3_wrapper/test/stereo_calib_report_test.cpp`

**Interfaces:**
```cpp
void writeStereoCalibrationReport(const StereoCalibRun& run);
// writes under run.config.output_dir:
//   calibration.json, centers.csv, trajectory.csv, report.html
// respects overwrite flag; creates directories
```

`calibration.json` fields (minimum):
- `result_class`, `exit_code_meaning`
- `recorded_camera_link_xy`, `estimated_camera_link_xy`, `delta_xy`, `ci_half_width`
- `accepted_pairs`, `sectors_used`, `tracked_ok`, `tracked_total`
- `thresholds`, `bag_path`
- `recorded_T_base_camera_link` (xyz + quaternion)
- `unreliable_reasons` / `summary`

HTML: self-contained, comparison table, simple SVG or canvas top-down trajectory, center scatter (can be minimal static SVG from numbers).

- [ ] **Step 1: Test writes files for a fake StereoCalibRun**
- [ ] **Step 2: Implement + commit**

```bash
git commit -m "feat: stereo calibration JSON/CSV/HTML report"
```

---

### Task 8: CLI executable + run script + gitignore

**Files:**
- Create: `orb_slam3_wrapper/src/stereo_rotation_center_check.cpp`
- Create: `tools/run_stereo_rotation_center_calibration.sh` (mode +x)
- Modify: `orb_slam3_wrapper/CMakeLists.txt` (add_executable + install)
- Modify: `.gitignore` add `tools/stereo-rotation-center-report/`

**CLI:**
```cpp
// parse: --bag --output --overwrite --vocab --settings
// defaults:
//   vocab = ament_index share orb_slam3_vendor/vocabulary/ORBvoc.txt
//   settings = ament_index share orb_slam3_wrapper/config/tasterobot_stereo.yaml
// try { run; write report; return resultExitCode; }
// catch (...) { stderr; return 1; }
```

Script: clone structure of `tools/run_lidar_rotation_center_calibration.sh` with package `orb_slam3_wrapper` and binary `stereo_rotation_center_check`.

- [ ] **Step 1: Implement CLI + CMake install**
- [ ] **Step 2: Build package**

```bash
colcon build --packages-select orb_slam3_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- [ ] **Step 3: Run unit tests**

```bash
colcon test --packages-select orb_slam3_wrapper --event-handlers console_direct+
colcon test-result --verbose
```

- [ ] **Step 4: Commit**

```bash
git commit -m "feat: stereo_rotation_center_check CLI and run script"
```

---

### Task 9: Local bag smoke run (manual verification)

**Files:** none required (runtime only); optional sample HTML later.

- [ ] **Step 1: Run on fixture**

```bash
tools/run_stereo_rotation_center_calibration.sh \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output tools/stereo-rotation-center-report/inplace-rotate
```

- [ ] **Step 2: Inspect**

```bash
python3 -c "import json;print(json.load(open('tools/stereo-rotation-center-report/inplace-rotate/calibration.json'))['result_class'])"
# or jq if available
ls -la tools/stereo-rotation-center-report/inplace-rotate/
```

Expected: report files exist; exit code 0/2/3 (not 1). Tracking should succeed on feature-rich bag. If ORB fails operationally, fix and re-run (do not weaken fail-closed without documenting).

- [ ] **Step 3: Commit only code fixes if any; do not commit large report outputs** (gitignored). Optionally add a short note under docs if thresholds needed loosening — prefer code comment + JSON thresholds field.

```bash
git commit -m "fix: stereo calib smoke issues from inplace-rotate"  # if needed
```

---

## Spec Coverage Checklist

| Spec requirement | Task |
|---|---|
| Offline MCAP stereo + tf_static | 5 |
| Full ORB-SLAM3 stereo tracking + 6DoF log | 6 |
| Pairwise SE(2) center math | 1, 4 |
| Planar only; R/z fixed | 2, 3 |
| Implied base_link→camera_link xy | 3 |
| ORB-only method | 6 |
| Report JSON/CSV/HTML | 7 |
| Exit classes + L∞ | 4 |
| No TF auto-edit | 6, 8 (invariant) |
| Unit tests math/mapping/classify | 1–4, 7 |
| Run script | 8 |
| inplace-rotate first fixture | 9 |

## Type Consistency Notes

- Namespace: `orb_slam3_wrapper` for all new types.
- `Point2` / `Pose2` / `MountXy` defined once in `stereo_calib_types.hpp`.
- `centerFromTransform` takes **source_to_target** Pose2 (lidar-compatible matrix).
- Planar poses used for pairing are **left-camera horizontal** poses (not base poses), so lever arm is observable.
- `resultExitCode`: Consistent=0, LikelyOffsetError=2, Inconclusive=3; operational=1 in main only.
