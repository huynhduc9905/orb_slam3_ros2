#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "orb_slam3_wrapper/mount_xy_mapper.hpp"
#include "orb_slam3_wrapper/planar_pose_projector.hpp"
#include "orb_slam3_wrapper/stereo_calib_pipeline.hpp"
#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {
namespace {

PlanarPose makePlanar(int64_t stamp_ns, double x, double y, double yaw) {
  PlanarPose p;
  p.stamp_ns = stamp_ns;
  p.pose = {x, y, yaw};
  p.height_m = 0.0;
  p.valid = true;
  return p;
}

// Pure spin of left-optical origin about world origin with fixed body offset p.
// pose = T_world_left: origin at R(θ)*p, yaw = θ.
std::vector<PlanarPose> pureSpinPlanarLeft(Point2 optical_in_base, int n,
                                           double dtheta) {
  std::vector<PlanarPose> poses;
  poses.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double th = i * dtheta;
    const double c = std::cos(th), s = std::sin(th);
    const double x = c * optical_in_base.x - s * optical_in_base.y;
    const double y = s * optical_in_base.x + c * optical_in_base.y;
    poses.push_back(
        makePlanar(static_cast<int64_t>(i) * 1'000'000'000LL, x, y, th));
  }
  return poses;
}

TEST(StereoCalibPipeline, PlanarLeftFromBaseAppliesOpticalOffset) {
  // Base at origin, yaw=0; optical at (0.32, 0.05) in base → left planar same.
  Eigen::Isometry3d T_world_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_base_optical = Eigen::Isometry3d::Identity();
  T_base_optical.translation() << 0.32, 0.05, 0.17;

  const auto left =
      planarLeftFromBasePose(/*stamp=*/42, T_world_base, T_base_optical);
  ASSERT_TRUE(left.valid);
  EXPECT_EQ(left.stamp_ns, 42);
  EXPECT_NEAR(left.pose.x, 0.32, 1e-12);
  EXPECT_NEAR(left.pose.y, 0.05, 1e-12);
  EXPECT_NEAR(left.pose.yaw, 0.0, 1e-12);

  // Base yaw 90°, still at origin: optical maps to (-0.05, 0.32).
  T_world_base.linear() =
      Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto left_r =
      planarLeftFromBasePose(0, T_world_base, T_base_optical);
  ASSERT_TRUE(left_r.valid);
  EXPECT_NEAR(left_r.pose.x, -0.05, 1e-12);
  EXPECT_NEAR(left_r.pose.y, 0.32, 1e-12);
  EXPECT_NEAR(left_r.pose.yaw, kPi / 2, 1e-12);
}

TEST(StereoCalibPipeline, EstimateFromPlanarPureSpinRecoversMount) {
  // Synthetic path: inject planar left poses of camera at (0.32, 0.05)
  // spinning about base origin; identity optical chain → median ≈ recorded.
  const Point2 optical_in_base{0.32, 0.05};
  const MountXy recorded_xy{0.32, 0.05};

  StaticCameraMount identity_mount;
  identity_mount.T_base_camera_link.setIdentity();
  identity_mount.T_base_camera_link.translation() << 0.32, 0.05, 0.17;
  identity_mount.T_camera_link_left_optical.setIdentity();

  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;
  thr.max_ci_half_width_m = 0.015;
  thr.agreement_floor_m = 0.010;
  thr.max_abs_center_m = 1.0;
  thr.min_pair_yaw_rad = 10.0 * kPi / 180.0;
  thr.max_pair_yaw_rad = 30.0 * kPi / 180.0;

  // Dense pure spin: Δθ = 0.20 rad between neighbors → pairs in [10°, 30°].
  const auto planar_left = pureSpinPlanarLeft(optical_in_base, 64, 0.20);

  const auto run = estimateFromPlanarPoses(thr, identity_mount, recorded_xy,
                                           planar_left, /*seed=*/42);

  ASSERT_EQ(run.planar.size(), planar_left.size());
  EXPECT_GE(run.samples.size(), thr.min_accepted_pairs);
  EXPECT_TRUE(run.aggregate.estimate.reliable) << [&] {
    std::string s;
    for (const auto& r : run.aggregate.estimate.unreliable_reasons) {
      s += r + ";";
    }
    return s;
  }();
  EXPECT_NEAR(run.aggregate.estimate.median_xy.x_m, recorded_xy.x_m, 1e-4);
  EXPECT_NEAR(run.aggregate.estimate.median_xy.y_m, recorded_xy.y_m, 1e-4);
  EXPECT_EQ(run.aggregate.result_class, ResultClass::kConsistent);
  EXPECT_NEAR(run.aggregate.delta_xy.x_m, 0.0, 1e-4);
  EXPECT_NEAR(run.aggregate.delta_xy.y_m, 0.0, 1e-4);
}

TEST(StereoCalibPipeline, EstimateFromPlanarRejectsOversizedCenter) {
  // Lever arm beyond max_abs_center_m → samples rejected → inconclusive.
  const Point2 optical_in_base{2.5, 0.0};
  const MountXy recorded_xy{2.5, 0.0};

  StaticCameraMount identity_mount;
  identity_mount.T_base_camera_link.setIdentity();
  identity_mount.T_base_camera_link.translation() << 2.5, 0.0, 0.0;
  identity_mount.T_camera_link_left_optical.setIdentity();

  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;
  thr.max_abs_center_m = 1.0;
  thr.min_pair_yaw_rad = 10.0 * kPi / 180.0;
  thr.max_pair_yaw_rad = 30.0 * kPi / 180.0;

  const auto planar_left = pureSpinPlanarLeft(optical_in_base, 64, 0.20);
  const auto run = estimateFromPlanarPoses(thr, identity_mount, recorded_xy,
                                           planar_left, /*seed=*/1);

  // Centers |c| = 2.5 > 1.0 → all gated; estimate unreliable / inconclusive.
  EXPECT_EQ(run.aggregate.result_class, ResultClass::kInconclusive);
  EXPECT_FALSE(run.aggregate.estimate.reliable);
}

TEST(StereoCalibPipeline, SophusToIsometryPreservesTranslationAndRotation) {
  Sophus::SE3f T(Eigen::Quaternionf(Eigen::AngleAxisf(
                     static_cast<float>(kPi / 3), Eigen::Vector3f::UnitZ())),
                 Eigen::Vector3f(1.0f, -2.0f, 0.5f));
  const Eigen::Isometry3d iso = sophusSe3fToIsometry(T);
  EXPECT_NEAR(iso.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(iso.translation().y(), -2.0, 1e-6);
  EXPECT_NEAR(iso.translation().z(), 0.5, 1e-6);
  const Eigen::Matrix3d R = iso.linear();
  EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), kPi / 3, 1e-5);
}

}  // namespace
}  // namespace orb_slam3_wrapper
