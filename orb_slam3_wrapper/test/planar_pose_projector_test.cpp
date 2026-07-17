#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "orb_slam3_wrapper/planar_pose_projector.hpp"
#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {
namespace {

TEST(PlanarPoseProjector, ExtractsYawAndXyFromBaseAlignedPose) {
  Eigen::Isometry3d T_base_optical = Eigen::Isometry3d::Identity();
  T_base_optical.translation() << 0.32, 0.05, 0.17;
  // 90° yaw of base about world Z; optical fixed relative to base
  Eigen::Isometry3d T_world_base = Eigen::Isometry3d::Identity();
  T_world_base.linear() =
      Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T_world_optical = T_world_base * T_base_optical;
  const auto planar = projectToHorizontal(0, T_world_optical, T_base_optical);
  ASSERT_TRUE(planar.valid);
  EXPECT_NEAR(planar.pose.yaw, kPi / 2, 1e-9);
  // After converting to base pose, xy should be T_world_base.translation() = 0
  // Implementation: extract from T_world_base = T_world_optical * inv(T_base_optical)
  EXPECT_NEAR(planar.pose.x, 0.0, 1e-9);
  EXPECT_NEAR(planar.pose.y, 0.0, 1e-9);
  EXPECT_NEAR(planar.height_m, 0.0, 1e-9);
  EXPECT_EQ(planar.stamp_ns, 0);
}

TEST(PlanarPoseProjector, PropagatesStampAndHeight) {
  Eigen::Isometry3d T_base_optical = Eigen::Isometry3d::Identity();
  T_base_optical.translation() << 0.1, 0.0, 0.2;
  Eigen::Isometry3d T_world_base = Eigen::Isometry3d::Identity();
  T_world_base.translation() << 1.5, -2.0, 0.3;
  T_world_base.linear() =
      Eigen::AngleAxisd(-kPi / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Isometry3d T_world_optical = T_world_base * T_base_optical;
  const int64_t stamp = 1'234'567'890LL;
  const auto planar =
      projectToHorizontal(stamp, T_world_optical, T_base_optical);
  ASSERT_TRUE(planar.valid);
  EXPECT_EQ(planar.stamp_ns, stamp);
  EXPECT_NEAR(planar.pose.x, 1.5, 1e-9);
  EXPECT_NEAR(planar.pose.y, -2.0, 1e-9);
  EXPECT_NEAR(planar.pose.yaw, -kPi / 4, 1e-9);
  EXPECT_NEAR(planar.height_m, 0.3, 1e-9);
}

TEST(PlanarPoseProjector, InvalidWhenNonFinite) {
  Eigen::Isometry3d T_base_optical = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_world_optical = Eigen::Isometry3d::Identity();
  T_world_optical.translation() << std::numeric_limits<double>::quiet_NaN(),
      0.0, 0.0;
  const auto planar =
      projectToHorizontal(0, T_world_optical, T_base_optical);
  EXPECT_FALSE(planar.valid);
}

}  // namespace
}  // namespace orb_slam3_wrapper
