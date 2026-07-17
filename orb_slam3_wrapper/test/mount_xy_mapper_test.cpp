#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "orb_slam3_wrapper/mount_xy_mapper.hpp"
#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {
namespace {

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

TEST(MountXyMapper, NonIdentityOpticalOffsetSubtractsFixedChain) {
  // camera_link → left_optical is a known pure translation offset.
  // With identity rotations, optical_xy_base = -c, and
  // camera_link_xy = optical_xy_base - t_optical_in_camera_link.xy
  StaticCameraMount m;
  m.T_base_camera_link.setIdentity();
  m.T_base_camera_link.translation() << 0.0, 0.0, 0.12;  // z kept from recorded
  m.T_camera_link_left_optical.setIdentity();
  m.T_camera_link_left_optical.translation() << 0.05, -0.02, 0.10;

  Point2 c{-0.32, -0.05};
  const auto xy = impliedCameraLinkXy(c, m);
  // optical_xy_base = -c = (0.32, 0.05)
  // camera_link_xy = (0.32, 0.05) - (0.05, -0.02) = (0.27, 0.07)
  EXPECT_NEAR(xy.x_m, 0.27, 1e-9);
  EXPECT_NEAR(xy.y_m, 0.07, 1e-9);
}

TEST(MountXyMapper, RotatedMountMapsLeverArmThroughYaw) {
  // Horizontal-left is yawed 90° relative to base. R maps cam → base:
  // optical_xy_base = -R_z(π/2) * c
  StaticCameraMount m;
  m.T_base_camera_link.setIdentity();
  m.T_base_camera_link.linear() =
      Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  m.T_camera_link_left_optical.setIdentity();

  Point2 c{-0.32, -0.05};
  // R_z(π/2) * c = (-(-0.05), -0.32) wait: R*[x,y] = [-y, x]
  // R * c = -(-0.05), -0.32?  R*[cx,cy] = [-cy, cx] = [0.05, -0.32]
  // optical_xy_base = -R*c = (-0.05, 0.32)
  // camera_link_xy = optical_xy_base (identity optical chain, but R_bc rotates t_co=0)
  const auto xy = impliedCameraLinkXy(c, m);
  EXPECT_NEAR(xy.x_m, -0.05, 1e-9);
  EXPECT_NEAR(xy.y_m, 0.32, 1e-9);
}

}  // namespace
}  // namespace orb_slam3_wrapper
