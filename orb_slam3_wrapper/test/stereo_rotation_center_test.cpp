#include <cmath>

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
