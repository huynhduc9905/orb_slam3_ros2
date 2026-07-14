#include <cmath>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/pose2.hpp"

namespace orb_lidar_mapper {
namespace {

TEST(Pose2, IdentityLeavesPoseUnchanged) {
  const Pose2 pose{1.0, -2.0, 0.4};
  EXPECT_TRUE((Pose2{} * pose).isApprox(pose, 1e-12));
  EXPECT_TRUE((pose * Pose2{}).isApprox(pose, 1e-12));
}

TEST(Pose2, InverseCancelsPose) {
  const Pose2 pose{1.0, -2.0, 0.4};
  EXPECT_TRUE((pose * pose.inverse()).isApprox(Pose2{}, 1e-12));
}

TEST(Pose2, CompositionAppliesRightPoseInLeftFrame) {
  const Pose2 left{1.0, 2.0, M_PI_2};
  const Pose2 right{2.0, 0.0, M_PI_2};
  EXPECT_TRUE((left * right).isApprox(Pose2{1.0, 4.0, M_PI}, 1e-12));
}

TEST(Pose2, NormalizesAngleToShortestEquivalent) {
  EXPECT_NEAR(Pose2::normalizeAngle(3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(Pose2::normalizeAngle(-3.0 * M_PI), -M_PI, 1e-12);
}

TEST(Pose2, ResidualInterpolationHitsBothEndpoints) {
  const Pose2 predicted_end{2.0, 0.0, 0.0};
  const Pose2 recovered_end{1.8, 0.3, 0.2};
  const Pose2 residual = predicted_end.inverse() * recovered_end;
  EXPECT_TRUE((predicted_end * residual.pow(0.0)).isApprox(predicted_end, 1e-12));
  EXPECT_TRUE((predicted_end * residual.pow(1.0)).isApprox(recovered_end, 1e-12));
}

TEST(Pose2, InterpolatesAcrossPiByShortestArc) {
  const Pose2 a{0.0, 0.0, 3.124};
  const Pose2 b{0.0, 0.0, -3.124};
  EXPECT_NEAR(std::abs(Pose2::interpolate(a, b, 0.5).yaw), M_PI, 0.02);
}

TEST(Pose2, ExpAndLogRoundTrip) {
  const Twist2 twist{1.3, -0.7, 0.9};
  const Twist2 recovered = Pose2::exp(twist).log();
  EXPECT_NEAR(recovered.vx, twist.vx, 1e-12);
  EXPECT_NEAR(recovered.vy, twist.vy, 1e-12);
  EXPECT_NEAR(recovered.omega, twist.omega, 1e-12);
}

TEST(Pose2, ExpAndLogRoundTripThroughSmallAngleBranch) {
  const Twist2 twist{1.3, -0.7, 1e-10};
  const Twist2 recovered = Pose2::exp(twist).log();
  EXPECT_NEAR(recovered.vx, twist.vx, 1e-12);
  EXPECT_NEAR(recovered.vy, twist.vy, 1e-12);
  EXPECT_NEAR(recovered.omega, twist.omega, 1e-16);
}

TEST(Pose2, PowClampsAlphaToUnitInterval) {
  const Pose2 pose{1.0, -2.0, 0.4};
  EXPECT_TRUE(pose.pow(-0.5).isApprox(Pose2{}, 1e-12));
  EXPECT_TRUE(pose.pow(1.5).isApprox(pose, 1e-12));
}

}  // namespace
}  // namespace orb_lidar_mapper
