#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/planar_icp.hpp"
#include "orb_lidar_mapper/rotation_center_estimator.hpp"

namespace orb_lidar_mapper {
namespace {

Point2 apply(const Pose2& pose, Point2 p) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y};
}

Pose2 rotationAboutCenter(double yaw, Point2 center) {
  return {center.x - std::cos(yaw) * center.x + std::sin(yaw) * center.y,
          center.y - std::sin(yaw) * center.x - std::cos(yaw) * center.y, yaw};
}

std::vector<Point2> room() {
  return {{-2.0, -1.0}, {-1.2, -1.0}, {-0.4, -1.0}, {0.7, -1.0},
          {1.8, -1.0}, {2.3, -0.3}, {2.3, 0.4}, {1.7, 1.4},
          {0.2, 1.7}, {-0.8, 1.3}, {-1.7, 1.1}, {-2.1, 0.2},
          {-0.4, 0.15}, {0.7, 0.5}, {1.1, -0.2}, {-1.3, -0.2}};
}

TEST(PlanarIcp, RecoversAsymmetricRoomMotion) {
  const auto source = room();
  const auto truth = rotationAboutCenter(0.35, {0.26, 0.0});
  std::vector<Point2> target;
  for (const auto point : source) target.push_back(apply(truth, point));

  const auto result = PlanarIcp(IcpConfig{}).align(source, target, 0.35);

  ASSERT_TRUE(result.converged) << result.rejection_reason;
  ASSERT_TRUE(result.rejection_reason.empty());
  ASSERT_TRUE(std::isfinite(result.trimmed_rmse_m));
  EXPECT_GE(result.overlap_ratio, 0.40);
  EXPECT_LE(result.trimmed_rmse_m, 0.05);
  const auto center = centerFromTransform(result.source_to_target);
  ASSERT_TRUE(center);
  EXPECT_NEAR(center->x, 0.26, 0.005);
  EXPECT_NEAR(center->y, 0.0, 0.005);
}

TEST(PlanarIcp, RejectsInsufficientPointsAndDegenerateGeometry) {
  const auto blank = PlanarIcp(IcpConfig{}).align({{}, {}, {}}, {{}, {}, {}}, 0.2);
  EXPECT_FALSE(blank.converged);
  EXPECT_EQ(blank.rejection_reason, "insufficient_points");

  std::vector<Point2> circle;
  for (int i = 0; i < 16; ++i) {
    const double a = 2.0 * kPi * i / 16.0;
    circle.push_back({std::cos(a), std::sin(a)});
  }
  const auto degenerate = PlanarIcp(IcpConfig{}).align(circle, circle, 0.2);
  EXPECT_FALSE(degenerate.converged);
  EXPECT_EQ(degenerate.rejection_reason, "degenerate_geometry");
}

TEST(PlanarIcp, RejectsYawDisagreement) {
  const auto source = room();
  const auto truth = rotationAboutCenter(0.35, {0.26, 0.0});
  std::vector<Point2> target;
  for (const auto point : source) target.push_back(apply(truth, point));
  const auto result = PlanarIcp(IcpConfig{}).align(source, target, 0.0);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.rejection_reason, "yaw_disagreement");
}

TEST(PlanarIcp, TrimsDeterministicallyAndReportsOverlap) {
  const auto source = room();
  const auto truth = rotationAboutCenter(0.35, {0.26, 0.0});
  std::vector<Point2> target;
  for (const auto point : source) target.push_back(apply(truth, point));
  target.push_back({20.0, 20.0});
  target.push_back({-20.0, 20.0});

  const auto first = PlanarIcp(IcpConfig{}).align(source, target, 0.35);
  const auto second = PlanarIcp(IcpConfig{}).align(source, target, 0.35);
  EXPECT_EQ(first.correspondence_count, second.correspondence_count);
  EXPECT_DOUBLE_EQ(first.trimmed_rmse_m, second.trimmed_rmse_m);
  EXPECT_TRUE(first.converged) << first.rejection_reason;
}

}  // namespace
}  // namespace orb_lidar_mapper
