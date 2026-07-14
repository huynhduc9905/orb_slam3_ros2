#include <gtest/gtest.h>
#include <cmath>
#include "orb_slam3_wrapper/pose_conversion.hpp"

namespace {
using Eigen::Isometry3d;
using Sophus::SE3f;

Isometry3d opticalToBase() {
  Isometry3d result = Isometry3d::Identity();
  result.linear() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  result.translation() << 0.30, 0.04, 0.22;
  return result;
}
SE3f pose(const Eigen::Quaternionf& r, const Eigen::Vector3f& t) { return SE3f(r, t); }
SE3f pose(const Eigen::Isometry3d& transform) { return SE3f(transform.matrix().cast<float>()); }
void expectPoseNear(const Isometry3d& actual, const Isometry3d& expected) {
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) EXPECT_NEAR(actual(row, col), expected(row, col), 1e-6);
}
Isometry3d asIsometry(const Eigen::Matrix4d& matrix) {
  Isometry3d result; result.matrix() = matrix; return result;
}
}

TEST(PoseConversion, FirstAnchorIsIdentityAndPreservesFullSE3) {
  orb_slam3_wrapper::PoseConverter converter(opticalToBase());
  const auto first = pose(Eigen::Quaternionf(Eigen::AngleAxisf(0.2F, Eigen::Vector3f::UnitY())),
                          Eigen::Vector3f(1.0F, -0.3F, 2.0F));
  expectPoseNear(converter.anchor(first), Isometry3d::Identity());
  EXPECT_TRUE(converter.initialized());
}

TEST(PoseConversion, ConvertsOpticalForwardMotionToBaseMotion) {
  orb_slam3_wrapper::PoseConverter converter(opticalToBase());
  converter.anchor(pose(opticalToBase()));
  auto later_transform = opticalToBase();
  later_transform.translation().x() += 1.0;
  const auto later = pose(later_transform);
  Isometry3d expected = Isometry3d::Identity(); expected.translation().x() = 1.0;
  expectPoseNear(converter.toBasePose(later), expected);
}

TEST(PoseConversion, PureBaseRotationDoesNotInventTranslationFromCameraOffset) {
  const auto extrinsic = opticalToBase(); orb_slam3_wrapper::PoseConverter converter(extrinsic);
  converter.anchor(pose(extrinsic));
  const Eigen::AngleAxisf rotation(0.7F, Eigen::Vector3f::UnitZ());
  Eigen::Isometry3d rotated_base = Eigen::Isometry3d::Identity();
  rotated_base.linear() = rotation.toRotationMatrix().cast<double>();
  const auto actual = converter.toBasePose(pose(rotated_base * extrinsic));
  EXPECT_NEAR(actual.translation().norm(), 0.0, 1e-6);
  EXPECT_NEAR(actual.linear()(0, 0), std::cos(0.7), 1e-6);
  EXPECT_NEAR(actual.linear()(1, 0), std::sin(0.7), 1e-6);
}

TEST(PoseConversion, ReferenceToFrameUsesNonCommutingWorldComposition) {
  orb_slam3_wrapper::PoseConverter converter(Isometry3d::Identity());
  const auto reference = pose(Eigen::Quaternionf(Eigen::AngleAxisf(0.4F, Eigen::Vector3f::UnitZ())),
                              Eigen::Vector3f(2.0F, 0.0F, 0.0F));
  const auto current = pose(Eigen::Quaternionf(Eigen::AngleAxisf(-0.2F, Eigen::Vector3f::UnitY())),
                            Eigen::Vector3f(2.5F, 0.7F, -0.1F));
  const auto actual = converter.referenceToFrame(reference, current);
  const Isometry3d expected = asIsometry((reference.inverse() * current).matrix().cast<double>());
  expectPoseNear(actual, expected);
  EXPECT_GT((actual.matrix() - (current * reference.inverse()).matrix().cast<double>()).norm(), 0.1);
}

TEST(PoseConversion, ReferenceToBaseUsesRelativeCameraTransformAndOffsetConjugation) {
  const auto extrinsic = opticalToBase();
  orb_slam3_wrapper::PoseConverter converter(extrinsic);
  converter.anchor(pose(extrinsic));
  const auto relative = pose(Eigen::Quaternionf(Eigen::AngleAxisf(0.6F, Eigen::Vector3f::UnitZ())),
                             Eigen::Vector3f(0.4F, -0.2F, 0.1F));
  const auto actual = converter.referenceToBaseFrame(relative);
  const auto cameraBase = extrinsic.inverse();
  const auto expected = extrinsic * asIsometry(relative.matrix().cast<double>()) * cameraBase;
  expectPoseNear(actual, expected);
  EXPECT_GT((actual.matrix() - asIsometry(relative.matrix().cast<double>()).matrix()).norm(), 0.1);
}
