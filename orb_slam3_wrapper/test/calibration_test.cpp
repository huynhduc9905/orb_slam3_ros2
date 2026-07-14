#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include "orb_slam3_wrapper/calibration.hpp"

namespace {
sensor_msgs::msg::CameraInfo makeInfo(
    const uint32_t width = 848, const uint32_t height = 480,
    const double fx = 426.9840393066406,
    const double cx = 430.81121826171875,
    const double cy = 238.95848083496094, const double p3 = 0.0) {
  sensor_msgs::msg::CameraInfo info;
  info.width = width; info.height = height;
  info.k[0] = fx; info.k[4] = fx; info.k[2] = cx; info.k[5] = cy;
  info.p[0] = fx; info.p[5] = fx; info.p[2] = cx; info.p[3] = p3; info.p[6] = cy;
  return info;
}
}

TEST(Calibration, DerivesBaselineFromRightProjectionAndTrustsImageFrames) {
  auto left = makeInfo(); auto right = makeInfo();
  right.p[3] = -21.429536819458008;
  right.header.frame_id = "camera_infra1_optical_frame";
  const auto result = orb_slam3_wrapper::Calibration::fromCameraInfo(
      left, right, "camera_infra1_optical_frame", "camera_infra2_optical_frame");
  EXPECT_NEAR(result.baseline_m, -right.p[3] / right.p[0], 1e-12);
  EXPECT_NEAR(result.baseline_m, 0.0501881428, 3e-9);
  EXPECT_EQ(result.left_frame, "camera_infra1_optical_frame");
  EXPECT_EQ(result.right_frame, "camera_infra2_optical_frame");
}

TEST(Calibration, RejectsUnexpectedDimensions) {
  auto left = makeInfo(); auto right = makeInfo(640);
  right.p[3] = -21.429536819458008;
  EXPECT_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "left", "right"),
               std::invalid_argument);
}

TEST(Calibration, RejectsMalformedIntrinsicsAndBaseline) {
  auto left = makeInfo(); auto right = makeInfo();
  right.p[3] = -21.429536819458008; left.k[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "left", "right"),
               std::invalid_argument);
  left = makeInfo(); right = makeInfo();
  EXPECT_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "left", "right"),
               std::invalid_argument);
}

TEST(Calibration, RejectsStereoIntrinsicsMismatch) {
  auto left = makeInfo(); auto right = makeInfo();
  right.p[3] = -21.429536819458008; right.p[0] += 0.5;
  EXPECT_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "left", "right"),
               std::invalid_argument);
}
