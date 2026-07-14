#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <string>
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
  info.k = {fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, p3, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
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

TEST(Calibration, RejectsEmptyOrAliasedImageFrames) {
  auto left = makeInfo(); auto right = makeInfo(); right.p[3] = -21.429536819458008;
  EXPECT_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "", "right"), std::invalid_argument);
  EXPECT_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "same", "same"), std::invalid_argument);
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

struct MalformedRectifiedMatrixCase {
  const char* name;
  void (*mutate)(sensor_msgs::msg::CameraInfo&, sensor_msgs::msg::CameraInfo&);
};

void malformedKOffDiagonal(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.k[1] = 0.1;
}

void malformedKOtherOffDiagonal(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.k[3] = 0.1;
}

void malformedKLowerOffDiagonal(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.k[6] = 0.1;
}

void malformedKLowerRight(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.k[7] = 0.1;
}

void malformedKBottomRow(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.k[8] = 0.9;
}

void malformedPStructure(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[1] = 0.1;
}

void malformedPSecondRow(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[4] = 0.1;
}

void malformedPSecondRowEnd(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[7] = 0.1;
}

void malformedPBottomLeft(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[8] = 0.1;
}

void malformedPBottomMiddle(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[9] = 0.1;
}

void malformedPBottomScale(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[10] = 0.9;
}

void malformedPBottomEnd(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[11] = 0.1;
}

void malformedLeftTx(sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.p[3] = 0.1;
}

void malformedKPrincipalPointMismatch(
    sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.k[2] += 1.0;
}

void malformedKVerticalPrincipalPointMismatch(
    sensor_msgs::msg::CameraInfo& left, sensor_msgs::msg::CameraInfo&) {
  left.k[5] += 1.0;
}

void malformedNonFiniteUnconsumedEntry(
    sensor_msgs::msg::CameraInfo&, sensor_msgs::msg::CameraInfo& right) {
  right.p[11] = std::numeric_limits<double>::infinity();
}

class CalibrationMalformedRectifiedMatrixTest
    : public ::testing::TestWithParam<MalformedRectifiedMatrixCase> {};

TEST_P(CalibrationMalformedRectifiedMatrixTest, RejectsMalformedInput) {
  auto left = makeInfo();
  auto right = makeInfo();
  right.p[3] = -21.429536819458008;
  GetParam().mutate(left, right);
  EXPECT_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "left", "right"),
               std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(
    RectifiedMatrices, CalibrationMalformedRectifiedMatrixTest,
    ::testing::Values(
        MalformedRectifiedMatrixCase{"K off-diagonal", malformedKOffDiagonal},
        MalformedRectifiedMatrixCase{"K other off-diagonal", malformedKOtherOffDiagonal},
        MalformedRectifiedMatrixCase{"K lower off-diagonal", malformedKLowerOffDiagonal},
        MalformedRectifiedMatrixCase{"K lower right", malformedKLowerRight},
        MalformedRectifiedMatrixCase{"K bottom row", malformedKBottomRow},
        MalformedRectifiedMatrixCase{"P structure", malformedPStructure},
        MalformedRectifiedMatrixCase{"P second row", malformedPSecondRow},
        MalformedRectifiedMatrixCase{"P second row end", malformedPSecondRowEnd},
        MalformedRectifiedMatrixCase{"P bottom left", malformedPBottomLeft},
        MalformedRectifiedMatrixCase{"P bottom middle", malformedPBottomMiddle},
        MalformedRectifiedMatrixCase{"P bottom scale", malformedPBottomScale},
        MalformedRectifiedMatrixCase{"P bottom end", malformedPBottomEnd},
        MalformedRectifiedMatrixCase{"left Tx", malformedLeftTx},
        MalformedRectifiedMatrixCase{"K principal point mismatch", malformedKPrincipalPointMismatch},
        MalformedRectifiedMatrixCase{"K vertical principal point mismatch", malformedKVerticalPrincipalPointMismatch},
        MalformedRectifiedMatrixCase{"non-finite unconsumed entry", malformedNonFiniteUnconsumedEntry}),
    [](const ::testing::TestParamInfo<MalformedRectifiedMatrixCase>& info) {
      std::string name = info.param.name;
      std::replace(name.begin(), name.end(), ' ', '_');
      std::replace(name.begin(), name.end(), '-', '_');
      return name;
    });

TEST(Calibration, AcceptsTinyRectificationNoise) {
  auto left = makeInfo();
  auto right = makeInfo();
  right.p[3] = -21.429536819458008;
  left.k[1] = 1e-12;
  left.p[8] = -1e-12;
  left.p[10] = 1.0 + 1e-12;
  EXPECT_NO_THROW(orb_slam3_wrapper::Calibration::fromCameraInfo(left, right, "left", "right"));
}
