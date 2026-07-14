#include "orb_slam3_wrapper/calibration.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace orb_slam3_wrapper {
namespace {

void requireFinitePositive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string("invalid stereo calibration ") + name);
  }
}

void requireClose(double left, double right, const char* name) {
  const double scale = std::max({1.0, std::abs(left), std::abs(right)});
  if (!std::isfinite(left) || !std::isfinite(right) || std::abs(left - right) > 1e-9 * scale) {
    throw std::invalid_argument(std::string("inconsistent stereo calibration ") + name);
  }
}

}  // namespace

StereoCalibration Calibration::fromCameraInfo(
    const sensor_msgs::msg::CameraInfo& left,
    const sensor_msgs::msg::CameraInfo& right,
    std::string_view left_image_frame,
    std::string_view right_image_frame) {
  if (left.width == 0 || left.height == 0 || left.width != right.width || left.height != right.height) {
    throw std::invalid_argument("incompatible stereo calibration image dimensions");
  }
  if (left_image_frame.empty() || right_image_frame.empty()) {
    throw std::invalid_argument("stereo image frame IDs must not be empty");
  }

  requireFinitePositive(left.p[0], "left fx");
  requireFinitePositive(left.p[5], "left fy");
  requireFinitePositive(right.p[0], "right fx");
  requireFinitePositive(right.p[5], "right fy");
  requireFinitePositive(left.k[0], "left K fx");
  requireFinitePositive(left.k[4], "left K fy");
  requireFinitePositive(right.k[0], "right K fx");
  requireFinitePositive(right.k[4], "right K fy");
  for (const double value : {left.p[2], left.p[6], right.p[2], right.p[6],
                             left.k[2], left.k[5], right.k[2], right.k[5]}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("non-finite stereo calibration principal point");
    }
  }
  requireClose(left.p[0], left.k[0], "left fx");
  requireClose(left.p[5], left.k[4], "left fy");
  requireClose(right.p[0], right.k[0], "right fx");
  requireClose(right.p[5], right.k[4], "right fy");
  requireClose(left.p[0], right.p[0], "fx");
  requireClose(left.p[5], right.p[5], "fy");
  requireClose(left.p[2], right.p[2], "cx");
  requireClose(left.p[6], right.p[6], "cy");

  const double baseline = -right.p[3] / right.p[0];
  requireFinitePositive(baseline, "baseline");
  return {static_cast<int>(left.width), static_cast<int>(left.height), left.p[0], left.p[5],
          left.p[2], left.p[6], baseline, std::string(left_image_frame), std::string(right_image_frame)};
}

}  // namespace orb_slam3_wrapper
