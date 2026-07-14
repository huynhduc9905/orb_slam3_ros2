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

void requireRectifiedPinhole(const sensor_msgs::msg::CameraInfo& info, const char* name,
                             bool require_negative_tx) {
  for (const double value : info.k) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string("non-finite stereo calibration ") + name + " K");
    }
  }
  for (const double value : info.p) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string("non-finite stereo calibration ") + name + " P");
    }
  }

  requireFinitePositive(info.k[0], "K fx");
  requireFinitePositive(info.k[4], "K fy");
  requireFinitePositive(info.p[0], "P fx");
  requireFinitePositive(info.p[5], "P fy");
  const double zero = 0.0;
  const double one = 1.0;
  requireClose(info.k[1], zero, "K[1]");
  requireClose(info.k[3], zero, "K[3]");
  requireClose(info.k[6], zero, "K[6]");
  requireClose(info.k[7], zero, "K[7]");
  requireClose(info.k[8], one, "K[8]");
  requireClose(info.p[1], zero, "P[1]");
  requireClose(info.p[4], zero, "P[4]");
  requireClose(info.p[7], zero, "P[7]");
  requireClose(info.p[8], zero, "P[8]");
  requireClose(info.p[9], zero, "P[9]");
  requireClose(info.p[10], one, "P[10]");
  requireClose(info.p[11], zero, "P[11]");
  if (require_negative_tx) {
    if (info.p[3] >= 0.0) {
      throw std::invalid_argument(std::string("invalid stereo calibration ") + name + " Tx");
    }
  } else {
    requireClose(info.p[3], zero, "left P Tx");
  }
  requireClose(info.p[0], info.k[0], "fx");
  requireClose(info.p[5], info.k[4], "fy");
  requireClose(info.p[2], info.k[2], "cx");
  requireClose(info.p[6], info.k[5], "cy");
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

  requireRectifiedPinhole(left, "left", false);
  requireRectifiedPinhole(right, "right", true);
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
