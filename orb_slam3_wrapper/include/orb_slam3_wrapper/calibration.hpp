#pragma once

#include <string>
#include <string_view>

#include <sensor_msgs/msg/camera_info.hpp>

namespace orb_slam3_wrapper {

struct StereoCalibration {
  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;
  double baseline_m;
  std::string left_frame;
  std::string right_frame;
};

class Calibration {
public:
  static StereoCalibration fromCameraInfo(
      const sensor_msgs::msg::CameraInfo& left,
      const sensor_msgs::msg::CameraInfo& right,
      std::string_view left_image_frame,
      std::string_view right_image_frame);
};

}  // namespace orb_slam3_wrapper
