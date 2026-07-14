#include "orb_slam3_wrapper/backend.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <stdexcept>

namespace orb_slam3_wrapper {

OrbSlam3Backend::OrbSlam3Backend(const std::string& vocabulary, const std::string& settings)
: vocabulary_path_(vocabulary), settings_path_(settings) {}

OrbSlam3Backend::~OrbSlam3Backend() {
  if (system_ && !shutdown_) {
    system_->Shutdown();
    shutdown_ = true;
  }
}

ORB_SLAM3::FrameSnapshot OrbSlam3Backend::trackStereo(
    const cv::Mat& left, const cv::Mat& right, double stamp) {
  if (!system_) throw std::runtime_error("ORB backend is not configured");
  system_->TrackStereo(left, right, stamp, {});
  return system_->GetLastFrameSnapshot();
}

bool OrbSlam3Backend::configureCalibration(const StereoCalibration& calibration,
                                           std::string& error) {
  cv::FileStorage settings(settings_path_, cv::FileStorage::READ);
  if (!settings.isOpened()) { error = "cannot open ORB settings for calibration validation"; return false; }
  const auto close = [](double a, double b) { return std::isfinite(a) && std::isfinite(b) &&
      std::abs(a - b) <= 1e-6 * std::max({1.0, std::abs(a), std::abs(b)}); };
  const auto required = [&](const char* key) {
    if (settings[key].empty()) { error = std::string("missing ORB setting ") + key; return false; }
    return true;
  };
  for (const auto key : {"Camera.type", "Camera1.fx", "Camera1.fy", "Camera1.cx", "Camera1.cy",
                         "Camera2.fx", "Camera2.fy", "Camera2.cx", "Camera2.cy", "Camera.width",
                         "Camera.height", "Stereo.b", "Stereo.T_c1_c2", "Camera1.image_frame",
                         "Camera2.image_frame"}) if (!required(key)) return false;
  const auto type = static_cast<std::string>(settings["Camera.type"]);
  const auto left_frame = static_cast<std::string>(settings["Camera1.image_frame"]);
  const auto right_frame = static_cast<std::string>(settings["Camera2.image_frame"]);
  cv::Mat transform = settings["Stereo.T_c1_c2"].mat();
  bool matrix_ok = transform.rows == 4 && transform.cols == 4 && transform.type() == CV_32F;
  if (matrix_ok) {
    for (int row = 0; row < transform.rows; ++row) for (int col = 0; col < transform.cols; ++col)
      matrix_ok = matrix_ok && std::isfinite(static_cast<double>(transform.at<float>(row, col)));
    matrix_ok = matrix_ok && close(transform.at<float>(3, 0), 0.0) && close(transform.at<float>(3, 1), 0.0) &&
      close(transform.at<float>(3, 2), 0.0) && close(transform.at<float>(3, 3), 1.0);
  }
  const bool valid = type == "PinHole" && left_frame == calibration.left_frame &&
      right_frame == calibration.right_frame && left_frame != right_frame && matrix_ok &&
      static_cast<int>(settings["Camera.width"]) == calibration.width &&
      static_cast<int>(settings["Camera.height"]) == calibration.height &&
      close(static_cast<double>(settings["Camera1.fx"]), calibration.fx) &&
      close(static_cast<double>(settings["Camera1.fy"]), calibration.fy) &&
      close(static_cast<double>(settings["Camera1.cx"]), calibration.cx) &&
      close(static_cast<double>(settings["Camera1.cy"]), calibration.cy) &&
      close(static_cast<double>(settings["Camera2.fx"]), calibration.fx) &&
      close(static_cast<double>(settings["Camera2.fy"]), calibration.fy) &&
      close(static_cast<double>(settings["Camera2.cx"]), calibration.cx) &&
      close(static_cast<double>(settings["Camera2.cy"]), calibration.cy) &&
      close(static_cast<double>(settings["Stereo.b"]), calibration.baseline_m) &&
      close(cv::norm(transform(cv::Range(0, 3), cv::Range(3, 4))), calibration.baseline_m);
  if (!valid) { error = "CameraInfo does not match the complete ORB stereo settings contract"; return false; }
  try {
    system_ = std::make_unique<ORB_SLAM3::System>(vocabulary_path_, settings_path_, ORB_SLAM3::System::STEREO, false);
    shutdown_ = false;
    return true;
  } catch (const std::exception& exception) {
    error = std::string("ORB initialization failed: ") + exception.what();
  } catch (...) { error = "ORB initialization failed with an unknown error"; }
  system_.reset();
  return false;
}

bool OrbSlam3Backend::mapChanged() { return system_ && system_->MapChanged(); }

ORB_SLAM3::GraphSnapshot OrbSlam3Backend::graphSnapshot() { return system_->GetGraphSnapshot(); }

}  // namespace orb_slam3_wrapper
