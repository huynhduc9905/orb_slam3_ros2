#include "orb_slam3_wrapper/backend.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>

namespace orb_slam3_wrapper {

OrbSlam3Backend::OrbSlam3Backend(const std::string& vocabulary, const std::string& settings)
: system_(std::make_unique<ORB_SLAM3::System>(vocabulary, settings, ORB_SLAM3::System::STEREO, false)),
  settings_path_(settings) {}

OrbSlam3Backend::~OrbSlam3Backend() {
  if (system_ && !shutdown_) {
    system_->Shutdown();
    shutdown_ = true;
  }
}

ORB_SLAM3::FrameSnapshot OrbSlam3Backend::trackStereo(
    const cv::Mat& left, const cv::Mat& right, double stamp) {
  system_->TrackStereo(left, right, stamp, {});
  return system_->GetLastFrameSnapshot();
}

bool OrbSlam3Backend::configureCalibration(const StereoCalibration& calibration,
                                           std::string& error) {
  if (calibration_attempted_) {
    error = "ORB calibration validation was already attempted";
    return false;
  }
  calibration_attempted_ = true;
  cv::FileStorage settings(settings_path_, cv::FileStorage::READ);
  if (!settings.isOpened()) { error = "cannot open ORB settings for calibration validation"; return false; }
  const auto close = [](double a, double b) { return std::isfinite(a) && std::isfinite(b) &&
      std::abs(a - b) <= 1e-6 * std::max({1.0, std::abs(a), std::abs(b)}); };
  const bool valid = static_cast<int>(settings["Camera.width"]) == calibration.width &&
      static_cast<int>(settings["Camera.height"]) == calibration.height &&
      close(static_cast<double>(settings["Camera1.fx"]), calibration.fx) &&
      close(static_cast<double>(settings["Camera1.fy"]), calibration.fy) &&
      close(static_cast<double>(settings["Camera1.cx"]), calibration.cx) &&
      close(static_cast<double>(settings["Camera1.cy"]), calibration.cy) &&
      close(static_cast<double>(settings["Stereo.b"]), calibration.baseline_m);
  if (!valid) error = "CameraInfo does not match ORB settings";
  return valid;
}

bool OrbSlam3Backend::mapChanged() { return system_->MapChanged(); }

ORB_SLAM3::GraphSnapshot OrbSlam3Backend::graphSnapshot() { return system_->GetGraphSnapshot(); }

}  // namespace orb_slam3_wrapper
