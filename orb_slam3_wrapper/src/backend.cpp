#include "orb_slam3_wrapper/backend.hpp"

namespace orb_slam3_wrapper {

OrbSlam3Backend::OrbSlam3Backend(const std::string& vocabulary, const std::string& settings)
: system_(std::make_unique<ORB_SLAM3::System>(vocabulary, settings, ORB_SLAM3::System::STEREO, false)) {}

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

bool OrbSlam3Backend::mapChanged() { return system_->MapChanged(); }

ORB_SLAM3::GraphSnapshot OrbSlam3Backend::graphSnapshot() { return system_->GetGraphSnapshot(); }

}  // namespace orb_slam3_wrapper
