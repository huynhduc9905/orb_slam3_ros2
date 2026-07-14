#pragma once

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include <System.h>

namespace orb_slam3_wrapper {

class SlamBackend {
public:
  virtual ~SlamBackend() = default;
  virtual ORB_SLAM3::FrameSnapshot trackStereo(
      const cv::Mat& left, const cv::Mat& right, double stamp) = 0;
  virtual bool mapChanged() = 0;
  virtual ORB_SLAM3::GraphSnapshot graphSnapshot() = 0;
};

class OrbSlam3Backend final : public SlamBackend {
public:
  OrbSlam3Backend(const std::string& vocabulary, const std::string& settings);
  ~OrbSlam3Backend() override;
  ORB_SLAM3::FrameSnapshot trackStereo(
      const cv::Mat& left, const cv::Mat& right, double stamp) override;
  bool mapChanged() override;
  ORB_SLAM3::GraphSnapshot graphSnapshot() override;

private:
  std::unique_ptr<ORB_SLAM3::System> system_;
  bool shutdown_{false};
};

}  // namespace orb_slam3_wrapper
