#pragma once

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include <System.h>

#include "orb_slam3_wrapper/calibration.hpp"

namespace orb_slam3_wrapper {

bool validRectifiedStereoTransform(const cv::Mat& transform, double baseline_m);

class SlamBackend {
public:
  virtual ~SlamBackend() = default;
  virtual bool configureCalibration(const StereoCalibration& calibration,
                                    std::string& error) = 0;
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
  bool configureCalibration(const StereoCalibration& calibration,
                            std::string& error) override;
  bool mapChanged() override;
  ORB_SLAM3::GraphSnapshot graphSnapshot() override;

private:
  std::string vocabulary_path_;
  std::unique_ptr<ORB_SLAM3::System> system_;
  bool shutdown_{false};
  std::string settings_path_;
};

}  // namespace orb_slam3_wrapper
