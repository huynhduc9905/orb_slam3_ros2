#pragma once

#include <condition_variable>
#include <functional>
#include <string>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <std_msgs/msg/header.hpp>

namespace orb_slam3_wrapper {

class LatestImageWorker {
public:
  using Callback = std::function<void(const std::vector<unsigned char>&,
                                     const std_msgs::msg::Header&)>;
  using Encoder = std::function<bool(const cv::Mat&, std::vector<unsigned char>&)>;
  using ErrorCallback = std::function<void(const std::string&)>;
  explicit LatestImageWorker(Callback callback, Encoder encoder = {}, ErrorCallback error_callback = {});
  ~LatestImageWorker();
  bool submit(const cv::Mat& image, const std_msgs::msg::Header& header);
  void stop();

private:
  void run();
  Callback callback_;
  Encoder encoder_;
  ErrorCallback error_callback_;
  std::mutex mutex_;
  std::condition_variable condition_;
  cv::Mat latest_;
  std_msgs::msg::Header latest_header_;
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace orb_slam3_wrapper
