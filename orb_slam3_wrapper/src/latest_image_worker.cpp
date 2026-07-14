#include "orb_slam3_wrapper/latest_image_worker.hpp"

#include <opencv2/imgcodecs.hpp>

namespace orb_slam3_wrapper {

LatestImageWorker::LatestImageWorker(Callback callback)
: callback_(std::move(callback)), thread_(&LatestImageWorker::run, this) {}

LatestImageWorker::~LatestImageWorker() { stop(); }

bool LatestImageWorker::submit(const cv::Mat& image, const std_msgs::msg::Header& header) {
  std::lock_guard lock(mutex_);
  if (stopping_) return false;
  latest_ = image.clone();
  latest_header_ = header;
  condition_.notify_one();
  return true;
}

void LatestImageWorker::stop() {
  {
    std::lock_guard lock(mutex_);
    if (stopping_) return;
    stopping_ = true;
  }
  condition_.notify_one();
  if (thread_.joinable()) thread_.join();
}

void LatestImageWorker::run() {
  for (;;) {
    cv::Mat image;
    std_msgs::msg::Header header;
    {
      std::unique_lock lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || !latest_.empty(); });
      if (latest_.empty() && stopping_) return;
      image = std::move(latest_);
      header = latest_header_;
    }
    std::vector<unsigned char> encoded;
    cv::imencode(".jpg", image, encoded);
    callback_(encoded, header);
  }
}

}  // namespace orb_slam3_wrapper
