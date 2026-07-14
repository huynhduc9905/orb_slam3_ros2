#include "orb_slam3_wrapper/latest_image_worker.hpp"

#include <opencv2/imgcodecs.hpp>
#include <exception>
#include <string>

namespace orb_slam3_wrapper {

LatestImageWorker::LatestImageWorker(Callback callback, Encoder encoder, ErrorCallback error_callback)
: callback_(std::move(callback)),
  encoder_(encoder ? std::move(encoder) : Encoder([](const cv::Mat& image, std::vector<unsigned char>& encoded) {
    return cv::imencode(".jpg", image, encoded);
  })), error_callback_(std::move(error_callback)), thread_(&LatestImageWorker::run, this) {}

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
    try {
      std::vector<unsigned char> encoded;
      if (!encoder_(image, encoded)) throw std::runtime_error("JPEG encoding failed");
      callback_(encoded, header);
    } catch (const std::exception& error) {
      try { if (error_callback_) error_callback_(error.what()); } catch (...) {}
    } catch (...) {
      try { if (error_callback_) error_callback_("unknown image worker failure"); } catch (...) {}
    }
  }
}

}  // namespace orb_slam3_wrapper
