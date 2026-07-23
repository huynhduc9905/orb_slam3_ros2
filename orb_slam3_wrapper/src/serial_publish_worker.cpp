#include "orb_slam3_wrapper/serial_publish_worker.hpp"

#include <exception>
#include <utility>

namespace orb_slam3_wrapper {

SerialPublishWorker::SerialPublishWorker(bool coalesce, std::size_t max_queue,
                                         ErrorCallback error_callback)
: coalesce_(coalesce),
  max_queue_(max_queue == 0 ? 1 : max_queue),
  error_callback_(std::move(error_callback)),
  thread_(&SerialPublishWorker::run, this) {}

SerialPublishWorker::~SerialPublishWorker() { stop(); }

bool SerialPublishWorker::submit(Task task) {
  if (!task) return false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return false;
    if (coalesce_) {
      queue_.clear();
      queue_.push_back(std::move(task));
    } else {
      queue_.push_back(std::move(task));
      // Bound memory: drop the oldest pending task on overflow rather than the
      // newest, so the most recent state is preserved.
      while (queue_.size() > max_queue_) queue_.pop_front();
    }
  }
  condition_.notify_one();
  return true;
}

void SerialPublishWorker::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    stopping_ = true;
  }
  condition_.notify_one();
  if (thread_.joinable()) thread_.join();
}

void SerialPublishWorker::run() {
  for (;;) {
    Task task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (queue_.empty()) {
        if (stopping_) return;
        continue;
      }
      task = std::move(queue_.front());
      queue_.pop_front();
    }
    try {
      task();
    } catch (const std::exception& error) {
      try { if (error_callback_) error_callback_(error.what()); } catch (...) {}
    } catch (...) {
      try { if (error_callback_) error_callback_("unknown publish worker failure"); } catch (...) {}
    }
  }
}

}  // namespace orb_slam3_wrapper
