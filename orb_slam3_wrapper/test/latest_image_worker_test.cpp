#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <stdexcept>

#include <opencv2/core/mat.hpp>
#include <orb_slam3_wrapper/latest_image_worker.hpp>

TEST(LatestImageWorker, DropsStaleFramesAndEncodesOffSubmittingThread) {
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<int> values;
  std::atomic<bool> callback_on_submitter{false};
  const auto submitter = std::this_thread::get_id();
  orb_slam3_wrapper::LatestImageWorker worker([&](const std::vector<unsigned char>& encoded,
                                                  const std_msgs::msg::Header&) {
    if (std::this_thread::get_id() == submitter) callback_on_submitter = true;
    std::lock_guard lock(mutex);
    values.push_back(encoded.empty() ? 0 : 1);
    condition.notify_all();
  });
  std_msgs::msg::Header header;
  cv::Mat first(1200, 1200, CV_8UC1, cv::Scalar(1));
  cv::Mat second(2, 2, CV_8UC1, cv::Scalar(2));
  ASSERT_TRUE(worker.submit(first, header));
  ASSERT_TRUE(worker.submit(second, header));
  std::unique_lock lock(mutex);
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&] { return !values.empty(); }));
  worker.stop();
  EXPECT_FALSE(callback_on_submitter.load());
  EXPECT_EQ(values.size(), 1u);
}

TEST(LatestImageWorker, EncoderFailureIsReportedAndWorkerProcessesNextFrame) {
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<int> processed;
  std::vector<std::string> errors;
  int encodes = 0;
  orb_slam3_wrapper::LatestImageWorker worker(
      [&](const std::vector<unsigned char>&, const std_msgs::msg::Header&) {
        std::lock_guard lock(mutex); processed.push_back(1); condition.notify_all();
      },
      [&](const cv::Mat&, std::vector<unsigned char>&) {
        if (++encodes == 1) throw std::runtime_error("encoder failed");
        return true;
      },
      [&](const std::string& error) {
        std::lock_guard lock(mutex); errors.push_back(error); condition.notify_all();
      });
  std_msgs::msg::Header header;
  ASSERT_TRUE(worker.submit(cv::Mat(2, 2, CV_8UC1), header));
  std::unique_lock lock(mutex);
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&] { return errors.size() == 1; }));
  lock.unlock();
  ASSERT_TRUE(worker.submit(cv::Mat(2, 2, CV_8UC1), header));
  lock.lock();
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&] { return processed.size() == 1; }));
  worker.stop();
  EXPECT_EQ(errors.size(), 1u);
  EXPECT_EQ(processed.size(), 1u);
}

TEST(LatestImageWorker, PublisherFailureDoesNotTerminateWorker) {
  std::mutex mutex;
  std::condition_variable condition;
  int callbacks = 0;
  int errors = 0;
  orb_slam3_wrapper::LatestImageWorker worker(
      [&](const std::vector<unsigned char>&, const std_msgs::msg::Header&) {
        if (++callbacks == 1) throw std::runtime_error("publisher failed");
        condition.notify_all();
      },
      [](const cv::Mat&, std::vector<unsigned char>& encoded) { encoded = {1}; return true; },
      [&](const std::string&) { std::lock_guard lock(mutex); ++errors; condition.notify_all(); });
  std_msgs::msg::Header header;
  ASSERT_TRUE(worker.submit(cv::Mat(2, 2, CV_8UC1), header));
  std::unique_lock lock(mutex);
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&] { return errors == 1; }));
  lock.unlock();
  ASSERT_TRUE(worker.submit(cv::Mat(2, 2, CV_8UC1), header));
  lock.lock();
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&] { return callbacks == 2; }));
  worker.stop();
  EXPECT_EQ(errors, 1);
}
