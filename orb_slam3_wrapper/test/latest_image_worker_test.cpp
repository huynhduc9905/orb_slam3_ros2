#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <vector>

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
