#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>

#define private public
#define protected public
#include <System.h>
#undef protected
#undef private

TEST(GlobalBundleAdjustmentShutdown, ShutdownWaitsForLiveOwnedWorker) {
  const char* vocabulary = std::getenv("ORB_TEST_VOCAB");
  const char* settings = std::getenv("ORB_TEST_SETTINGS");
  ASSERT_NE(vocabulary, nullptr);
  ASSERT_NE(settings, nullptr);

  auto system = std::make_unique<ORB_SLAM3::System>(
      vocabulary, settings, ORB_SLAM3::System::STEREO, false);
  ASSERT_NE(system->mpLoopCloser, nullptr);
  ASSERT_NE(system->mpAtlas, nullptr);

  std::mutex mutex;
  std::condition_variable condition;
  bool worker_entered = false;
  bool release_worker = false;
  ORB_SLAM3::LoopClosing::SetGlobalBundleAdjustmentStartHookForTesting([&] {
    std::unique_lock<std::mutex> lock(mutex);
    worker_entered = true;
    condition.notify_all();
    condition.wait(lock, [&] { return release_worker; });
  });

  // The test hook holds a real LoopClosing-owned GBA worker before it can
  // touch the map. Shutdown must remain blocked until that worker is released
  // and joined, rather than deleting its map dependencies underneath it.
  system->mpLoopCloser->LaunchGlobalBundleAdjustment(
      system->mpAtlas->GetCurrentMap(), 0);
  bool reached_worker_hook = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    reached_worker_hook = condition.wait_for(
        lock, std::chrono::seconds(5), [&] { return worker_entered; });
    if (!reached_worker_hook)
      release_worker = true;
  }
  if (!reached_worker_hook) {
    condition.notify_all();
    system->Shutdown();
    ORB_SLAM3::LoopClosing::SetGlobalBundleAdjustmentStartHookForTesting({});
    FAIL() << "GBA worker did not reach the test hook";
  }

  std::atomic<bool> shutdown_finished{false};
  std::thread shutdown_thread([&] {
    system->Shutdown();
    shutdown_finished.store(true);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(shutdown_finished.load());

  {
    std::lock_guard<std::mutex> lock(mutex);
    release_worker = true;
  }
  condition.notify_all();
  shutdown_thread.join();

  EXPECT_TRUE(shutdown_finished.load());
  ORB_SLAM3::LoopClosing::SetGlobalBundleAdjustmentStartHookForTesting({});
}
