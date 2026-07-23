#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace orb_slam3_wrapper {

// Runs publish() calls on a dedicated background thread so a slow, RELIABLE-QoS
// subscriber can never block the caller (the ORB tracking / graph-timer spin
// thread). Two modes:
//   - coalesce=true : keep only the most recent task (latest-wins). Correct for
//     state-like streams (graph_snapshot, path, marker arrays) where a slow
//     reader only needs the newest value; intermediate tasks are dropped.
//   - coalesce=false: FIFO up to max_queue tasks, dropping the OLDEST on
//     overflow. Correct for discrete, ordered messages (tracking events) that
//     must not be silently collapsed.
// The worker owns nothing but the tasks handed to it; each publisher must be
// driven by exactly one worker (never concurrently from another thread) so no
// publisher-level locking is required here.
class SerialPublishWorker {
public:
  using Task = std::function<void()>;
  using ErrorCallback = std::function<void(const std::string&)>;

  SerialPublishWorker(bool coalesce, std::size_t max_queue,
                      ErrorCallback error_callback = {});
  ~SerialPublishWorker();

  SerialPublishWorker(const SerialPublishWorker&) = delete;
  SerialPublishWorker& operator=(const SerialPublishWorker&) = delete;

  // Enqueue a publish task. Returns false if the worker is stopping.
  bool submit(Task task);
  void stop();

private:
  void run();

  const bool coalesce_;
  const std::size_t max_queue_;
  ErrorCallback error_callback_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Task> queue_;
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace orb_slam3_wrapper
