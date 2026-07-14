#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/map_rebuilder.hpp"

namespace orb_lidar_mapper {
namespace {

using namespace std::chrono_literals;

class Gate {
 public:
  void arrive() { std::lock_guard<std::mutex> lock(mutex_); arrived_ = true; cv_.notify_all(); }
  void wait() { std::unique_lock<std::mutex> lock(mutex_); ASSERT_TRUE(cv_.wait_for(lock, 2s, [this] { return arrived_; })); }
  void release() { std::lock_guard<std::mutex> lock(mutex_); released_ = true; cv_.notify_all(); }
  void block() { std::unique_lock<std::mutex> lock(mutex_); cv_.wait(lock, [this] { return released_; }); }
 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool arrived_{};
  bool released_{};
};

ArchivedScan scan(std::uint64_t id, double endpoint = 1.0) {
  return {id, static_cast<std::int64_t>(id), {{{0.0, 0.0}, {endpoint, 0.0}, true}}};
}

ScanPose pose(std::uint64_t id, double x = 0.0, bool committed = true) {
  return {id, static_cast<std::int64_t>(id), {x, 0.0, 0.0}, committed, 0};
}

std::shared_ptr<const TrajectoryRevision> trajectory(std::uint64_t revision,
                                                     std::vector<ScanPose> scans) {
  auto value = std::make_shared<TrajectoryRevision>();
  value->graph_revision = revision;
  value->scans = std::move(scans);
  return value;
}

std::shared_ptr<const ScanArchive> archive(std::vector<ArchivedScan> scans) {
  auto value = std::make_shared<ScanArchive>();
  value->scans = std::move(scans);
  return value;
}

TEST(MapRebuilder, IncrementalAppendPublishesTransformedCommittedScanWithoutArchiveReplay) {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::shared_ptr<const MapSnapshot>> published;
  GridConfig config; config.resolution_m = 1.0;
  MapRebuilder rebuilder(config, [&](auto snapshot, const RebuildStatus&) {
    std::lock_guard<std::mutex> lock(mutex); published.push_back(std::move(snapshot)); cv.notify_all();
  });

  rebuilder.appendCommitted(scan(1), pose(1, 4.0), 7);
  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return published.size() == 1; }));
  const auto first = published.front();
  EXPECT_EQ(first->graph_revision, 7U);
  EXPECT_EQ(first->map_revision, 1U);
  EXPECT_EQ(first->committed_scan_count, 1U);
  EXPECT_EQ(first->grid.cellAt(5, 0), 100);
  EXPECT_EQ(rebuilder.current(), first);
}

TEST(MapRebuilder, KeepsPreviousSnapshotAndCoalescesToNewestRebuild) {
  Gate gate;
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::uint64_t> published;
  GridConfig config; config.resolution_m = 1.0;
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 2) { gate.arrive(); gate.block(); }
  };
  MapRebuilder rebuilder(config, [&](auto snapshot, const RebuildStatus& status) {
    std::lock_guard<std::mutex> lock(mutex);
    if (status.state == RebuildState::kPublished) published.push_back(snapshot->graph_revision);
    cv.notify_all();
  }, hooks);

  rebuilder.requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  { std::unique_lock<std::mutex> lock(mutex); ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return published.size() == 1; })); }
  const auto old = rebuilder.current();
  rebuilder.requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
  gate.wait();
  EXPECT_EQ(rebuilder.current(), old);
  rebuilder.requestRebuild(trajectory(3, {pose(3, 10.0)}), archive({scan(3)}));
  gate.release();
  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return published.size() == 2; }));
  EXPECT_EQ(published, (std::vector<std::uint64_t>{1, 3}));
  EXPECT_EQ(rebuilder.current()->graph_revision, 3U);
  EXPECT_EQ(rebuilder.current()->grid.cellAt(11, 0), 100);
}

TEST(MapRebuilder, FailurePreservesSnapshotAndWorkerRecovers) {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<RebuildStatus> statuses;
  bool fail = true;
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 2 && fail) throw std::runtime_error("injected");
  };
  MapRebuilder rebuilder({}, [&](auto, const RebuildStatus& status) {
    std::lock_guard<std::mutex> lock(mutex); statuses.push_back(status); cv.notify_all();
  }, hooks);
  rebuilder.requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  { std::unique_lock<std::mutex> lock(mutex); ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return statuses.size() == 1; })); }
  const auto old = rebuilder.current();
  rebuilder.requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
  { std::unique_lock<std::mutex> lock(mutex); ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return statuses.size() == 2; })); }
  EXPECT_EQ(statuses.back().state, RebuildState::kFailed);
  EXPECT_EQ(rebuilder.current(), old);
  fail = false;
  rebuilder.requestRebuild(trajectory(3, {pose(3)}), archive({scan(3)}));
  { std::unique_lock<std::mutex> lock(mutex); ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return statuses.size() == 3; })); }
  EXPECT_EQ(rebuilder.current()->graph_revision, 3U);
}

TEST(MapRebuilder, RebuildSupersedesQueuedIncrementalsAndOldSnapshotsStayImmutable) {
  Gate gate;
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::shared_ptr<const MapSnapshot>> snapshots;
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 2) { gate.arrive(); gate.block(); }
  };
  GridConfig config; config.resolution_m = 1.0;
  MapRebuilder rebuilder(config, [&](auto snapshot, const RebuildStatus& status) {
    if (status.state == RebuildState::kPublished) { std::lock_guard<std::mutex> lock(mutex); snapshots.push_back(snapshot); cv.notify_all(); }
  }, hooks);
  rebuilder.appendCommitted(scan(1), pose(1), 1);
  { std::unique_lock<std::mutex> lock(mutex); ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return snapshots.size() == 1; })); }
  const auto old = snapshots.front();
  rebuilder.requestRebuild(trajectory(2, {pose(1), pose(2)}), archive({scan(1), scan(2, 2.0)}));
  gate.wait();
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 2);
  gate.release();
  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return snapshots.size() == 2; }));
  EXPECT_EQ(snapshots.back()->committed_scan_count, 2U);
  EXPECT_EQ(old->committed_scan_count, 1U);
  EXPECT_EQ(old->grid.cellAt(2, 0), -1);
  EXPECT_FALSE(cv.wait_for(lock, 100ms, [&] { return snapshots.size() > 2; }));
}

TEST(MapRebuilder, CallbackMayCallCurrentAndDestructorStopsBlockedWorker) {
  Gate gate;
  std::mutex mutex;
  std::condition_variable cv;
  bool callback_current{};
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t, std::uint64_t) { gate.arrive(); gate.block(); };
  MapRebuilder* raw = nullptr;
  auto rebuilder = std::make_unique<MapRebuilder>(GridConfig{}, [&](auto snapshot, const RebuildStatus&) {
    callback_current = snapshot && raw && raw->current() == snapshot;
    std::lock_guard<std::mutex> lock(mutex); cv.notify_all();
  }, hooks);
  raw = rebuilder.get();
  rebuilder->requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  gate.wait();
  gate.release();
  { std::unique_lock<std::mutex> lock(mutex); ASSERT_TRUE(cv.wait_for(lock, 2s, [&] { return callback_current; })); }
  rebuilder.reset();
}

}  // namespace
}  // namespace orb_lidar_mapper
