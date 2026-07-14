#include <chrono>
#include <atomic>
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

class Recorder {
 public:
  void record(std::shared_ptr<const MapSnapshot> snapshot, const RebuildStatus& status) {
    std::lock_guard<std::mutex> lock(mutex);
    snapshots.push_back(std::move(snapshot));
    statuses.push_back(status);
    cv.notify_all();
  }
  bool waitFor(std::size_t count) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 2s, [&] { return statuses.size() >= count; });
  }
  bool waitForPublishedRevision(std::uint64_t revision) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 2s, [&] {
      for (const RebuildStatus& status : statuses)
        if (status.state == RebuildState::kPublished && status.graph_revision == revision) return true;
      return false;
    });
  }
  bool saw(RebuildState state, std::uint64_t revision) const {
    std::lock_guard<std::mutex> lock(mutex);
    for (const RebuildStatus& status : statuses)
      if (status.state == state && status.graph_revision == revision) return true;
    return false;
  }
  std::size_t published(std::uint64_t revision) const {
    std::lock_guard<std::mutex> lock(mutex);
    std::size_t result{};
    for (const RebuildStatus& status : statuses)
      if (status.state == RebuildState::kPublished && status.graph_revision == revision) ++result;
    return result;
  }
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::shared_ptr<const MapSnapshot>> snapshots;
  std::vector<RebuildStatus> statuses;
};

TEST(MapRebuilder, IncrementalAppendPublishesImmutableSnapshot) {
  Recorder recorder;
  GridConfig config; config.resolution_m = 1.0;
  MapRebuilder rebuilder(config, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); });

  rebuilder.appendCommitted(scan(1), pose(1, 4.0), 7);
  ASSERT_TRUE(recorder.waitForPublishedRevision(7));
  const auto first = rebuilder.current();
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->map_revision, 1U);
  EXPECT_EQ(first->committed_scan_count, 1U);
  EXPECT_EQ(first->grid.cellAt(5, 0), 100);
  EXPECT_TRUE(recorder.saw(RebuildState::kIdle, 7));
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 7);
  ASSERT_TRUE(recorder.waitFor(4));
  EXPECT_EQ(first->committed_scan_count, 1U);
  EXPECT_EQ(first->grid.cellAt(2, 0), -1);
}

TEST(MapRebuilder, NewerRequestAtFinalGatePreventsObsoletePublication) {
  Gate final_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_full_commit = [&](std::uint64_t revision) {
    if (revision == 2) { final_gate.arrive(); final_gate.block(); }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
  final_gate.wait();
  rebuilder.requestRebuild(trajectory(3, {pose(3, 10.0)}), archive({scan(3)}));
  final_gate.release();
  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  EXPECT_EQ(recorder.published(2), 0U);
  EXPECT_EQ(rebuilder.current()->graph_revision, 3U);
}

TEST(MapRebuilder, FailedRepresentedRebuildRetainsIncrementalForRecovery) {
  Gate gate;
  Recorder recorder;
  bool fail = true;
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 2) { gate.arrive(); gate.block(); if (fail) throw std::runtime_error("injected"); }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
  gate.wait();
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 2);
  gate.release();
  ASSERT_TRUE(recorder.waitFor(5));
  ASSERT_TRUE(recorder.saw(RebuildState::kFailed, 2));
  fail = false;
  rebuilder.requestRebuild(trajectory(3, {pose(1), pose(2)}), archive({scan(1), scan(2, 2.0)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 2U);
}

TEST(MapRebuilder, FullRequestSuppressesInFlightRepresentedIncremental) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) { incremental_gate.arrive(); incremental_gate.block(); }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 2);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(3, {pose(1), pose(2)}), archive({scan(1), scan(2, 2.0)}));
  incremental_gate.release();
  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  EXPECT_EQ(recorder.published(2), 0U);
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 2U);
}

TEST(MapRebuilder, InvalidArchiveOrTrajectoryIdsFailWithoutChangingCurrent) {
  Recorder recorder;
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); });
  rebuilder.requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  const auto current = rebuilder.current();
  rebuilder.requestRebuild(trajectory(2, {pose(2), pose(2)}), archive({scan(2), scan(2)}));
  ASSERT_TRUE(recorder.waitFor(5));
  EXPECT_TRUE(recorder.saw(RebuildState::kFailed, 2));
  EXPECT_EQ(rebuilder.current(), current);
  rebuilder.requestRebuild(trajectory(3, {pose(3)}), archive({scan(4)}));
  ASSERT_TRUE(recorder.waitFor(7));
  EXPECT_TRUE(recorder.saw(RebuildState::kFailed, 3));
  EXPECT_EQ(rebuilder.current(), current);
}

TEST(MapRebuilder, StaleGraphRequestsAndDuplicateIncrementsCannotReplaceNewerMap) {
  Recorder recorder;
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); });
  rebuilder.requestRebuild(trajectory(3, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  rebuilder.requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
  rebuilder.requestRebuild(trajectory(3, {pose(3)}), archive({scan(3)}));
  rebuilder.appendCommitted(scan(1), pose(1), 3);
  rebuilder.appendCommitted(scan(2), pose(2), 2);
  rebuilder.requestRebuild(trajectory(4, {pose(4)}), archive({scan(4)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(4));
  EXPECT_EQ(recorder.published(2), 0U);
  EXPECT_EQ(recorder.published(3), 1U);
  EXPECT_EQ(rebuilder.current()->graph_revision, 4U);
}

TEST(MapRebuilder, CallbackExceptionsAndCallbackDestructionLeaveWorkerSafe) {
  Recorder recorder;
  std::unique_ptr<MapRebuilder> rebuilder;
  std::atomic<bool> destroyed{};
  rebuilder = std::make_unique<MapRebuilder>(GridConfig{}, [&](auto snapshot, const RebuildStatus& status) {
    recorder.record(snapshot, status);
    if (status.state == RebuildState::kPublished && status.graph_revision == 1) {
      EXPECT_EQ(rebuilder->current(), snapshot);
      rebuilder.reset();
      destroyed = true;
      throw std::runtime_error("callback");
    }
  });
  rebuilder->requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  EXPECT_TRUE(destroyed.load());
}

TEST(MapRebuilder, ConstructorStressAndStatusFieldsAreConsistent) {
  for (int i = 0; i != 100; ++i) {
    Recorder recorder;
    MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); });
    rebuilder.requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
    ASSERT_TRUE(recorder.waitForPublishedRevision(1));
    std::lock_guard<std::mutex> lock(recorder.mutex);
    for (const RebuildStatus& status : recorder.statuses) {
      if (status.state == RebuildState::kPublished) {
        EXPECT_EQ(status.graph_revision, 1U);
        EXPECT_EQ(status.input_scan_count, 1U);
        EXPECT_EQ(status.committed_scan_count, 1U);
        EXPECT_EQ(status.map_revision, 1U);
      }
    }
  }
}

}  // namespace
}  // namespace orb_lidar_mapper
