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
  bool waitForPublishedCommittedCount(std::uint64_t count) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 2s, [&] {
      for (const RebuildStatus& status : statuses) {
        if (status.state == RebuildState::kPublished && status.committed_scan_count >= count) return true;
      }
      return false;
    });
  }
  bool waitForStatus(RebuildState state, std::uint64_t revision) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 2s, [&] {
      for (const RebuildStatus& status : statuses)
        if (status.state == state && status.graph_revision == revision) return true;
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
  std::vector<RebuildStatus> copyStatuses() const {
    std::lock_guard<std::mutex> lock(mutex);
    return statuses;
  }
  std::size_t statusIndex(RebuildState state, std::uint64_t revision) const {
    std::lock_guard<std::mutex> lock(mutex);
    for (std::size_t i = 0; i < statuses.size(); ++i)
      if (statuses[i].state == state && statuses[i].graph_revision == revision) return i;
    return statuses.size();
  }
  std::size_t statusIndex(RebuildState state, std::uint64_t revision, std::size_t occurrence) const {
    std::lock_guard<std::mutex> lock(mutex);
    std::size_t seen{};
    for (std::size_t i = 0; i < statuses.size(); ++i) {
      if (statuses[i].state == state && statuses[i].graph_revision == revision && ++seen == occurrence)
        return i;
    }
    return statuses.size();
  }
  bool sawDetail(RebuildState state, std::uint64_t revision, const std::string& detail) const {
    std::lock_guard<std::mutex> lock(mutex);
    for (const RebuildStatus& status : statuses)
      if (status.state == state && status.graph_revision == revision && status.detail == detail) return true;
    return false;
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
  EXPECT_TRUE(recorder.waitForStatus(RebuildState::kIdle, 7));
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 7);
  ASSERT_TRUE(recorder.waitFor(4));
  EXPECT_EQ(first->committed_scan_count, 1U);
  EXPECT_EQ(first->grid.cellAt(2, 0), -1);
}

TEST(MapRebuilder, ReadyIncrementalsCoalesceIntoOneAtomicPublication) {
  Gate full_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 1) {
      full_gate.arrive();
      full_gate.block();
    }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  full_gate.wait();
  rebuilder.appendCommitted(scan(2), pose(2), 1);
  rebuilder.appendCommitted(scan(3), pose(3), 1);
  rebuilder.appendCommitted(scan(4), pose(4), 1);
  full_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedCommittedCount(4));
  ASSERT_EQ(rebuilder.current()->committed_scan_count, 4U);
  EXPECT_EQ(recorder.published(1), 2U);
  const auto statuses = recorder.copyStatuses();
  bool saw_batched_publication{};
  for (const RebuildStatus& status : statuses) {
    if (status.state == RebuildState::kPublished && status.graph_revision == 1 &&
        status.input_scan_count == 3 && status.committed_scan_count == 4) {
      saw_batched_publication = true;
    }
  }
  EXPECT_TRUE(saw_batched_publication);
}

TEST(MapRebuilder, PendingFullRebuildPreemptsDeferredIncrementalBatch) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) {
      incremental_gate.arrive();
      incremental_gate.block();
    }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  ASSERT_TRUE(recorder.waitForStatus(RebuildState::kIdle, 1));
  const std::size_t before_full = recorder.copyStatuses().size();
  rebuilder.appendCommitted(scan(2), pose(2), 1);
  incremental_gate.wait();
  rebuilder.appendCommitted(scan(3), pose(3), 1);
  rebuilder.appendCommitted(scan(4), pose(4), 1);
  rebuilder.requestRebuild(trajectory(2, {pose(1)}), archive({scan(1)}));
  incremental_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedRevision(2));
  const auto statuses = recorder.copyStatuses();
  const std::size_t building = recorder.statusIndex(RebuildState::kBuilding, 2);
  ASSERT_LT(building, statuses.size());
  for (std::size_t i = before_full; i < building; ++i) {
    EXPECT_NE(statuses[i].state, RebuildState::kPublished);
  }
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
  std::atomic<bool> fail{true};
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 2) { gate.arrive(); gate.block(); if (fail.load()) throw std::runtime_error("injected"); }
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
  fail.store(false);
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
  ASSERT_TRUE(recorder.waitFor(2));
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 2);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(3, {pose(1), pose(2)}), archive({scan(1), scan(2, 2.0)}));
  incremental_gate.release();
  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  EXPECT_EQ(recorder.published(2), 0U);
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 2U);
}

TEST(MapRebuilder, FailedRepresentedFullRequeuesInFlightIncrementalExactlyOnce) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) { incremental_gate.arrive(); incremental_gate.block(); }
  };
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 2) throw std::runtime_error("injected");
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  ASSERT_TRUE(recorder.waitFor(2));
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 2);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(2, {pose(1), pose(2)}), archive({scan(1), scan(2, 2.0)}));
  incremental_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedRevision(2));
  EXPECT_TRUE(recorder.saw(RebuildState::kFailed, 2));
  EXPECT_EQ(recorder.published(2), 1U);
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 2U);
}

TEST(MapRebuilder, CancelledRepresentedFullRequeuesInFlightIncrementalExactlyOnce) {
  Gate incremental_gate;
  Gate full_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) { incremental_gate.arrive(); incremental_gate.block(); }
  };
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 2) { full_gate.arrive(); full_gate.block(); }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  ASSERT_TRUE(recorder.waitFor(2));
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 2);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(2, {pose(1), pose(2)}), archive({scan(1), scan(2, 2.0)}));
  incremental_gate.release();
  full_gate.wait();
  rebuilder.requestRebuild(trajectory(3, {pose(1)}), archive({scan(1)}));
  full_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  ASSERT_TRUE(recorder.waitForStatus(RebuildState::kFailed, 3));
  EXPECT_EQ(recorder.published(3), 1U);
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 1U);
  EXPECT_TRUE(recorder.sawDetail(RebuildState::kFailed, 3,
                                 "incremental update stale after full rebuild"));
}

TEST(MapRebuilder, DeferredNonrepresentedIncrementalNeverPublishesAfterNewerFull) {
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
  rebuilder.requestRebuild(trajectory(3, {pose(1)}), archive({scan(1)}));
  incremental_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  ASSERT_TRUE(recorder.waitFor(5));
  EXPECT_EQ(recorder.published(3), 1U);
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 1U);
  EXPECT_TRUE(recorder.sawDetail(RebuildState::kFailed, 3,
                                 "incremental update stale after full rebuild"));
}

TEST(MapRebuilder, FailedDeferredNonrepresentedIncrementalRetriesWhenFullDoesNotCommit) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) { incremental_gate.arrive(); incremental_gate.block(); }
  };
  hooks.before_rebuild_scan = [](std::uint64_t revision, std::uint64_t) {
    if (revision == 3) throw std::runtime_error("injected");
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.appendCommitted(scan(2, 2.0), pose(2), 2);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(3, {pose(1)}), archive({scan(1)}));
  incremental_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedRevision(2));
  EXPECT_TRUE(recorder.saw(RebuildState::kFailed, 3));
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 2U);
}

TEST(MapRebuilder, ObsoleteFailureIsSuppressedByNewerPendingFull) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) { incremental_gate.arrive(); incremental_gate.block(); }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  ASSERT_TRUE(recorder.waitFor(2));
  const std::size_t prior_statuses = recorder.copyStatuses().size();
  rebuilder.appendCommitted(scan(2), pose(2), 1);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(1, {pose(3)}), archive({scan(4)}));
  rebuilder.requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
  incremental_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedRevision(2));
  EXPECT_FALSE(recorder.saw(RebuildState::kFailed, 1));
  const auto statuses = recorder.copyStatuses();
  for (std::size_t i = prior_statuses; i < statuses.size(); ++i) {
    const RebuildStatus& status = statuses[i];
    if (status.graph_revision == 1)
      EXPECT_NE(status.state, RebuildState::kIdle);
  }
}

TEST(MapRebuilder, EligibleFailurePrecedesSustainedIncrementals) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) { incremental_gate.arrive(); incremental_gate.block(); }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.appendCommitted(scan(2), pose(2), 1);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(1, {pose(3)}), archive({scan(4)}));
  for (std::uint64_t id = 3; id != 12; ++id) rebuilder.appendCommitted(scan(id), pose(id), 1);
  incremental_gate.release();

  ASSERT_TRUE(recorder.waitForStatus(RebuildState::kFailed, 1));
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  const std::size_t failed = recorder.statusIndex(RebuildState::kFailed, 1);
  const std::size_t third_publish = recorder.statusIndex(RebuildState::kPublished, 1, 3);
  ASSERT_NE(failed, recorder.copyStatuses().size());
  EXPECT_LT(failed, third_publish);
}

TEST(MapRebuilder, IncrementalHookExceptionRetriesAndPublishes) {
  Recorder recorder;
  std::atomic<unsigned int> attempts{};
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 1 && attempts.fetch_add(1) == 0) throw std::runtime_error("injected");
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  EXPECT_EQ(attempts.load(), 2U);
  EXPECT_TRUE(recorder.saw(RebuildState::kFailed, 1));
  const auto statuses = recorder.copyStatuses();
  for (const RebuildStatus& status : statuses) {
    if (status.state == RebuildState::kFailed && status.graph_revision == 1) {
      EXPECT_EQ(status.map_revision, 0U);
      EXPECT_EQ(status.committed_scan_count, 0U);
    }
  }
}

TEST(MapRebuilder, PersistentIncrementalHookExceptionIsPreservedWithoutSpin) {
  Recorder recorder;
  std::atomic<unsigned int> attempts{};
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 1) { ++attempts; throw std::runtime_error("injected"); }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitFor(2));
  EXPECT_EQ(attempts.load(), 2U);
  EXPECT_EQ(rebuilder.current(), nullptr);
  rebuilder.requestRebuild(trajectory(2, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(2));
  EXPECT_EQ(rebuilder.current()->committed_scan_count, 1U);
}

TEST(MapRebuilder, IncrementalFailureIsSuppressedWhenNewerFullIsPending) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) {
      incremental_gate.arrive();
      incremental_gate.block();
      throw std::runtime_error("injected");
    }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  ASSERT_TRUE(recorder.waitForStatus(RebuildState::kIdle, 1));
  const std::size_t before_failure = recorder.copyStatuses().size();
  rebuilder.appendCommitted(scan(2), pose(2), 1);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(2, {pose(1)}), archive({scan(1)}));
  incremental_gate.release();

  ASSERT_TRUE(recorder.waitForPublishedRevision(2));
  ASSERT_TRUE(recorder.waitFor(before_failure + 3));
  const auto statuses = recorder.copyStatuses();
  const std::size_t building = recorder.statusIndex(RebuildState::kBuilding, 2);
  const std::size_t published = recorder.statusIndex(RebuildState::kPublished, 2);
  ASSERT_LT(building, statuses.size());
  ASSERT_LT(published, statuses.size());
  for (std::size_t i = before_failure; i < published; ++i) {
    EXPECT_NE(statuses[i].state, RebuildState::kFailed);
    EXPECT_NE(statuses[i].state, RebuildState::kIdle);
  }
}

TEST(MapRebuilder, FullBuildFailureReportsCommittedSnapshotFields) {
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [](std::uint64_t revision, std::uint64_t) {
    if (revision == 2) throw std::runtime_error("injected");
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
  ASSERT_TRUE(recorder.waitForStatus(RebuildState::kFailed, 2));
  std::lock_guard<std::mutex> lock(recorder.mutex);
  bool saw_failure{};
  for (std::size_t i = 0; i < recorder.statuses.size(); ++i) {
    if (recorder.statuses[i].state == RebuildState::kFailed &&
        recorder.statuses[i].detail == "rebuild failed") {
      saw_failure = true;
      ASSERT_NE(recorder.snapshots[i], nullptr);
      EXPECT_EQ(recorder.statuses[i].map_revision, recorder.snapshots[i]->map_revision);
      EXPECT_EQ(recorder.statuses[i].committed_scan_count,
                recorder.snapshots[i]->committed_scan_count);
    }
  }
  EXPECT_TRUE(saw_failure);
}

TEST(MapRebuilder, ValidationFailureReportsCommittedSnapshotFields) {
  Recorder recorder;
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); });

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.requestRebuild(trajectory(2, {pose(2), pose(2)}), archive({scan(2), scan(2)}));
  ASSERT_TRUE(recorder.waitForStatus(RebuildState::kFailed, 2));
  std::lock_guard<std::mutex> lock(recorder.mutex);
  bool saw_failure{};
  for (std::size_t i = 0; i < recorder.statuses.size(); ++i) {
    if (recorder.statuses[i].state == RebuildState::kFailed &&
        recorder.statuses[i].detail == "duplicate trajectory scan id") {
      saw_failure = true;
      ASSERT_NE(recorder.snapshots[i], nullptr);
      EXPECT_EQ(recorder.statuses[i].map_revision, recorder.snapshots[i]->map_revision);
      EXPECT_EQ(recorder.statuses[i].committed_scan_count,
                recorder.snapshots[i]->committed_scan_count);
    }
  }
  EXPECT_TRUE(saw_failure);
}

TEST(MapRebuilder, IncrementalFailureReportsCommittedSnapshotFields) {
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [](std::uint64_t id) {
    if (id == 2) throw std::runtime_error("injected");
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.appendCommitted(scan(2), pose(2), 1);
  ASSERT_TRUE(recorder.waitFor(4));
  std::lock_guard<std::mutex> lock(recorder.mutex);
  bool saw_failure{};
  for (std::size_t i = 0; i < recorder.statuses.size(); ++i) {
    if (recorder.statuses[i].state == RebuildState::kFailed &&
        recorder.statuses[i].detail == "incremental update failed") {
      saw_failure = true;
      ASSERT_NE(recorder.snapshots[i], nullptr);
      EXPECT_EQ(recorder.statuses[i].map_revision, recorder.snapshots[i]->map_revision);
      EXPECT_EQ(recorder.statuses[i].committed_scan_count,
                recorder.snapshots[i]->committed_scan_count);
    }
  }
  EXPECT_TRUE(saw_failure);
}

TEST(MapRebuilder, StaleDiagnosticReportsCommittedSnapshotFields) {
  Gate incremental_gate;
  Recorder recorder;
  MapRebuilderTestHooks hooks;
  hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) {
      incremental_gate.arrive();
      incremental_gate.block();
    }
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  rebuilder.appendCommitted(scan(2), pose(2), 2);
  incremental_gate.wait();
  rebuilder.requestRebuild(trajectory(3, {pose(1)}), archive({scan(1)}));
  incremental_gate.release();
  ASSERT_TRUE(recorder.waitForPublishedRevision(3));
  ASSERT_TRUE(recorder.waitForStatus(RebuildState::kFailed, 3));
  std::lock_guard<std::mutex> lock(recorder.mutex);
  bool saw_failure{};
  for (std::size_t i = 0; i < recorder.statuses.size(); ++i) {
    if (recorder.statuses[i].state == RebuildState::kFailed &&
        recorder.statuses[i].detail == "incremental update stale after full rebuild") {
      saw_failure = true;
      ASSERT_NE(recorder.snapshots[i], nullptr);
      EXPECT_EQ(recorder.statuses[i].map_revision, recorder.snapshots[i]->map_revision);
      EXPECT_EQ(recorder.statuses[i].committed_scan_count,
                recorder.snapshots[i]->committed_scan_count);
    }
  }
  EXPECT_TRUE(saw_failure);
}

TEST(MapRebuilder, SuccessfulIncrementalCommitClearsReconciliationEntriesForItsScan) {
  Recorder retry_recorder;
  std::atomic<bool> fail{true};
  MapRebuilderTestHooks retry_hooks;
  retry_hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2 && fail.load()) throw std::runtime_error("injected");
  };
  MapRebuilder retry_rebuilder({}, [&](auto snapshot, const RebuildStatus& status) {
    retry_recorder.record(snapshot, status);
  }, retry_hooks);

  retry_rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(retry_recorder.waitForPublishedRevision(1));
  retry_rebuilder.appendCommitted(scan(2), pose(2), 1);
  ASSERT_TRUE(retry_recorder.waitFor(4));
  EXPECT_EQ(retry_rebuilder.testState().retry_exhausted_incrementals, 1U);
  fail.store(false);
  retry_rebuilder.appendCommitted(scan(2), pose(2), 1);
  ASSERT_TRUE(retry_recorder.waitFor(7));
  EXPECT_EQ(retry_rebuilder.testState().retry_exhausted_incrementals, 0U);

  Gate stale_gate;
  Recorder stale_recorder;
  MapRebuilderTestHooks stale_hooks;
  stale_hooks.before_incremental_commit = [&](std::uint64_t id) {
    if (id == 2) {
      stale_gate.arrive();
      stale_gate.block();
    }
  };
  MapRebuilder stale_rebuilder({}, [&](auto snapshot, const RebuildStatus& status) {
    stale_recorder.record(snapshot, status);
  }, stale_hooks);

  stale_rebuilder.appendCommitted(scan(1), pose(1), 1);
  ASSERT_TRUE(stale_recorder.waitForPublishedRevision(1));
  stale_rebuilder.appendCommitted(scan(2), pose(2), 2);
  stale_gate.wait();
  stale_rebuilder.requestRebuild(trajectory(3, {pose(1)}), archive({scan(1)}));
  stale_gate.release();
  ASSERT_TRUE(stale_recorder.waitForPublishedRevision(3));
  ASSERT_TRUE(stale_recorder.waitForStatus(RebuildState::kFailed, 3));
  EXPECT_EQ(stale_rebuilder.testState().stale_incrementals, 1U);
  stale_rebuilder.appendCommitted(scan(2), pose(2), 3);
  ASSERT_TRUE(stale_recorder.waitFor(8));
  EXPECT_EQ(stale_rebuilder.testState().stale_incrementals, 0U);
}

TEST(MapRebuilder, FullCorrectionAtIncrementalRevisionPublishes) {
  Recorder recorder;
  GridConfig config; config.resolution_m = 1.0;
  MapRebuilder rebuilder(config, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); });

  rebuilder.appendCommitted(scan(1), pose(1), 4);
  ASSERT_TRUE(recorder.waitForPublishedRevision(4));
  rebuilder.requestRebuild(trajectory(4, {pose(1, 7.0)}), archive({scan(1)}));

  ASSERT_TRUE(recorder.waitFor(4));
  EXPECT_EQ(recorder.published(4), 2U);
  EXPECT_EQ(rebuilder.current()->grid.cellAt(8, 0), 100);
}

TEST(MapRebuilder, FailedFullRevisionCanBeRetriedAtSameRevision) {
  Recorder recorder;
  std::atomic<bool> fail{true};
  MapRebuilderTestHooks hooks;
  hooks.before_rebuild_scan = [&](std::uint64_t revision, std::uint64_t) {
    if (revision == 4 && fail.exchange(false)) throw std::runtime_error("injected");
  };
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); }, hooks);

  rebuilder.requestRebuild(trajectory(4, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitFor(2));
  ASSERT_TRUE(recorder.saw(RebuildState::kFailed, 4));
  rebuilder.requestRebuild(trajectory(4, {pose(1, 3.0)}), archive({scan(1)}));

  ASSERT_TRUE(recorder.waitForPublishedRevision(4));
  EXPECT_EQ(recorder.published(4), 1U);
}

TEST(MapRebuilder, LowerFullRevisionIsRejectedAfterNewerCommittedState) {
  Recorder recorder;
  MapRebuilder rebuilder({}, [&](auto snapshot, const RebuildStatus& status) { recorder.record(snapshot, status); });

  rebuilder.appendCommitted(scan(1), pose(1), 4);
  ASSERT_TRUE(recorder.waitForPublishedRevision(4));
  rebuilder.requestRebuild(trajectory(3, {pose(2)}), archive({scan(2)}));
  rebuilder.requestRebuild(trajectory(5, {pose(3)}), archive({scan(3)}));

  ASSERT_TRUE(recorder.waitForPublishedRevision(5));
  EXPECT_EQ(recorder.published(3), 0U);
  EXPECT_EQ(rebuilder.current()->graph_revision, 5U);
}

TEST(MapRebuilder, CallbackQueuedWorkHasNoIdleBetweenPublishedAndBuilding) {
  Recorder recorder;
  std::unique_ptr<MapRebuilder> rebuilder;
  rebuilder = std::make_unique<MapRebuilder>(GridConfig{}, [&](auto snapshot, const RebuildStatus& status) {
    recorder.record(snapshot, status);
    if (status.state == RebuildState::kPublished && status.graph_revision == 1) {
      rebuilder->requestRebuild(trajectory(2, {pose(2)}), archive({scan(2)}));
    }
  });

  rebuilder->requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(2));
  ASSERT_TRUE(recorder.waitFor(5));
  const std::vector<RebuildStatus> statuses = recorder.copyStatuses();
  ASSERT_GE(statuses.size(), 5U);
  EXPECT_EQ(statuses[0].state, RebuildState::kBuilding);
  EXPECT_EQ(statuses[0].graph_revision, 1U);
  EXPECT_EQ(statuses[1].state, RebuildState::kPublished);
  EXPECT_EQ(statuses[1].graph_revision, 1U);
  EXPECT_EQ(statuses[2].state, RebuildState::kBuilding);
  EXPECT_EQ(statuses[2].graph_revision, 2U);
  EXPECT_EQ(statuses[3].state, RebuildState::kPublished);
  EXPECT_EQ(statuses[3].graph_revision, 2U);
  EXPECT_EQ(statuses[4].state, RebuildState::kIdle);
  EXPECT_EQ(statuses[4].graph_revision, 2U);
}

TEST(MapRebuilder, CallbackDestructionStopsBeforeAnyFurtherPublishCallbacks) {
  Recorder recorder;
  Gate worker_stopped;
  std::unique_ptr<MapRebuilder> rebuilder;
  std::atomic<bool> destroyed{};
  std::atomic<unsigned int> callbacks_after_destruction{};
  MapRebuilderTestHooks hooks;
  hooks.worker_stopped = [&] { worker_stopped.arrive(); };
  rebuilder = std::make_unique<MapRebuilder>(GridConfig{}, [&](auto snapshot, const RebuildStatus& status) {
    if (destroyed.load()) ++callbacks_after_destruction;
    recorder.record(snapshot, status);
    if (status.state == RebuildState::kPublished) {
      rebuilder.reset();
      destroyed = true;
    }
  }, hooks);

  rebuilder->requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  worker_stopped.wait();
  EXPECT_TRUE(destroyed.load());
  EXPECT_EQ(callbacks_after_destruction.load(), 0U);
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
  Gate worker_stopped;
  std::unique_ptr<MapRebuilder> rebuilder;
  std::atomic<bool> destroyed{};
  MapRebuilderTestHooks hooks;
  hooks.worker_stopped = [&] { worker_stopped.arrive(); };
  rebuilder = std::make_unique<MapRebuilder>(GridConfig{}, [&](auto snapshot, const RebuildStatus& status) {
    recorder.record(snapshot, status);
    if (status.state == RebuildState::kPublished && status.graph_revision == 1) {
      EXPECT_EQ(rebuilder->current(), snapshot);
      rebuilder.reset();
      destroyed = true;
      throw std::runtime_error("callback");
    }
  }, hooks);
  rebuilder->requestRebuild(trajectory(1, {pose(1)}), archive({scan(1)}));
  ASSERT_TRUE(recorder.waitForPublishedRevision(1));
  worker_stopped.wait();
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
