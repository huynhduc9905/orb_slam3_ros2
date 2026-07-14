#include "orb_lidar_mapper/map_rebuilder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace orb_lidar_mapper {
namespace {

Point2 transform(const Pose2& pose, Point2 point) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {pose.x + cosine * point.x - sine * point.y,
          pose.y + sine * point.x + cosine * point.y};
}

std::vector<Ray2> transformed(const ArchivedScan& scan, const ScanPose& pose) {
  std::vector<Ray2> result;
  result.reserve(scan.rays.size());
  for (const Ray2& ray : scan.rays) result.push_back({transform(pose.pose, ray.origin),
                                                       transform(pose.pose, ray.end), ray.has_hit});
  return result;
}

}  // namespace

struct MapRebuilder::Impl {
  struct Incremental { ArchivedScan scan; ScanPose pose; std::uint64_t graph_revision{}; };
  struct FullRequest {
    std::shared_ptr<const TrajectoryRevision> trajectory;
    std::shared_ptr<const ScanArchive> archive;
    std::uint64_t generation{};
  };

  Impl(GridConfig config_in, PublishCallback callback_in, MapRebuilderTestHooks hooks_in)
      : config(std::move(config_in)), callback(std::move(callback_in)), hooks(std::move(hooks_in)),
        grid(std::make_unique<TiledOccupancyGrid>(config)), worker([this] { run(); }) {}

  ~Impl() {
    { std::lock_guard<std::mutex> lock(mutex); stop = true; }
    cv.notify_all();
    if (worker.joinable()) worker.join();
  }

  void publish(GridSnapshot grid_snapshot, std::uint64_t graph_revision,
               std::uint64_t input_count, std::uint64_t committed_count, double duration_ms) {
    std::shared_ptr<const MapSnapshot> snapshot;
    RebuildStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex);
      snapshot = std::make_shared<const MapSnapshot>(MapSnapshot{
          graph_revision, ++map_revision, committed_count, std::move(grid_snapshot)});
      current_snapshot = snapshot;
      status = {RebuildState::kPublished, graph_revision, map_revision, input_count,
                committed_count, duration_ms, {}};
    }
    invoke(snapshot, status);
  }

  void failed(std::uint64_t graph_revision, std::uint64_t input_count,
              std::uint64_t committed_count, double duration_ms, const char* detail) {
    RebuildStatus status;
    std::shared_ptr<const MapSnapshot> snapshot;
    { std::lock_guard<std::mutex> lock(mutex);
      snapshot = current_snapshot;
      status = {RebuildState::kFailed, graph_revision, map_revision, input_count,
                committed_count, duration_ms, detail}; }
    invoke(snapshot, status);
  }

  void invoke(const std::shared_ptr<const MapSnapshot>& snapshot, const RebuildStatus& status) noexcept {
    try { if (callback) callback(snapshot, status); } catch (...) {}
  }

  bool cancelled(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex);
    return stop || generation != request_generation;
  }

  void rebuild(FullRequest request) noexcept {
    const auto started = std::chrono::steady_clock::now();
    std::uint64_t committed{};
    try {
      auto private_grid = std::make_unique<TiledOccupancyGrid>(config);
      std::unordered_map<std::uint64_t, const ScanPose*> poses;
      for (const ScanPose& pose : request.trajectory->scans) poses.emplace(pose.scan_id, &pose);
      for (const ArchivedScan& scan : request.archive->scans) {
        if (cancelled(request.generation)) return;
        if (hooks.before_rebuild_scan) hooks.before_rebuild_scan(request.trajectory->graph_revision, scan.scan_id);
        if (cancelled(request.generation)) return;
        const auto found = poses.find(scan.scan_id);
        if (found == poses.end() || !found->second->committed) continue;
        private_grid->insert(transformed(scan, *found->second));
        ++committed;
      }
      if (cancelled(request.generation)) return;
      const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      grid = std::move(private_grid);
      committed_scan_count = committed;
      publish(grid->snapshot(), request.trajectory->graph_revision, request.archive->scans.size(), committed, elapsed);
    } catch (...) {
      if (!cancelled(request.generation)) {
        const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        failed(request.trajectory->graph_revision, request.archive->scans.size(), committed, elapsed, "rebuild failed");
      }
    }
  }

  void run() noexcept {
    while (true) {
      std::optional<FullRequest> full;
      std::optional<Incremental> incremental;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] { return stop || full_request || !incrementals.empty(); });
        if (stop) return;
        if (full_request) { full = std::move(full_request); full_request.reset(); }
        else { incremental = std::move(incrementals.front()); incrementals.pop_front(); }
      }
      if (full) { rebuild(std::move(*full)); continue; }
      try {
        grid->insert(transformed(incremental->scan, incremental->pose));
        ++committed_scan_count;
        publish(grid->snapshot(), incremental->graph_revision, 1, committed_scan_count, 0.0);
      } catch (...) {
        failed(incremental->graph_revision, 1, committed_scan_count, 0.0, "incremental update failed");
      }
    }
  }

  GridConfig config;
  PublishCallback callback;
  MapRebuilderTestHooks hooks;
  std::unique_ptr<TiledOccupancyGrid> grid;
  std::thread worker;
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::deque<Incremental> incrementals;
  std::unordered_set<std::uint64_t> represented_by_rebuild;
  std::optional<FullRequest> full_request;
  std::shared_ptr<const MapSnapshot> current_snapshot;
  std::uint64_t request_generation{};
  std::uint64_t map_revision{};
  std::uint64_t committed_scan_count{};
  bool stop{};
};

MapRebuilder::MapRebuilder(GridConfig config, PublishCallback callback, MapRebuilderTestHooks hooks)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(callback), std::move(hooks))) {}
MapRebuilder::~MapRebuilder() = default;

void MapRebuilder::appendCommitted(const ArchivedScan& scan, const ScanPose& pose,
                                   std::uint64_t graph_revision) {
  if (!pose.committed || scan.scan_id != pose.scan_id) return;
  { std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->represented_by_rebuild.count(scan.scan_id) != 0) return;
    impl_->incrementals.push_back({scan, pose, graph_revision}); }
  impl_->cv.notify_one();
}

void MapRebuilder::requestRebuild(std::shared_ptr<const TrajectoryRevision> trajectory,
                                  std::shared_ptr<const ScanArchive> scans) {
  if (!trajectory || !scans) return;
  { std::lock_guard<std::mutex> lock(impl_->mutex);
    std::unordered_set<std::uint64_t> represented;
    for (const ArchivedScan& scan : scans->scans) represented.insert(scan.scan_id);
    impl_->incrementals.erase(std::remove_if(impl_->incrementals.begin(), impl_->incrementals.end(),
      [&represented](const Impl::Incremental& item) { return represented.count(item.scan.scan_id) != 0; }),
      impl_->incrementals.end());
    impl_->represented_by_rebuild = std::move(represented);
    impl_->full_request = {std::move(trajectory), std::move(scans), ++impl_->request_generation}; }
  impl_->cv.notify_one();
}

std::shared_ptr<const MapSnapshot> MapRebuilder::current() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->current_snapshot;
}

}  // namespace orb_lidar_mapper
