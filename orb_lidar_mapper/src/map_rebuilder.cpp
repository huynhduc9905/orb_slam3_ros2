#include "orb_lidar_mapper/map_rebuilder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
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
  for (const Ray2& ray : scan.rays) {
    result.push_back({transform(pose.pose, ray.origin), transform(pose.pose, ray.end), ray.has_hit});
  }
  return result;
}

std::string validate(const TrajectoryRevision& trajectory, const ScanArchive& archive) {
  std::unordered_set<std::uint64_t> trajectory_ids;
  for (const ScanPose& pose : trajectory.scans) {
    if (!trajectory_ids.insert(pose.scan_id).second) return "duplicate trajectory scan id";
  }
  std::unordered_set<std::uint64_t> archive_ids;
  for (const ArchivedScan& scan : archive.scans) {
    if (!archive_ids.insert(scan.scan_id).second) return "duplicate archive scan id";
    if (trajectory_ids.count(scan.scan_id) == 0) return "archive scan id missing from trajectory";
  }
  if (trajectory_ids.size() != archive_ids.size()) return "trajectory scan id missing from archive";
  return {};
}

}  // namespace

struct MapRebuilder::Impl : std::enable_shared_from_this<MapRebuilder::Impl> {
  struct Incremental { ArchivedScan scan; ScanPose pose; std::uint64_t graph_revision{}; };
  struct FullRequest {
    std::shared_ptr<const TrajectoryRevision> trajectory;
    std::shared_ptr<const ScanArchive> archive;
    std::unordered_set<std::uint64_t> represented;
    std::uint64_t generation{};
  };

  Impl(GridConfig config_in, PublishCallback callback_in, MapRebuilderTestHooks hooks_in)
      : config(std::move(config_in)), callback(std::move(callback_in)), hooks(std::move(hooks_in)),
        grid(std::make_unique<TiledOccupancyGrid>(config)) {}

  ~Impl() { shutdown(); }

  void start(const std::shared_ptr<Impl>& self) { worker = std::thread([self] { self->run(); }); }

  void shutdown() noexcept {
    { std::lock_guard<std::mutex> lock(mutex); stop = true; }
    cv.notify_all();
    if (!worker.joinable()) return;
    if (worker.get_id() == std::this_thread::get_id()) worker.detach();
    else worker.join();
  }

  void invoke(const std::shared_ptr<const MapSnapshot>& snapshot, const RebuildStatus& status) noexcept {
    try { if (callback) callback(snapshot, status); } catch (...) {}
  }

  void emit(RebuildState state, std::uint64_t graph_revision, std::uint64_t input_count,
            std::uint64_t committed_count, double duration_ms, std::string detail = {}) {
    std::shared_ptr<const MapSnapshot> snapshot;
    RebuildStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex);
      snapshot = current_snapshot;
      status = {state, graph_revision, map_revision, input_count, committed_count, duration_ms,
                std::move(detail)};
    }
    invoke(snapshot, status);
  }

  bool cancelled(const FullRequest& request) const {
    std::lock_guard<std::mutex> lock(mutex);
    return stop || request.generation != request_generation;
  }

  bool currentFullRepresentsLocked(std::uint64_t scan_id) const {
    const auto represented = [this, scan_id](const std::optional<FullRequest>& request) {
      return request && request->generation == request_generation &&
             request->represented.count(scan_id) != 0;
    };
    return represented(full_request) || represented(active_full);
  }

  void finishIdle(std::uint64_t graph_revision, std::uint64_t input_count,
                  std::uint64_t committed_count, double duration_ms) {
    emit(RebuildState::kIdle, graph_revision, input_count, committed_count, duration_ms);
  }

  void rebuild(FullRequest request) noexcept {
    const auto started = std::chrono::steady_clock::now();
    std::uint64_t committed{};
    try {
      auto candidate_grid = std::make_unique<TiledOccupancyGrid>(config);
      std::unordered_map<std::uint64_t, const ScanPose*> poses;
      poses.reserve(request.trajectory->scans.size());
      for (const ScanPose& pose : request.trajectory->scans) poses.emplace(pose.scan_id, &pose);
      std::unordered_set<std::uint64_t> applied;
      for (const ArchivedScan& scan : request.archive->scans) {
        if (cancelled(request)) return;
        if (hooks.before_rebuild_scan) hooks.before_rebuild_scan(request.trajectory->graph_revision, scan.scan_id);
        if (cancelled(request)) return;
        const auto found = poses.find(scan.scan_id);
        if (found != poses.end() && found->second->committed) {
          candidate_grid->insert(transformed(scan, *found->second));
          applied.insert(scan.scan_id);
          ++committed;
        }
      }
      GridSnapshot candidate_snapshot = candidate_grid->snapshot();
      auto snapshot = std::make_shared<MapSnapshot>();
      snapshot->grid = std::move(candidate_snapshot);
      if (hooks.before_full_commit) hooks.before_full_commit(request.trajectory->graph_revision);
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      RebuildStatus published;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (stop || request.generation != request_generation || !active_full ||
            active_full->generation != request.generation ||
            request.trajectory->graph_revision <= committed_graph_revision) return;
        grid = std::move(candidate_grid);
        committed_scan_count = committed;
        committed_graph_revision = request.trajectory->graph_revision;
        applied_scan_ids = std::move(applied);
        incrementals.erase(std::remove_if(incrementals.begin(), incrementals.end(),
            [&request](const Incremental& item) { return request.represented.count(item.scan.scan_id) != 0; }),
            incrementals.end());
        snapshot->graph_revision = committed_graph_revision;
        snapshot->map_revision = ++map_revision;
        snapshot->committed_scan_count = committed_scan_count;
        current_snapshot = snapshot;
        active_full.reset();
        published = {RebuildState::kPublished, snapshot->graph_revision, snapshot->map_revision,
                     request.archive->scans.size(), snapshot->committed_scan_count, elapsed, {}};
      }
      invoke(snapshot, published);
      finishIdle(request.trajectory->graph_revision, request.archive->scans.size(), committed, elapsed);
    } catch (...) {
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      bool report_failure = false;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (!stop && request.generation == request_generation && active_full &&
            active_full->generation == request.generation) {
          active_full.reset();
          report_failure = true;
        }
      }
      if (report_failure) {
        emit(RebuildState::kFailed, request.trajectory->graph_revision, request.archive->scans.size(),
             committed, elapsed, "rebuild failed");
        finishIdle(request.trajectory->graph_revision, request.archive->scans.size(), committed, elapsed);
      }
    }
  }

  void incremental(Incremental item) noexcept {
    try {
      std::unique_ptr<TiledOccupancyGrid> candidate_grid;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (stop || item.graph_revision < committed_graph_revision ||
            applied_scan_ids.count(item.scan.scan_id) != 0) return;
        candidate_grid = std::make_unique<TiledOccupancyGrid>(*grid);
      }
      candidate_grid->insert(transformed(item.scan, item.pose));
      GridSnapshot candidate_snapshot = candidate_grid->snapshot();
      auto snapshot = std::make_shared<MapSnapshot>();
      snapshot->grid = std::move(candidate_snapshot);
      if (hooks.before_incremental_commit) hooks.before_incremental_commit(item.scan.scan_id);
      RebuildStatus published;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (stop || item.graph_revision < committed_graph_revision ||
            applied_scan_ids.count(item.scan.scan_id) != 0) return;
        if (full_request || active_full) {
          if (!currentFullRepresentsLocked(item.scan.scan_id)) incrementals.push_front(std::move(item));
          return;
        }
        grid = std::move(candidate_grid);
        ++committed_scan_count;
        committed_graph_revision = std::max(committed_graph_revision, item.graph_revision);
        applied_scan_ids.insert(item.scan.scan_id);
        snapshot->graph_revision = committed_graph_revision;
        snapshot->map_revision = ++map_revision;
        snapshot->committed_scan_count = committed_scan_count;
        current_snapshot = snapshot;
        published = {RebuildState::kPublished, snapshot->graph_revision, snapshot->map_revision, 1,
                     snapshot->committed_scan_count, 0.0, {}};
      }
      invoke(snapshot, published);
      finishIdle(published.graph_revision, 1, published.committed_scan_count, 0.0);
    } catch (...) {
      emit(RebuildState::kFailed, item.graph_revision, 1, 0, 0.0, "incremental update failed");
      finishIdle(item.graph_revision, 1, 0, 0.0);
    }
  }

  void run() noexcept {
    while (true) {
      std::optional<FullRequest> full;
      std::optional<Incremental> item;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] { return stop || full_request || !incrementals.empty(); });
        if (stop) return;
        if (full_request) {
          active_full = *full_request;
          full = active_full;
          full_request.reset();
        } else {
          item = std::move(incrementals.front());
          incrementals.pop_front();
        }
      }
      if (full) rebuild(std::move(*full));
      else incremental(std::move(*item));
    }
  }

  GridConfig config;
  PublishCallback callback;
  MapRebuilderTestHooks hooks;
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::unique_ptr<TiledOccupancyGrid> grid;
  std::thread worker;
  std::deque<Incremental> incrementals;
  std::optional<FullRequest> full_request;
  std::optional<FullRequest> active_full;
  std::unordered_set<std::uint64_t> applied_scan_ids;
  std::shared_ptr<const MapSnapshot> current_snapshot;
  std::uint64_t request_generation{};
  std::uint64_t newest_graph_revision{};
  std::uint64_t committed_graph_revision{};
  std::uint64_t map_revision{};
  std::uint64_t committed_scan_count{};
  bool stop{};
};

MapRebuilder::MapRebuilder(GridConfig config, PublishCallback callback, MapRebuilderTestHooks hooks) {
  auto state = std::make_shared<Impl>(std::move(config), std::move(callback), std::move(hooks));
  state->start(state);
  impl_ = std::move(state);
}

MapRebuilder::~MapRebuilder() {
  if (impl_) impl_->shutdown();
}

void MapRebuilder::appendCommitted(const ArchivedScan& scan, const ScanPose& pose,
                                   std::uint64_t graph_revision) {
  const auto state = impl_;
  if (!state || !pose.committed || scan.scan_id != pose.scan_id) return;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->stop || graph_revision < state->committed_graph_revision ||
        state->applied_scan_ids.count(scan.scan_id) != 0) return;
    state->newest_graph_revision = std::max(state->newest_graph_revision, graph_revision);
    state->incrementals.push_back({scan, pose, graph_revision});
  }
  state->cv.notify_one();
}

void MapRebuilder::requestRebuild(std::shared_ptr<const TrajectoryRevision> trajectory,
                                  std::shared_ptr<const ScanArchive> archive) {
  const auto state = impl_;
  if (!state) return;
  const std::uint64_t graph_revision = trajectory ? trajectory->graph_revision : 0;
  const std::string validation = !trajectory || !archive ? "missing rebuild input" : validate(*trajectory, *archive);
  if (!validation.empty()) {
    state->emit(RebuildState::kFailed, graph_revision, archive ? archive->scans.size() : 0, 0, 0.0, validation);
    state->finishIdle(graph_revision, archive ? archive->scans.size() : 0, 0, 0.0);
    return;
  }
  Impl::FullRequest request;
  request.trajectory = std::move(trajectory);
  request.archive = std::move(archive);
  request.represented.reserve(request.archive->scans.size());
  for (const ArchivedScan& scan : request.archive->scans) request.represented.insert(scan.scan_id);
  const std::size_t input_count = request.archive->scans.size();
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->stop || request.trajectory->graph_revision <= state->newest_graph_revision ||
        request.trajectory->graph_revision <= state->committed_graph_revision) return;
    request.generation = ++state->request_generation;
    state->newest_graph_revision = request.trajectory->graph_revision;
    state->full_request = std::move(request);
  }
  state->emit(RebuildState::kBuilding, graph_revision, input_count, 0, 0.0);
  state->cv.notify_one();
}

std::shared_ptr<const MapSnapshot> MapRebuilder::current() const {
  const auto state = impl_;
  if (!state) return nullptr;
  std::lock_guard<std::mutex> lock(state->mutex);
  return state->current_snapshot;
}

}  // namespace orb_lidar_mapper
