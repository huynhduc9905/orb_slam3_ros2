#include "orb_lidar_mapper/map_rebuilder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace orb_lidar_mapper {
namespace {

// Cap every incremental publication so a full request waits for no more than
// this many complete scan updates before receiving worker priority.
constexpr std::size_t kMaxIncrementalsPerPublication = 8;

Point2 transform(const Pose2& pose, Point2 point) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {pose.x + cosine * point.x - sine * point.y,
          pose.y + sine * point.x + cosine * point.y};
}

std::vector<Ray2> transformed(
  const ArchivedScan& scan, const ScanPose& pose,
  const TrajectoryRevision* trajectory = nullptr) {
  if (scan.bracketed_motion && trajectory) {
    const ArchivedBracketedMotion& motion = *scan.bracketed_motion;
    if (motion.start_frame_index >= trajectory->frames.size() ||
        motion.end_frame_index >= trajectory->frames.size()) {
      return {};
    }
    const FrameAnchor& start = trajectory->frames[motion.start_frame_index];
    const FrameAnchor& end = trajectory->frames[motion.end_frame_index];
    if (!start.pose_valid || !end.pose_valid || !start.wheel_pose || !end.wheel_pose ||
        start.map_id != end.map_id || start.graph_revision != end.graph_revision) {
      return {};
    }

    const Pose2 predicted_end = start.map_pose *
      start.wheel_pose->inverse() * *end.wheel_pose;
    const Pose2 residual = predicted_end.inverse() * end.map_pose;
    std::vector<Ray2> result;
    result.reserve(motion.rays.size());
    for (const RayMotion2& ray : motion.rays) {
      const Pose2 base_pose = start.map_pose * start.wheel_pose->inverse() *
        ray.wheel_pose * residual.pow(ray.alpha);
      const Pose2 lidar_pose = base_pose * motion.base_to_lidar;
      result.push_back({
        transform(lidar_pose, Point2{}),
        transform(lidar_pose, ray.lidar_end),
        ray.has_hit});
    }
    return result;
  }

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
  struct Incremental {
    ArchivedScan scan;
    ScanPose pose;
    std::uint64_t graph_revision{};
    bool deferred_by_full{};
    std::uint8_t retries{};
  };
  struct FullRequest {
    std::shared_ptr<const TrajectoryRevision> trajectory;
    std::shared_ptr<const ScanArchive> archive;
    std::unordered_set<std::uint64_t> represented;
    std::uint64_t generation{};
  };
  struct Failure {
    std::uint64_t graph_revision{};
    std::uint64_t input_count{};
    std::uint64_t generation{};
    std::string detail;
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

  void emitFailure(std::uint64_t graph_revision, std::uint64_t input_count,
                   double duration_ms, std::string detail) {
    std::shared_ptr<const MapSnapshot> snapshot;
    RebuildStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex);
      snapshot = current_snapshot;
      status = {RebuildState::kFailed, graph_revision, map_revision, input_count,
                committed_scan_count, duration_ms, std::move(detail)};
    }
    invoke(snapshot, status);
  }

  bool failureIsObsoleteLocked(const Failure& failure) const {
    const auto newer = [&failure](const std::optional<FullRequest>& request) {
      return request && request->generation > failure.generation &&
             request->trajectory->graph_revision >= failure.graph_revision;
    };
    return failure.graph_revision < committed_graph_revision || newer(full_request) || newer(active_full);
  }

  bool cancelled(const FullRequest& request) const {
    std::lock_guard<std::mutex> lock(mutex);
    return stop || request.generation != request_generation;
  }

  void finishIdleIfQuiescent(std::uint64_t generation, std::uint64_t graph_revision,
                             std::uint64_t input_count, std::uint64_t committed_count,
                             double duration_ms) {
    std::shared_ptr<const MapSnapshot> snapshot;
    RebuildStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stop || generation != request_generation || full_request || active_full ||
          !incrementals.empty() || !failures.empty()) return;
      snapshot = current_snapshot;
      status = {RebuildState::kIdle, graph_revision, map_revision, input_count, committed_count,
                duration_ms, {}};
    }
    invoke(snapshot, status);
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
          auto rebuilt_rays = transformed(
            scan, *found->second, request.trajectory.get());
          if (scan.bracketed_motion && rebuilt_rays.empty()) {
            throw std::runtime_error(
              "archived bracketed motion is incompatible with trajectory");
          }
          candidate_grid->insert(rebuilt_rays);
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
            request.trajectory->graph_revision < committed_graph_revision) return;
        grid = std::move(candidate_grid);
        committed_scan_count = committed;
        committed_graph_revision = request.trajectory->graph_revision;
        applied_scan_ids = std::move(applied);
        const auto erase_represented = [&request](auto& items) {
          items.erase(std::remove_if(items.begin(), items.end(), [&request](const Incremental& item) {
            return request.represented.count(item.scan.scan_id) != 0;
          }), items.end());
        };
        erase_represented(incrementals);
        erase_represented(retry_exhausted_incrementals);
        erase_represented(stale_incrementals);
        snapshot->graph_revision = committed_graph_revision;
        snapshot->map_revision = ++map_revision;
        snapshot->committed_scan_count = committed_scan_count;
        current_snapshot = snapshot;
        latest_successful_full_revision = request.trajectory->graph_revision;
        active_full.reset();
        published = {RebuildState::kPublished, snapshot->graph_revision, snapshot->map_revision,
                     request.archive->scans.size(), snapshot->committed_scan_count, elapsed, {}};
      }
      invoke(snapshot, published);
      finishIdleIfQuiescent(request.generation, request.trajectory->graph_revision,
                            request.archive->scans.size(), committed, elapsed);
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
        emitFailure(request.trajectory->graph_revision, request.archive->scans.size(), elapsed,
                    "rebuild failed");
        finishIdleIfQuiescent(request.generation, request.trajectory->graph_revision,
                              request.archive->scans.size(), committed, elapsed);
      }
    }
  }

  void incremental(std::vector<Incremental> items) noexcept {
    std::vector<Incremental> batch;
    batch.reserve(items.size());
    std::size_t active_index{};
    try {
      std::unique_ptr<TiledOccupancyGrid> candidate_grid;
      std::unordered_set<std::uint64_t> batch_scan_ids;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (stop) return;
        for (Incremental& item : items) {
          if (applied_scan_ids.count(item.scan.scan_id) != 0 ||
              !batch_scan_ids.insert(item.scan.scan_id).second) continue;
          if (item.graph_revision < committed_graph_revision) {
            if (item.deferred_by_full) stale_incrementals.push_back(std::move(item));
            failures.push_back({committed_graph_revision, 1, request_generation,
                                "incremental update stale after full rebuild"});
            continue;
          }
          batch.push_back(std::move(item));
        }
        if (!batch.empty()) candidate_grid = std::make_unique<TiledOccupancyGrid>(*grid);
      }
      if (batch.empty()) {
        cv.notify_one();
        return;
      }

      for (; active_index < batch.size(); ++active_index) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (stop) return;
          if (full_request || active_full) {
            for (auto it = batch.rbegin(); it != batch.rend(); ++it) {
              it->deferred_by_full = true;
              incrementals.push_front(std::move(*it));
            }
            return;
          }
        }
        candidate_grid->insert(transformed(batch[active_index].scan, batch[active_index].pose));
        if (hooks.before_incremental_commit) hooks.before_incremental_commit(batch[active_index].scan.scan_id);
      }
      GridSnapshot candidate_snapshot = candidate_grid->snapshot();
      auto snapshot = std::make_shared<MapSnapshot>();
      snapshot->grid = std::move(candidate_snapshot);
      RebuildStatus published;
      std::uint64_t generation{};
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (stop) return;
        if (full_request || active_full) {
          for (auto it = batch.rbegin(); it != batch.rend(); ++it) {
            it->deferred_by_full = true;
            incrementals.push_front(std::move(*it));
          }
          return;
        } else {
          grid = std::move(candidate_grid);
          committed_scan_count += batch.size();
          for (const Incremental& item : batch) {
            committed_graph_revision = std::max(committed_graph_revision, item.graph_revision);
            applied_scan_ids.insert(item.scan.scan_id);
          }
          const auto erase_batch = [&batch](auto& items) {
            items.erase(std::remove_if(items.begin(), items.end(), [&batch](const Incremental& candidate) {
              return std::any_of(batch.begin(), batch.end(), [&candidate](const Incremental& item) {
                return item.scan.scan_id == candidate.scan.scan_id;
              });
            }), items.end());
          };
          erase_batch(retry_exhausted_incrementals);
          erase_batch(stale_incrementals);
          snapshot->graph_revision = committed_graph_revision;
          snapshot->map_revision = ++map_revision;
          snapshot->committed_scan_count = committed_scan_count;
          current_snapshot = snapshot;
          generation = request_generation;
          published = {RebuildState::kPublished, snapshot->graph_revision, snapshot->map_revision, batch.size(),
                       snapshot->committed_scan_count, 0.0, {}};
        }
      }
      invoke(snapshot, published);
      finishIdleIfQuiescent(generation, published.graph_revision, batch.size(),
                            published.committed_scan_count, 0.0);
    } catch (...) {
      std::uint64_t generation{};
      const std::size_t failed_index = batch.empty() ? 0 : std::min(active_index, batch.size() - 1);
      const std::uint64_t graph_revision = batch.empty() ? 0 : batch[failed_index].graph_revision;
      bool retry{};
      bool enqueue_failure{};
      {
        std::lock_guard<std::mutex> lock(mutex);
        generation = request_generation;
        if (!stop) {
          for (std::size_t index = batch.size(); index-- > 0;) {
            Incremental& item = batch[index];
            if (item.graph_revision < committed_graph_revision ||
                applied_scan_ids.count(item.scan.scan_id) != 0) continue;
            if (index == failed_index) {
              if (item.retries == 0) {
                ++item.retries;
                retry = true;
              } else {
                retry_exhausted_incrementals.push_back(std::move(item));
                continue;
              }
            }
            incrementals.push_front(std::move(item));
          }
          failures.push_back({graph_revision, 1, generation, "incremental update failed"});
          enqueue_failure = true;
        }
      }
      if (retry || enqueue_failure) cv.notify_one();
      finishIdleIfQuiescent(generation, graph_revision, 1, 0, 0.0);
    }
  }

  void run() noexcept {
    while (true) {
      std::optional<FullRequest> full;
      std::vector<Incremental> items;
      std::optional<Failure> failure;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] { return stop || full_request || !incrementals.empty() || !failures.empty(); });
        if (stop) break;
        if (full_request) {
          active_full = *full_request;
          full = active_full;
          full_request.reset();
        } else if (!failures.empty()) {
          failure = std::move(failures.front());
          failures.pop_front();
        } else {
          while (!incrementals.empty() && items.size() < kMaxIncrementalsPerPublication) {
            items.push_back(std::move(incrementals.front()));
            incrementals.pop_front();
          }
        }
      }
      if (full) {
        emit(RebuildState::kBuilding, full->trajectory->graph_revision, full->archive->scans.size(), 0, 0.0);
        rebuild(std::move(*full));
      } else if (!items.empty()) {
        incremental(std::move(items));
      } else if (failure) {
        bool eligible{};
        std::uint64_t generation{};
        {
          std::lock_guard<std::mutex> lock(mutex);
          eligible = !failureIsObsoleteLocked(*failure);
          generation = request_generation;
        }
        if (eligible) {
          emitFailure(failure->graph_revision, failure->input_count, 0.0, failure->detail);
          finishIdleIfQuiescent(generation, failure->graph_revision, failure->input_count, 0, 0.0);
        } else {
          finishIdleIfQuiescent(generation, committed_graph_revision, 0, committed_scan_count, 0.0);
        }
      }
    }
    try { if (hooks.worker_stopped) hooks.worker_stopped(); } catch (...) {}
  }

  GridConfig config;
  PublishCallback callback;
  MapRebuilderTestHooks hooks;
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::unique_ptr<TiledOccupancyGrid> grid;
  std::thread worker;
  std::deque<Incremental> incrementals;
  std::deque<Failure> failures;
  std::vector<Incremental> retry_exhausted_incrementals;
  std::vector<Incremental> stale_incrementals;
  std::optional<FullRequest> full_request;
  std::optional<FullRequest> active_full;
  std::unordered_set<std::uint64_t> applied_scan_ids;
  std::shared_ptr<const MapSnapshot> current_snapshot;
  std::uint64_t request_generation{};
  std::uint64_t committed_graph_revision{};
  std::optional<std::uint64_t> latest_successful_full_revision;
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
    { std::lock_guard<std::mutex> lock(state->mutex);
      if (state->stop) return;
      state->failures.push_back({graph_revision, archive ? archive->scans.size() : 0,
                                 state->request_generation, validation}); }
    state->cv.notify_one();
    return;
  }
  Impl::FullRequest request;
  request.trajectory = std::move(trajectory);
  request.archive = std::move(archive);
  request.represented.reserve(request.archive->scans.size());
  for (const ArchivedScan& scan : request.archive->scans) request.represented.insert(scan.scan_id);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    const std::uint64_t revision = request.trajectory->graph_revision;
    const auto blocks_revision = [revision](const std::optional<Impl::FullRequest>& candidate) {
      return candidate && candidate->trajectory->graph_revision >= revision;
    };
    if (state->stop || revision < state->committed_graph_revision ||
        (state->latest_successful_full_revision &&
         *state->latest_successful_full_revision == revision) ||
        blocks_revision(state->full_request) || blocks_revision(state->active_full)) return;
    request.generation = ++state->request_generation;
    state->full_request = std::move(request);
  }
  state->cv.notify_one();
}

std::shared_ptr<const MapSnapshot> MapRebuilder::current() const {
  const auto state = impl_;
  if (!state) return nullptr;
  std::lock_guard<std::mutex> lock(state->mutex);
  return state->current_snapshot;
}

MapRebuilderTestState MapRebuilder::testState() const {
  const auto state = impl_;
  if (!state) return {};
  std::lock_guard<std::mutex> lock(state->mutex);
  return {state->retry_exhausted_incrementals.size(), state->stale_incrementals.size()};
}

}  // namespace orb_lidar_mapper
