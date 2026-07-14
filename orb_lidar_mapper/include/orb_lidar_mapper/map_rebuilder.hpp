#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "orb_lidar_mapper/tiled_occupancy_grid.hpp"
#include "orb_lidar_mapper/trajectory_store.hpp"

namespace orb_lidar_mapper {

struct ArchivedScan {
  std::uint64_t scan_id{};
  std::int64_t stamp_ns{};
  std::vector<Ray2> rays;
};

struct ScanArchive {
  std::vector<ArchivedScan> scans;
};

struct MapSnapshot {
  std::uint64_t graph_revision{};
  std::uint64_t map_revision{};
  std::uint64_t committed_scan_count{};
  GridSnapshot grid;
};

enum class RebuildState { kIdle, kBuilding, kPublished, kFailed };

struct RebuildStatus {
  RebuildState state{RebuildState::kIdle};
  std::uint64_t graph_revision{};
  std::uint64_t map_revision{};
  std::uint64_t input_scan_count{};
  std::uint64_t committed_scan_count{};
  double duration_ms{};
  std::string detail;
};

// A narrow deterministic seam for tests that need to block or throw between scans.
struct MapRebuilderTestHooks {
  std::function<void(std::uint64_t graph_revision, std::uint64_t scan_id)> before_rebuild_scan;
  std::function<void(std::uint64_t graph_revision)> before_full_commit;
  std::function<void(std::uint64_t scan_id)> before_incremental_commit;
  std::function<void()> worker_stopped;
};

struct MapRebuilderTestState {
  std::size_t retry_exhausted_incrementals{};
  std::size_t stale_incrementals{};
};

class MapRebuilder {
 public:
  using PublishCallback = std::function<void(std::shared_ptr<const MapSnapshot>, const RebuildStatus&)>;

  MapRebuilder(GridConfig config, PublishCallback callback,
               MapRebuilderTestHooks hooks = {});
  ~MapRebuilder();
  MapRebuilder(const MapRebuilder&) = delete;
  MapRebuilder& operator=(const MapRebuilder&) = delete;

  void appendCommitted(const ArchivedScan& scan, const ScanPose& pose,
                       std::uint64_t graph_revision);
  void requestRebuild(std::shared_ptr<const TrajectoryRevision> trajectory,
                      std::shared_ptr<const ScanArchive> scans);
  std::shared_ptr<const MapSnapshot> current() const;
  MapRebuilderTestState testState() const;

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace orb_lidar_mapper
