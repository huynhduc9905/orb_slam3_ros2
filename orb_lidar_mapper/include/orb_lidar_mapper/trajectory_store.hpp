#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "orb_lidar_mapper/timed_pose_buffer.hpp"

namespace orb_lidar_mapper {

enum class TrackingState { kNoImages, kNotInitialized, kOk, kRecentlyLost, kLost };

struct TrajectoryConfig {
  std::int64_t wheel_retention_ns{};
  std::int64_t wheel_max_gap_ns{};
};

struct FrameAnchor {
  std::int64_t stamp_ns{};
  TrackingState state{};
  bool pose_valid{};
  std::uint64_t map_id{};
  std::uint64_t reference_keyframe_id{};
  Pose2 map_pose;
  Pose2 reference_to_frame;
  Pose2 wheel_pose;
};

struct ScanPose {
  std::uint64_t scan_id{};
  std::int64_t stamp_ns{};
  Pose2 pose;
  bool committed{};
  std::uint64_t graph_revision{};
};

struct TrajectoryRevision {
  std::uint64_t graph_revision{};
  std::vector<FrameAnchor> frames;
  std::vector<ScanPose> scans;
};

struct KeyframeValue {
  std::uint64_t keyframe_id{};
  std::uint64_t map_id{};
  Pose2 map_pose;
};

struct GraphSnapshotValue {
  std::uint64_t graph_revision{};
  std::uint64_t active_map_id{};
  bool active_map_connected{};
  std::vector<KeyframeValue> keyframes;
};

class TrajectoryStore {
 public:
  explicit TrajectoryStore(TrajectoryConfig config);
  ~TrajectoryStore();
  bool addWheel(TimedPose2 sample);
  void addTrackedFrame(FrameAnchor frame);
  std::optional<ScanPose> placeScan(std::int64_t stamp_ns);
  bool applyGraphSnapshot(const GraphSnapshotValue& snapshot);
  std::shared_ptr<const TrajectoryRevision> snapshot() const;
  std::size_t unresolvedScanCount() const;

 private:
  struct LossInterval;
  struct StoredScan;

  void recomputeAll();
  void recomputeScan(StoredScan& scan);
  std::optional<std::size_t> latestValidFrameAt(std::int64_t stamp_ns) const;
  std::optional<Pose2> poseFromFrame(std::size_t frame_index, std::int64_t stamp_ns) const;
  std::optional<double> cumulativeWheelDistance(
    std::int64_t from_ns, const Pose2& from_pose, std::int64_t to_ns,
    const Pose2& to_pose) const;
  void finalizeInterval(std::size_t interval_index);
  void pruneWheelHistory();
  void publish();

  TimedPoseBuffer wheels_;
  TrajectoryConfig config_;
  std::vector<TimedPose2> wheel_history_;
  std::vector<FrameAnchor> frames_;
  std::vector<StoredScan> scans_;
  std::vector<LossInterval> intervals_;
  std::optional<std::size_t> open_interval_;
  std::uint64_t graph_revision_{};
  std::uint64_t next_scan_id_{};
  std::shared_ptr<const TrajectoryRevision> revision_;
};

}  // namespace orb_lidar_mapper
