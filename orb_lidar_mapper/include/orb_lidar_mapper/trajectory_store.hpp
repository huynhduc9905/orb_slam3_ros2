#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
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
  // Interpolated wheel pose at this frame's timestamp — used only to bridge the
  // visual anchor to a scan's exact sub-anchor timestamp. Absent at ingestion
  // when wheel odometry has not arrived yet; addWheel() backfills it later.
  std::optional<Pose2> wheel_pose;
  std::uint64_t graph_revision{};
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

struct VisualWheelBracket {
  std::size_t start_frame_index{};
  std::size_t end_frame_index{};
  std::int64_t start_stamp_ns{};
  std::int64_t end_stamp_ns{};
  Pose2 start_map_pose;
  Pose2 end_map_pose;
  Pose2 start_wheel_pose;
  Pose2 end_wheel_pose;
};

class TrajectoryStore {
 public:
  explicit TrajectoryStore(TrajectoryConfig config);
  ~TrajectoryStore();
  bool addWheel(TimedPose2 sample);
  void addTrackedFrame(FrameAnchor frame);
  std::optional<ScanPose> placeScan(
    std::int64_t stamp_ns, bool force_provisional = false);
  std::optional<VisualWheelBracket> visualWheelBracket(
    std::int64_t stamp_ns, std::int64_t scan_end_ns,
    std::int64_t max_visual_anchor_gap_ns) const;
  std::optional<ScanPose> placeBracketedScan(
    std::int64_t stamp_ns, std::int64_t scan_end_ns,
    std::int64_t max_visual_anchor_gap_ns);
  bool isLossTimestamp(std::int64_t stamp_ns) const;
  std::optional<std::int64_t> latestFrameStamp() const noexcept;
  bool applyGraphSnapshot(const GraphSnapshotValue& snapshot);
  std::shared_ptr<const TrajectoryRevision> snapshot() const;
  std::size_t unresolvedScanCount() const;
  std::size_t wheelStateCount() const noexcept;

 private:
  struct LossInterval;
  struct StoredScan;
  struct WheelState {
    std::int64_t stamp_ns{};
    Pose2 pose;
    double cumulative_distance{};
  };

  void recomputeAll();
  void recomputeScan(StoredScan& scan);
  std::optional<std::size_t> latestValidFrameAt(std::int64_t stamp_ns) const;
  std::optional<std::size_t> lossIntervalAt(std::int64_t stamp_ns) const;
  std::optional<std::pair<std::size_t, std::size_t>> bracketingValidFrames(
    std::int64_t stamp_ns, std::int64_t scan_end_ns,
    std::int64_t max_visual_anchor_gap_ns) const;
  std::optional<Pose2> bracketedPose(
    std::size_t start_frame, std::size_t end_frame,
    const Pose2& wheel_pose, double alpha) const;
  std::optional<Pose2> poseFromFrame(std::size_t frame_index, std::int64_t stamp_ns) const;
  std::optional<double> wheelCumulativeDistance(
    std::int64_t stamp_ns, const Pose2& pose) const;
  void finalizeInterval(std::size_t interval_index);
  void refreshWheelData();
  void pruneWheelStates();
  void publish();

  TimedPoseBuffer wheels_;
  TrajectoryConfig config_;
  std::optional<WheelState> wheel_predecessor_;
  std::vector<WheelState> wheel_states_;
  std::vector<FrameAnchor> frames_;
  std::vector<StoredScan> scans_;
  std::vector<LossInterval> intervals_;
  std::optional<std::size_t> open_interval_;
  std::uint64_t graph_revision_{};
  std::uint64_t next_scan_id_{};
  std::shared_ptr<const TrajectoryRevision> revision_;
};

}  // namespace orb_lidar_mapper
