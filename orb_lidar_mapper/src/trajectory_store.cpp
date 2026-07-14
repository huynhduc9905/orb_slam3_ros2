#include "orb_lidar_mapper/trajectory_store.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace orb_lidar_mapper {
namespace {

constexpr double kDistanceEpsilon = 1e-6;

std::uint64_t orderedDuration(std::int64_t earlier_ns, std::int64_t later_ns) {
  return static_cast<std::uint64_t>(later_ns) - static_cast<std::uint64_t>(earlier_ns);
}

double translationDistance(const Pose2& a, const Pose2& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

struct TrajectoryStore::LossInterval {
  std::size_t start_frame{};
  std::int64_t loss_stamp_ns{};
  std::optional<std::size_t> end_frame;
  std::optional<double> start_cumulative_distance;
  std::optional<double> end_cumulative_distance;
  std::optional<double> total_distance;
};

struct TrajectoryStore::StoredScan {
  ScanPose value;
  std::optional<std::size_t> anchor_frame;
  std::optional<std::size_t> interval;
  std::optional<Pose2> wheel_pose;
  std::optional<double> wheel_cumulative_distance;
  std::optional<double> correction_alpha;
};

TrajectoryStore::~TrajectoryStore() = default;

TrajectoryStore::TrajectoryStore(TrajectoryConfig config)
: wheels_(config.wheel_retention_ns, config.wheel_max_gap_ns), config_(config) {
  publish();
}

bool TrajectoryStore::addWheel(TimedPose2 sample) {
  if (!wheels_.push(sample)) {
    return false;
  }
  if (!wheel_states_.empty() && wheel_states_.back().stamp_ns == sample.stamp_ns) {
    WheelState& state = wheel_states_.back();
    state.pose = sample.pose;
    if (wheel_states_.size() > 1) {
      const WheelState& previous = wheel_states_[wheel_states_.size() - 2];
      state.cumulative_distance = previous.cumulative_distance +
        translationDistance(previous.pose, state.pose);
    } else if (wheel_predecessor_) {
      state.cumulative_distance = wheel_predecessor_->cumulative_distance +
        translationDistance(wheel_predecessor_->pose, state.pose);
    }
  } else {
    const double cumulative_distance = wheel_states_.empty() ? 0.0 :
      wheel_states_.back().cumulative_distance +
      translationDistance(wheel_states_.back().pose, sample.pose);
    wheel_states_.push_back({sample.stamp_ns, sample.pose, cumulative_distance});
  }
  refreshWheelData();
  for (std::size_t interval_index = 0; interval_index < intervals_.size(); ++interval_index) {
    finalizeInterval(interval_index);
  }
  recomputeAll();
  pruneWheelStates();
  publish();
  return true;
}

void TrajectoryStore::addTrackedFrame(FrameAnchor frame) {
  const std::size_t frame_index = frames_.size();
  frames_.push_back(std::move(frame));
  const FrameAnchor& added = frames_.back();

  if (added.state == TrackingState::kOk && added.pose_valid) {
    if (open_interval_) {
      intervals_[*open_interval_].end_frame = frame_index;
      finalizeInterval(*open_interval_);
      open_interval_.reset();
      recomputeAll();
    }
  } else if ((added.state == TrackingState::kRecentlyLost || added.state == TrackingState::kLost) &&
             !open_interval_) {
    const auto anchor = latestValidFrameAt(added.stamp_ns);
    if (anchor) {
      const FrameAnchor& start = frames_[*anchor];
      intervals_.push_back({*anchor, added.stamp_ns, std::nullopt,
                            wheelCumulativeDistance(start.stamp_ns, start.wheel_pose),
                            std::nullopt, std::nullopt});
      open_interval_ = intervals_.size() - 1;
    }
  }
  pruneWheelStates();
  publish();
}

std::optional<std::size_t> TrajectoryStore::latestValidFrameAt(std::int64_t stamp_ns) const {
  for (std::size_t i = frames_.size(); i > 0; --i) {
    const FrameAnchor& frame = frames_[i - 1];
    if (frame.stamp_ns <= stamp_ns && frame.state == TrackingState::kOk && frame.pose_valid) {
      return i - 1;
    }
  }
  return std::nullopt;
}

std::optional<Pose2> TrajectoryStore::poseFromFrame(
  std::size_t frame_index, std::int64_t stamp_ns) const {
  const FrameAnchor& anchor = frames_[frame_index];
  const auto scan_wheel_pose = wheels_.interpolate(stamp_ns);
  if (!scan_wheel_pose) {
    return std::nullopt;
  }
  return anchor.map_pose * anchor.wheel_pose.inverse() * *scan_wheel_pose;
}

std::optional<double> TrajectoryStore::wheelCumulativeDistance(
  std::int64_t stamp_ns, const Pose2& pose) const {
  if (wheel_predecessor_ && stamp_ns == wheel_predecessor_->stamp_ns) {
    return wheel_predecessor_->cumulative_distance;
  }
  if (wheel_states_.empty() || stamp_ns < wheel_states_.front().stamp_ns ||
      stamp_ns > wheel_states_.back().stamp_ns) {
    return std::nullopt;
  }
  for (std::size_t index = 0; index < wheel_states_.size(); ++index) {
    const WheelState& state = wheel_states_[index];
    if (state.stamp_ns == stamp_ns) {
      if (index == 0) {
        return state.cumulative_distance;
      }
      const WheelState& previous = wheel_states_[index - 1];
      return previous.cumulative_distance + translationDistance(previous.pose, pose);
    }
    if (state.stamp_ns > stamp_ns) {
      const WheelState& previous = wheel_states_[index - 1];
      return previous.cumulative_distance + translationDistance(previous.pose, pose);
    }
  }
  return std::nullopt;
}

std::optional<ScanPose> TrajectoryStore::placeScan(std::int64_t stamp_ns) {
  StoredScan scan;
  scan.value.scan_id = next_scan_id_;
  scan.value.stamp_ns = stamp_ns;
  scan.value.graph_revision = graph_revision_;
  const auto scan_wheel_pose = wheels_.interpolate(stamp_ns);
  if (!scan_wheel_pose) {
    return std::nullopt;
  }
  scan.wheel_pose = *scan_wheel_pose;
  scan.wheel_cumulative_distance = wheelCumulativeDistance(stamp_ns, *scan.wheel_pose);

  if (open_interval_) {
    const LossInterval& interval = intervals_[*open_interval_];
    if (stamp_ns >= interval.loss_stamp_ns) {
      scan.interval = *open_interval_;
      const FrameAnchor& anchor = frames_[interval.start_frame];
      scan.value.pose = anchor.map_pose * anchor.wheel_pose.inverse() * *scan.wheel_pose;
      scan.value.committed = false;
    }
  }
  if (!scan.interval) {
    const auto anchor = latestValidFrameAt(stamp_ns);
    if (!anchor) {
      return std::nullopt;
    }
    scan.anchor_frame = *anchor;
    const FrameAnchor& frame = frames_[*anchor];
    scan.value.pose = frame.map_pose * frame.wheel_pose.inverse() * *scan.wheel_pose;
    scan.value.committed = true;
  }

  ++next_scan_id_;
  scans_.push_back(scan);
  publish();
  return scan.value;
}

void TrajectoryStore::recomputeScan(StoredScan& scan) {
  scan.value.graph_revision = graph_revision_;
  if (scan.anchor_frame) {
    const FrameAnchor& anchor = frames_[*scan.anchor_frame];
    if (anchor.pose_valid && scan.wheel_pose) {
      scan.value.pose = anchor.map_pose * anchor.wheel_pose.inverse() * *scan.wheel_pose;
      scan.value.committed = true;
    } else {
      scan.value.committed = false;
    }
    return;
  }

  const LossInterval& interval = intervals_[*scan.interval];
  if (!scan.wheel_pose || !scan.correction_alpha || !interval.end_frame ||
      !frames_[interval.start_frame].pose_valid || !frames_[*interval.end_frame].pose_valid) {
    scan.value.committed = false;
    return;
  }
  const FrameAnchor& start = frames_[interval.start_frame];
  const FrameAnchor& end = frames_[*interval.end_frame];
  const Pose2 predicted = start.map_pose * start.wheel_pose.inverse() * *scan.wheel_pose;
  const Pose2 predicted_end = start.map_pose * start.wheel_pose.inverse() * end.wheel_pose;
  const Pose2 residual = predicted_end.inverse() * end.map_pose;
  scan.value.pose = predicted * residual.pow(*scan.correction_alpha);
  scan.value.committed = true;
}

void TrajectoryStore::finalizeInterval(std::size_t interval_index) {
  LossInterval& interval = intervals_[interval_index];
  if (!interval.end_frame) {
    return;
  }
  const FrameAnchor& start = frames_[interval.start_frame];
  const FrameAnchor& end = frames_[*interval.end_frame];
  if (end.stamp_ns < start.stamp_ns) {
    return;
  }
  if (!interval.start_cumulative_distance) {
    interval.start_cumulative_distance =
      wheelCumulativeDistance(start.stamp_ns, start.wheel_pose);
  }
  interval.end_cumulative_distance = wheelCumulativeDistance(end.stamp_ns, end.wheel_pose);
  if (!interval.start_cumulative_distance || !interval.end_cumulative_distance) {
    return;
  }
  interval.total_distance = *interval.end_cumulative_distance -
    *interval.start_cumulative_distance;
  const auto duration = orderedDuration(start.stamp_ns, end.stamp_ns);
  for (StoredScan& scan : scans_) {
    if (scan.interval != interval_index || scan.value.stamp_ns < start.stamp_ns ||
        scan.value.stamp_ns > end.stamp_ns) {
      continue;
    }
    if (!scan.wheel_cumulative_distance) {
      continue;
    }
    const double distance = *scan.wheel_cumulative_distance -
      *interval.start_cumulative_distance;
    scan.correction_alpha = *interval.total_distance < kDistanceEpsilon
      ? (duration == 0 ? 1.0 : static_cast<double>(orderedDuration(start.stamp_ns, scan.value.stamp_ns)) /
                            static_cast<double>(duration))
      : distance / *interval.total_distance;
  }
}

void TrajectoryStore::refreshWheelData() {
  for (StoredScan& scan : scans_) {
    const auto wheel_pose = wheels_.interpolate(scan.value.stamp_ns);
    const auto cumulative_distance = wheel_pose
      ? wheelCumulativeDistance(scan.value.stamp_ns, *wheel_pose) : std::nullopt;
    if (wheel_pose && cumulative_distance) {
      scan.wheel_pose = *wheel_pose;
      scan.wheel_cumulative_distance = *cumulative_distance;
    }
  }
  for (LossInterval& interval : intervals_) {
    const FrameAnchor& start = frames_[interval.start_frame];
    if (const auto cumulative = wheelCumulativeDistance(start.stamp_ns, start.wheel_pose)) {
      interval.start_cumulative_distance = *cumulative;
    }
    if (interval.end_frame) {
      const FrameAnchor& end = frames_[*interval.end_frame];
      if (const auto cumulative = wheelCumulativeDistance(end.stamp_ns, end.wheel_pose)) {
        interval.end_cumulative_distance = *cumulative;
      }
    }
  }
}

void TrajectoryStore::pruneWheelStates() {
  if (wheel_states_.empty()) {
    return;
  }
  const std::int64_t newest = wheel_states_.back().stamp_ns;
  const auto retention = static_cast<std::uint64_t>(config_.wheel_retention_ns);
  while (!wheel_states_.empty() &&
         orderedDuration(wheel_states_.front().stamp_ns, newest) > retention) {
    wheel_predecessor_ = wheel_states_.front();
    wheel_states_.erase(wheel_states_.begin());
  }
}

void TrajectoryStore::recomputeAll() {
  for (StoredScan& scan : scans_) {
    recomputeScan(scan);
  }
}

bool TrajectoryStore::applyGraphSnapshot(const GraphSnapshotValue& snapshot) {
  if (snapshot.graph_revision <= graph_revision_) {
    return false;
  }
  graph_revision_ = snapshot.graph_revision;
  for (FrameAnchor& frame : frames_) {
    if (!snapshot.active_map_connected) {
      frame.pose_valid = false;
      continue;
    }
    const auto keyframe = std::find_if(snapshot.keyframes.begin(), snapshot.keyframes.end(),
      [&frame, &snapshot](const KeyframeValue& value) {
        return value.keyframe_id == frame.reference_keyframe_id &&
               value.map_id == snapshot.active_map_id && frame.map_id == snapshot.active_map_id;
      });
    if (keyframe == snapshot.keyframes.end()) {
      frame.pose_valid = false;
    } else {
      frame.map_pose = keyframe->map_pose * frame.reference_to_frame;
      frame.pose_valid = true;
    }
  }
  recomputeAll();
  publish();
  return true;
}

void TrajectoryStore::publish() {
  auto revision = std::make_shared<TrajectoryRevision>();
  revision->graph_revision = graph_revision_;
  revision->frames = frames_;
  revision->scans.reserve(scans_.size());
  for (const StoredScan& scan : scans_) {
    revision->scans.push_back(scan.value);
  }
  revision_ = std::move(revision);
}

std::shared_ptr<const TrajectoryRevision> TrajectoryStore::snapshot() const {
  return revision_;
}

std::size_t TrajectoryStore::unresolvedScanCount() const {
  std::size_t count = 0;
  for (const StoredScan& scan : scans_) {
    if (!scan.value.committed) {
      ++count;
    }
  }
  return count;
}

std::size_t TrajectoryStore::wheelStateCount() const noexcept {
  return wheel_states_.size() + (wheel_predecessor_ ? 1U : 0U);
}

}  // namespace orb_lidar_mapper
