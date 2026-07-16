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
  std::optional<std::size_t> end_anchor_frame;
  std::optional<std::size_t> interval;
  std::optional<Pose2> wheel_pose;
  std::optional<double> wheel_cumulative_distance;
  std::optional<double> correction_alpha;
  bool force_provisional{};
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
      const auto start_cumulative = start.wheel_pose
        ? wheelCumulativeDistance(start.stamp_ns, *start.wheel_pose)
        : std::nullopt;
      intervals_.push_back({*anchor, added.stamp_ns, std::nullopt,
                            start_cumulative,
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

std::optional<std::size_t> TrajectoryStore::lossIntervalAt(std::int64_t stamp_ns) const {
  for (std::size_t index = intervals_.size(); index > 0; --index) {
    const LossInterval& interval = intervals_[index - 1];
    if (stamp_ns < interval.loss_stamp_ns) {
      continue;
    }
    if (!interval.end_frame || stamp_ns <= frames_[*interval.end_frame].stamp_ns) {
      return index - 1;
    }
  }
  return std::nullopt;
}

bool TrajectoryStore::isLossTimestamp(std::int64_t stamp_ns) const {
  return lossIntervalAt(stamp_ns).has_value();
}

std::optional<std::pair<std::size_t, std::size_t>> TrajectoryStore::bracketingValidFrames(
  std::int64_t stamp_ns, std::int64_t scan_end_ns,
  std::int64_t max_visual_anchor_gap_ns) const {
  if (scan_end_ns < stamp_ns || max_visual_anchor_gap_ns < 0) {
    return std::nullopt;
  }

  const auto start_index = latestValidFrameAt(stamp_ns);
  if (!start_index) {
    return std::nullopt;
  }

  std::optional<std::size_t> end_index;
  for (std::size_t index = *start_index; index < frames_.size(); ++index) {
    const FrameAnchor& frame = frames_[index];
    if (frame.stamp_ns >= scan_end_ns && frame.state == TrackingState::kOk && frame.pose_valid) {
      end_index = index;
      break;
    }
  }
  if (!end_index) {
    return std::nullopt;
  }

  const FrameAnchor& start = frames_[*start_index];
  const FrameAnchor& end = frames_[*end_index];
  if (start.map_id != end.map_id || start.graph_revision != end.graph_revision) {
    return std::nullopt;
  }
  const auto max_gap = static_cast<std::uint64_t>(max_visual_anchor_gap_ns);
  if (orderedDuration(start.stamp_ns, stamp_ns) > max_gap ||
      orderedDuration(scan_end_ns, end.stamp_ns) > max_gap) {
    return std::nullopt;
  }

  // Never interpolate a committed scan across a tracking-loss, map, or graph
  // transition. A later graph snapshot can make both anchors compatible and
  // pending scans are retried from the mapper callback.
  for (std::size_t index = *start_index; index <= *end_index; ++index) {
    const FrameAnchor& frame = frames_[index];
    if (frame.state != TrackingState::kOk || !frame.pose_valid ||
        frame.map_id != start.map_id || frame.graph_revision != start.graph_revision) {
      return std::nullopt;
    }
  }
  return std::make_pair(*start_index, *end_index);
}

std::optional<Pose2> TrajectoryStore::bracketedPose(
  std::size_t start_frame, std::size_t end_frame,
  const Pose2& wheel_pose, double alpha) const {
  const FrameAnchor& start = frames_[start_frame];
  const FrameAnchor& end = frames_[end_frame];
  if (!start.pose_valid || !end.pose_valid || !start.wheel_pose || !end.wheel_pose) {
    return std::nullopt;
  }
  if (start_frame == end_frame) {
    return start.map_pose;
  }

  const Pose2 predicted = start.map_pose * start.wheel_pose->inverse() * wheel_pose;
  const Pose2 predicted_end = start.map_pose * start.wheel_pose->inverse() * *end.wheel_pose;
  const Pose2 residual = predicted_end.inverse() * end.map_pose;
  return predicted * residual.pow(alpha);
}

std::optional<Pose2> TrajectoryStore::poseFromFrame(
  std::size_t frame_index, std::int64_t stamp_ns) const {
  const FrameAnchor& anchor = frames_[frame_index];
  if (!anchor.wheel_pose) {
    return anchor.map_pose;
  }
  const auto scan_wheel_pose = wheels_.interpolate(stamp_ns);
  if (!scan_wheel_pose) {
    return anchor.map_pose;
  }
  return anchor.map_pose * anchor.wheel_pose->inverse() * *scan_wheel_pose;
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

std::optional<ScanPose> TrajectoryStore::placeScan(
  std::int64_t stamp_ns, bool force_provisional) {
  StoredScan scan;
  scan.force_provisional = force_provisional;
  scan.value.scan_id = next_scan_id_;
  scan.value.stamp_ns = stamp_ns;
  scan.value.graph_revision = graph_revision_;
  const auto scan_wheel_pose = wheels_.interpolate(stamp_ns);
  if (scan_wheel_pose) {
    scan.wheel_pose = *scan_wheel_pose;
    scan.wheel_cumulative_distance = wheelCumulativeDistance(stamp_ns, *scan_wheel_pose);
  }

  const auto loss_interval = lossIntervalAt(stamp_ns);
  if (loss_interval) {
    const LossInterval& interval = intervals_[*loss_interval];
    scan.interval = *loss_interval;
    const FrameAnchor& anchor = frames_[interval.start_frame];
    if (anchor.wheel_pose && scan.wheel_pose) {
      scan.value.pose = anchor.map_pose * anchor.wheel_pose->inverse() * *scan.wheel_pose;
    }
    // The timestamp, not callback arrival order, determines membership in a
    // LOST interval. A delayed scan remains attached to its closed interval so
    // recovery correction cannot turn it into an unrelated nearest-anchor scan.
    scan.value.committed = false;
  }
  if (!scan.interval) {
    const auto anchor = latestValidFrameAt(stamp_ns);
    if (!anchor) {
      return std::nullopt;
    }
    scan.anchor_frame = *anchor;
    const FrameAnchor& frame = frames_[*anchor];
    if (frame.wheel_pose && scan.wheel_pose) {
      // Full accuracy: bridge the anchor's visual pose to the scan's exact
      // timestamp using the relative wheel motion between the two instants.
      scan.value.pose = frame.map_pose * frame.wheel_pose->inverse() * *scan.wheel_pose;
    } else {
      // Nearest-anchor snap: wheel odometry is unavailable for this instant
      // (e.g. live timing skew), but ORB-SLAM3 is tracking OK, so use the
      // anchor's visual pose directly rather than dropping the scan. Wheel
      // remains the primary fallback only during LOST/RECENTLY_LOST intervals.
      scan.value.pose = frame.map_pose;
    }
    scan.value.committed = !scan.force_provisional;
  }

  ++next_scan_id_;
  scans_.push_back(scan);
  if (scan.interval && intervals_[*scan.interval].end_frame) {
    finalizeInterval(*scan.interval);
    recomputeScan(scans_.back());
  }
  publish();
  return scans_.back().value;
}

std::optional<VisualWheelBracket> TrajectoryStore::visualWheelBracket(
  std::int64_t stamp_ns, std::int64_t scan_end_ns,
  std::int64_t max_visual_anchor_gap_ns) const {
  const auto indices = bracketingValidFrames(
    stamp_ns, scan_end_ns, max_visual_anchor_gap_ns);
  if (!indices) {
    return std::nullopt;
  }
  const FrameAnchor& start = frames_[indices->first];
  const FrameAnchor& end = frames_[indices->second];
  if (!start.wheel_pose || !end.wheel_pose) {
    return std::nullopt;
  }
  return VisualWheelBracket{
    indices->first, indices->second,
    start.stamp_ns, end.stamp_ns,
    start.map_pose, end.map_pose,
    *start.wheel_pose, *end.wheel_pose};
}

std::optional<ScanPose> TrajectoryStore::placeBracketedScan(
  std::int64_t stamp_ns, std::int64_t scan_end_ns,
  std::int64_t max_visual_anchor_gap_ns) {
  const auto bracket = visualWheelBracket(
    stamp_ns, scan_end_ns, max_visual_anchor_gap_ns);
  const auto scan_wheel_pose = wheels_.interpolate(stamp_ns);
  if (!bracket || !scan_wheel_pose) {
    return std::nullopt;
  }

  const auto duration = orderedDuration(bracket->start_stamp_ns, bracket->end_stamp_ns);
  const double alpha = duration == 0 ? 0.0 :
    static_cast<double>(orderedDuration(bracket->start_stamp_ns, stamp_ns)) /
    static_cast<double>(duration);
  const auto pose = bracketedPose(
    bracket->start_frame_index, bracket->end_frame_index, *scan_wheel_pose, alpha);
  if (!pose) {
    return std::nullopt;
  }

  StoredScan scan;
  scan.value.scan_id = next_scan_id_++;
  scan.value.stamp_ns = stamp_ns;
  scan.value.pose = *pose;
  scan.value.committed = true;
  scan.value.graph_revision = graph_revision_;
  scan.anchor_frame = bracket->start_frame_index;
  scan.end_anchor_frame = bracket->end_frame_index;
  scan.wheel_pose = *scan_wheel_pose;
  scan.wheel_cumulative_distance = wheelCumulativeDistance(stamp_ns, *scan_wheel_pose);
  scan.correction_alpha = alpha;
  scans_.push_back(scan);
  publish();
  return scan.value;
}

void TrajectoryStore::recomputeScan(StoredScan& scan) {
  scan.value.graph_revision = graph_revision_;
  if (scan.anchor_frame) {
    const FrameAnchor& anchor = frames_[*scan.anchor_frame];
    if (!anchor.pose_valid) {
      scan.value.committed = false;
      return;
    }
    if (scan.end_anchor_frame) {
      if (!scan.wheel_pose || !scan.correction_alpha) {
        scan.value.committed = false;
        return;
      }
      const auto pose = bracketedPose(
        *scan.anchor_frame, *scan.end_anchor_frame,
        *scan.wheel_pose, *scan.correction_alpha);
      if (!pose) {
        scan.value.committed = false;
        return;
      }
      scan.value.pose = *pose;
    } else if (anchor.wheel_pose && scan.wheel_pose) {
      scan.value.pose = anchor.map_pose * anchor.wheel_pose->inverse() * *scan.wheel_pose;
    } else {
      // Nearest-anchor snap — see placeScan().
      scan.value.pose = anchor.map_pose;
    }
    scan.value.committed = !scan.force_provisional;
    return;
  }

  const LossInterval& interval = intervals_[*scan.interval];
  if (!scan.wheel_pose || !scan.correction_alpha || !interval.end_frame ||
      !frames_[interval.start_frame].pose_valid || !frames_[*interval.end_frame].pose_valid ||
      !frames_[interval.start_frame].wheel_pose || !frames_[*interval.end_frame].wheel_pose) {
    scan.value.committed = false;
    return;
  }
  const FrameAnchor& start = frames_[interval.start_frame];
  const FrameAnchor& end = frames_[*interval.end_frame];
  const Pose2 predicted = start.map_pose * start.wheel_pose->inverse() * *scan.wheel_pose;
  const Pose2 predicted_end = start.map_pose * start.wheel_pose->inverse() * *end.wheel_pose;
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
  if (!interval.start_cumulative_distance && start.wheel_pose) {
    interval.start_cumulative_distance =
      wheelCumulativeDistance(start.stamp_ns, *start.wheel_pose);
  }
  interval.end_cumulative_distance = end.wheel_pose
    ? wheelCumulativeDistance(end.stamp_ns, *end.wheel_pose)
    : std::nullopt;
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
  // Tracked frames may arrive before the corresponding odometry callback.
  // Backfill their wheel bridge as soon as the timestamp becomes bracketed so
  // deferred scans can use visual anchors on both sides without a permanent
  // nearest-anchor snap.
  for (FrameAnchor& frame : frames_) {
    if (!frame.wheel_pose) {
      frame.wheel_pose = wheels_.interpolate(frame.stamp_ns);
    }
  }
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
    if (start.wheel_pose) {
      if (const auto cumulative = wheelCumulativeDistance(start.stamp_ns, *start.wheel_pose)) {
        interval.start_cumulative_distance = *cumulative;
      }
    }
    if (interval.end_frame) {
      const FrameAnchor& end = frames_[*interval.end_frame];
      if (end.wheel_pose) {
        if (const auto cumulative = wheelCumulativeDistance(end.stamp_ns, *end.wheel_pose)) {
          interval.end_cumulative_distance = *cumulative;
        }
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

std::optional<std::int64_t> TrajectoryStore::latestFrameStamp() const noexcept {
  return frames_.empty() ? std::nullopt :
    std::optional<std::int64_t>(frames_.back().stamp_ns);
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
      frame.graph_revision = snapshot.graph_revision;
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
