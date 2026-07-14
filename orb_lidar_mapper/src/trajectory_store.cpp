#include "orb_lidar_mapper/trajectory_store.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace orb_lidar_mapper {
namespace {

constexpr double kDistanceEpsilon = 1e-6;

double translationDistance(const Pose2& a, const Pose2& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

struct TrajectoryStore::LossInterval {
  std::size_t start_frame{};
  std::int64_t loss_stamp_ns{};
  std::optional<std::size_t> end_frame;
};

struct TrajectoryStore::StoredScan {
  ScanPose value;
  std::optional<std::size_t> anchor_frame;
  std::optional<std::size_t> interval;
};

TrajectoryStore::~TrajectoryStore() = default;

TrajectoryStore::TrajectoryStore(TrajectoryConfig config)
: wheels_(config.wheel_retention_ns, config.wheel_max_gap_ns) {
  publish();
}

bool TrajectoryStore::addWheel(TimedPose2 sample) {
  if (!wheels_.push(sample)) {
    return false;
  }
  if (!wheel_history_.empty() && wheel_history_.back().stamp_ns == sample.stamp_ns) {
    wheel_history_.back() = sample;
  } else {
    wheel_history_.push_back(sample);
  }
  return true;
}

void TrajectoryStore::addTrackedFrame(FrameAnchor frame) {
  const std::size_t frame_index = frames_.size();
  frames_.push_back(std::move(frame));
  const FrameAnchor& added = frames_.back();

  if (added.state == TrackingState::kOk && added.pose_valid) {
    if (open_interval_) {
      intervals_[*open_interval_].end_frame = frame_index;
      open_interval_.reset();
      recomputeAll();
    }
  } else if ((added.state == TrackingState::kRecentlyLost || added.state == TrackingState::kLost) &&
             !open_interval_) {
    const auto anchor = latestValidFrameAt(added.stamp_ns);
    if (anchor) {
      intervals_.push_back({*anchor, added.stamp_ns, std::nullopt});
      open_interval_ = intervals_.size() - 1;
    }
  }
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
  const auto relative = wheels_.relative(anchor.stamp_ns, stamp_ns);
  if (!relative) {
    return std::nullopt;
  }
  return anchor.map_pose * *relative;
}

std::optional<double> TrajectoryStore::cumulativeWheelDistance(
  std::int64_t from_ns, std::int64_t to_ns) const {
  const auto from = wheels_.interpolate(from_ns);
  const auto to = wheels_.interpolate(to_ns);
  if (!from || !to || to_ns < from_ns) {
    return std::nullopt;
  }
  Pose2 previous = *from;
  double distance = 0.0;
  for (const TimedPose2& sample : wheel_history_) {
    if (sample.stamp_ns <= from_ns || sample.stamp_ns >= to_ns) {
      continue;
    }
    distance += translationDistance(previous, sample.pose);
    previous = sample.pose;
  }
  return distance + translationDistance(previous, *to);
}

std::optional<ScanPose> TrajectoryStore::placeScan(std::int64_t stamp_ns) {
  StoredScan scan;
  scan.value.scan_id = next_scan_id_;
  scan.value.stamp_ns = stamp_ns;
  scan.value.graph_revision = graph_revision_;

  if (open_interval_) {
    const LossInterval& interval = intervals_[*open_interval_];
    if (stamp_ns >= interval.loss_stamp_ns) {
      scan.interval = *open_interval_;
      const auto predicted = poseFromFrame(interval.start_frame, stamp_ns);
      if (!predicted) {
        return std::nullopt;
      }
      scan.value.pose = *predicted;
      scan.value.committed = false;
    }
  }
  if (!scan.interval) {
    const auto anchor = latestValidFrameAt(stamp_ns);
    if (!anchor) {
      return std::nullopt;
    }
    const auto pose = poseFromFrame(*anchor, stamp_ns);
    if (!pose) {
      return std::nullopt;
    }
    scan.anchor_frame = *anchor;
    scan.value.pose = *pose;
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
    const auto pose = anchor.pose_valid ? poseFromFrame(*scan.anchor_frame, scan.value.stamp_ns)
                                        : std::nullopt;
    if (pose) {
      scan.value.pose = *pose;
      scan.value.committed = true;
    } else {
      scan.value.committed = false;
    }
    return;
  }

  const LossInterval& interval = intervals_[*scan.interval];
  const auto predicted = poseFromFrame(interval.start_frame, scan.value.stamp_ns);
  if (!predicted || !interval.end_frame || !frames_[interval.start_frame].pose_valid ||
      !frames_[*interval.end_frame].pose_valid) {
    scan.value.committed = false;
    return;
  }
  const auto predicted_end = poseFromFrame(interval.start_frame, frames_[*interval.end_frame].stamp_ns);
  if (!predicted_end) {
    scan.value.committed = false;
    return;
  }
  const Pose2 residual = predicted_end->inverse() * frames_[*interval.end_frame].map_pose;
  const auto distance_to_scan = cumulativeWheelDistance(
    frames_[interval.start_frame].stamp_ns, scan.value.stamp_ns);
  const auto total_distance = cumulativeWheelDistance(
    frames_[interval.start_frame].stamp_ns, frames_[*interval.end_frame].stamp_ns);
  if (!distance_to_scan || !total_distance) {
    scan.value.committed = false;
    return;
  }
  double alpha = 0.0;
  if (*total_distance < kDistanceEpsilon) {
    const auto duration = frames_[*interval.end_frame].stamp_ns - frames_[interval.start_frame].stamp_ns;
    alpha = duration == 0 ? 1.0 : static_cast<double>(scan.value.stamp_ns - frames_[interval.start_frame].stamp_ns) /
                                      static_cast<double>(duration);
  } else {
    alpha = *distance_to_scan / *total_distance;
  }
  scan.value.pose = *predicted * residual.pow(alpha);
  scan.value.committed = true;
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

}  // namespace orb_lidar_mapper
