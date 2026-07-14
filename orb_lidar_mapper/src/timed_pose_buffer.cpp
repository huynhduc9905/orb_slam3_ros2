#include "orb_lidar_mapper/timed_pose_buffer.hpp"

namespace orb_lidar_mapper {

TimedPoseBuffer::TimedPoseBuffer(std::int64_t retention_ns, std::int64_t max_gap_ns)
: retention_ns_(retention_ns), max_gap_ns_(max_gap_ns) {}

bool TimedPoseBuffer::push(TimedPose2 sample) {
  if (!samples_.empty() && sample.stamp_ns < samples_.back().stamp_ns) {
    return false;
  }

  samples_.push_back(sample);
  const std::int64_t oldest_allowed = sample.stamp_ns - retention_ns_;
  while (!samples_.empty() && samples_.front().stamp_ns < oldest_allowed) {
    samples_.pop_front();
  }
  return true;
}

std::optional<Pose2> TimedPoseBuffer::interpolate(std::int64_t stamp_ns) const {
  if (samples_.empty() || stamp_ns < samples_.front().stamp_ns ||
      stamp_ns > samples_.back().stamp_ns) {
    return std::nullopt;
  }

  for (std::size_t index = 0; index < samples_.size(); ++index) {
    const TimedPose2& later = samples_[index];
    if (later.stamp_ns == stamp_ns) {
      return later.pose;
    }
    if (later.stamp_ns > stamp_ns) {
      const TimedPose2& earlier = samples_[index - 1];
      const std::int64_t before_ns = stamp_ns - earlier.stamp_ns;
      const std::int64_t after_ns = later.stamp_ns - stamp_ns;
      const std::int64_t gap_ns = later.stamp_ns - earlier.stamp_ns;
      if (before_ns > max_gap_ns_ || after_ns > max_gap_ns_ || gap_ns > max_gap_ns_) {
        return std::nullopt;
      }
      const double alpha = static_cast<double>(before_ns) / static_cast<double>(gap_ns);
      return Pose2::interpolate(earlier.pose, later.pose, alpha);
    }
  }

  return std::nullopt;
}

std::optional<Pose2> TimedPoseBuffer::relative(
  std::int64_t from_ns, std::int64_t to_ns) const {
  const auto from = interpolate(from_ns);
  const auto to = interpolate(to_ns);
  if (!from || !to) {
    return std::nullopt;
  }
  return from->inverse() * *to;
}

std::size_t TimedPoseBuffer::size() const noexcept {
  return samples_.size();
}

}  // namespace orb_lidar_mapper
