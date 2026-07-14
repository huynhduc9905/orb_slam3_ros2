#include "orb_lidar_mapper/timed_pose_buffer.hpp"

#include <cstdint>
#include <stdexcept>

namespace orb_lidar_mapper {
namespace {

std::uint64_t orderedDuration(std::int64_t earlier_ns, std::int64_t later_ns) {
  return static_cast<std::uint64_t>(later_ns) - static_cast<std::uint64_t>(earlier_ns);
}

}  // namespace

TimedPoseBuffer::TimedPoseBuffer(std::int64_t retention_ns, std::int64_t max_gap_ns)
: retention_ns_(retention_ns), max_gap_ns_(max_gap_ns) {
  if (retention_ns < 0 || max_gap_ns < 0) {
    throw std::invalid_argument("TimedPoseBuffer durations must be non-negative");
  }
}

bool TimedPoseBuffer::push(TimedPose2 sample) {
  if (!samples_.empty() && sample.stamp_ns < samples_.back().stamp_ns) {
    return false;
  }

  if (!samples_.empty() && sample.stamp_ns == samples_.back().stamp_ns) {
    samples_.back() = sample;
    return true;
  }

  samples_.push_back(sample);
  const auto retention_ns = static_cast<std::uint64_t>(retention_ns_);
  while (!samples_.empty() &&
         orderedDuration(samples_.front().stamp_ns, sample.stamp_ns) > retention_ns) {
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
      const auto before_ns = orderedDuration(earlier.stamp_ns, stamp_ns);
      const auto after_ns = orderedDuration(stamp_ns, later.stamp_ns);
      const auto gap_ns = orderedDuration(earlier.stamp_ns, later.stamp_ns);
      const auto max_gap_ns = static_cast<std::uint64_t>(max_gap_ns_);
      if (before_ns > max_gap_ns || after_ns > max_gap_ns || gap_ns > max_gap_ns) {
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
