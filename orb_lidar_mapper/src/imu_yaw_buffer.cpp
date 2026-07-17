#include "orb_lidar_mapper/imu_yaw_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace orb_lidar_mapper {
namespace {

std::uint64_t orderedDuration(std::int64_t earlier_ns, std::int64_t later_ns) {
  return static_cast<std::uint64_t>(later_ns) - static_cast<std::uint64_t>(earlier_ns);
}

double interpolatedRate(const TimedYawRate& earlier, const TimedYawRate& later,
                        std::int64_t stamp_ns) {
  if (stamp_ns == earlier.stamp_ns) {
    return earlier.omega_rad_s;
  }
  if (stamp_ns == later.stamp_ns) {
    return later.omega_rad_s;
  }
  const double alpha = static_cast<double>(stamp_ns - earlier.stamp_ns) /
                       static_cast<double>(later.stamp_ns - earlier.stamp_ns);
  return earlier.omega_rad_s + alpha * (later.omega_rad_s - earlier.omega_rad_s);
}

}  // namespace

ImuYawBuffer::ImuYawBuffer(std::int64_t retention_ns, std::int64_t max_gap_ns)
: retention_ns_(retention_ns), max_gap_ns_(max_gap_ns) {
  if (retention_ns < 0 || max_gap_ns < 0) {
    throw std::invalid_argument("ImuYawBuffer durations must be non-negative");
  }
}

bool ImuYawBuffer::push(TimedYawRate sample) {
  if (!std::isfinite(sample.omega_rad_s)) {
    return false;
  }
  if (!samples_.empty() && sample.stamp_ns < samples_.back().stamp_ns) {
    return false;
  }

  if (!samples_.empty() && sample.stamp_ns == samples_.back().stamp_ns) {
    // Running average of contiguous duplicate timestamps.
    ++last_duplicate_count_;
    const double n = static_cast<double>(last_duplicate_count_);
    samples_.back().omega_rad_s =
      samples_.back().omega_rad_s +
      (sample.omega_rad_s - samples_.back().omega_rad_s) / n;
    return true;
  }

  samples_.push_back(sample);
  last_duplicate_count_ = 1;
  const auto retention_ns = static_cast<std::uint64_t>(retention_ns_);
  while (!samples_.empty() &&
         orderedDuration(samples_.front().stamp_ns, sample.stamp_ns) > retention_ns) {
    samples_.pop_front();
  }
  return true;
}

bool ImuYawBuffer::covers(std::int64_t start_ns, std::int64_t end_ns) const {
  if (end_ns < start_ns || samples_.empty() ||
      samples_.front().stamp_ns > start_ns || samples_.back().stamp_ns < end_ns) {
    return false;
  }
  const auto max_gap_ns = static_cast<std::uint64_t>(max_gap_ns_);
  for (std::size_t index = 1; index < samples_.size(); ++index) {
    const auto& earlier = samples_[index - 1];
    const auto& later = samples_[index];
    // Only enforce max gap for segments that overlap the query interval.
    if (later.stamp_ns > start_ns && earlier.stamp_ns < end_ns &&
        orderedDuration(earlier.stamp_ns, later.stamp_ns) > max_gap_ns) {
      return false;
    }
  }
  return true;
}

std::optional<double> ImuYawBuffer::integratedYaw(std::int64_t start_ns,
                                                  std::int64_t target_ns) const {
  if (target_ns < start_ns || !covers(start_ns, target_ns)) {
    return std::nullopt;
  }
  if (target_ns == start_ns) {
    return 0.0;
  }

  double yaw = 0.0;
  std::int64_t previous = start_ns;
  double previous_rate = 0.0;
  std::size_t segment = 1;
  while (segment < samples_.size() && samples_[segment].stamp_ns <= start_ns) {
    ++segment;
  }
  if (segment == samples_.size()) {
    previous_rate = samples_.back().omega_rad_s;
  } else {
    previous_rate = interpolatedRate(samples_[segment - 1], samples_[segment], start_ns);
  }

  while (previous < target_ns) {
    const std::int64_t next = segment < samples_.size()
      ? std::min(target_ns, samples_[segment].stamp_ns)
      : target_ns;
    const double next_rate = segment < samples_.size()
      ? interpolatedRate(samples_[segment - 1], samples_[segment], next)
      : previous_rate;
    yaw += 0.5 * (previous_rate + next_rate) *
           static_cast<double>(next - previous) / 1'000'000'000.0;
    previous = next;
    previous_rate = next_rate;
    while (segment < samples_.size() && samples_[segment].stamp_ns <= previous) {
      ++segment;
    }
  }
  return yaw;
}

std::size_t ImuYawBuffer::size() const noexcept {
  return samples_.size();
}

}  // namespace orb_lidar_mapper
