#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "orb_lidar_mapper/pose2.hpp"

namespace orb_lidar_mapper {

struct TimedPose2 {
  std::int64_t stamp_ns{};
  Pose2 pose;
};

class TimedPoseBuffer {
 public:
  TimedPoseBuffer(std::int64_t retention_ns, std::int64_t max_gap_ns);

  bool push(TimedPose2 sample);
  std::optional<Pose2> interpolate(std::int64_t stamp_ns) const;
  std::optional<Pose2> relative(std::int64_t from_ns, std::int64_t to_ns) const;
  std::optional<double> maximumYawExcursion(
    std::int64_t from_ns, std::int64_t to_ns) const;
  std::optional<std::int64_t> newestStamp() const noexcept;
  std::size_t size() const noexcept;

 private:
  std::deque<TimedPose2> samples_;
  std::int64_t retention_ns_;
  std::int64_t max_gap_ns_;
};

}  // namespace orb_lidar_mapper
