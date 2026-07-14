#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "orb_lidar_mapper/pose2.hpp"
#include "orb_lidar_mapper/timed_pose_buffer.hpp"

namespace orb_lidar_mapper {

struct Point2 {
  double x{};
  double y{};

  bool operator==(const Point2& rhs) const { return x == rhs.x && y == rhs.y; }
};

struct ScanValue {
  std::uint64_t id{};
  std::int64_t stamp_ns{};
  float angle_min{};
  float angle_increment{};
  float time_increment{};
  float range_min{};
  float range_max{};
  std::vector<float> ranges;
};

struct Ray2 {
  Point2 origin;
  Point2 end;
  bool has_hit{};
};

class ScanDeskewer {
 public:
  static std::optional<std::vector<Ray2>> deskew(const ScanValue& scan,
                                                   const Pose2& committed_scan_base_pose,
                                                   const Pose2& base_to_lidar,
                                                   const TimedPoseBuffer& wheels);
};

}  // namespace orb_lidar_mapper
