#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "orb_lidar_mapper/imu_yaw_buffer.hpp"
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

struct ScanMotionBracket {
  std::int64_t start_stamp_ns{};
  std::int64_t end_stamp_ns{};
  Pose2 start_map_pose;
  Pose2 end_map_pose;
  Pose2 start_wheel_pose;
  Pose2 end_wheel_pose;
};

struct RayMotion2 {
  Pose2 wheel_pose;
  double alpha{};
  Point2 lidar_end;
  bool has_hit{};
};

struct BracketedDeskewResult {
  std::vector<Ray2> rays;
  std::vector<RayMotion2> ray_motions;
};

class ScanDeskewer {
 public:
  static std::optional<std::vector<Ray2>> deskew(const ScanValue& scan,
                                                   const Pose2& committed_scan_base_pose,
                                                   const Pose2& base_to_lidar,
                                                   const TimedPoseBuffer& wheels,
                                                   const ImuYawBuffer* imu = nullptr);
  static std::optional<BracketedDeskewResult> deskewBracketed(
    const ScanValue& scan, const Pose2& base_to_lidar,
    const TimedPoseBuffer& wheels, const ScanMotionBracket& bracket,
    const ImuYawBuffer* imu = nullptr);
};

}  // namespace orb_lidar_mapper
