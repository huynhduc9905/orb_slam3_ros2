#pragma once

#include <cstdint>
#include <vector>

#include "orb_lidar_mapper/pose2.hpp"
#include "orb_lidar_mapper/scan_deskewer.hpp"
#include "orb_lidar_mapper/timed_pose_buffer.hpp"

namespace orb_lidar_mapper {

enum class DeskewMethod { kOdom, kImu, kExistingScan };
enum class ResultClass { kConsistent, kLikelyOffsetError, kInconclusive };
inline constexpr double kPi = 3.14159265358979323846;

struct TimedTwist2 {
  std::int64_t stamp_ns{};
  Twist2 twist;
};

struct TimedYawRate {
  std::int64_t stamp_ns{};
  double omega_rad_s{};
};

struct StaticLidarMount {
  double x_m{};
  double y_m{};
  double z_m{};
  double yaw_rad{};
};

struct RotationDataset {
  std::vector<ScanValue> raw_scans;
  std::vector<ScanValue> undistorted_scans;
  std::vector<TimedPose2> odom_poses;
  std::vector<TimedTwist2> odom_twists;
  std::vector<TimedYawRate> imu_yaw_rates;
  StaticLidarMount recorded_mount;
};

struct MotionInterval {
  std::int64_t start_ns{};
  std::int64_t end_ns{};
};

}  // namespace orb_lidar_mapper
