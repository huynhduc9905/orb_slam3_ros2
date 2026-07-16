#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "orb_lidar_mapper/calibration_deskew.hpp"
#include "orb_lidar_mapper/planar_icp.hpp"

namespace orb_lidar_mapper {

struct ScanPair {
  std::size_t source_index{};
  std::size_t target_index{};
  std::size_t yaw_sector{};
  double odom_yaw_delta_rad{};
};

struct CenterSample {
  DeskewMethod method{};
  std::uint64_t source_scan_id{};
  std::uint64_t target_scan_id{};
  std::size_t yaw_sector{};
  bool accepted{};
  Point2 center;
  IcpResult icp;
  std::string rejection_reason;
};

std::vector<ScanPair> selectCalibrationPairs(
    const std::vector<DeskewedScan>& scans, const TimedPoseBuffer& odom,
    const std::vector<MotionInterval>& stable_intervals, double min_yaw,
    double max_yaw);

std::optional<Point2> centerFromTransform(const Pose2& source_to_target);

CenterSample estimateRotationCenter(
    DeskewMethod method, const ScanPair& pair, const DeskewedScan& source,
    const DeskewedScan& target, const PlanarIcp& icp, double max_center_x,
    double max_abs_center_y);

}  // namespace orb_lidar_mapper
