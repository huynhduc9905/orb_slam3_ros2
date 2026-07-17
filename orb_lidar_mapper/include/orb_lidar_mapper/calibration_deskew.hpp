#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "orb_lidar_mapper/calibration_types.hpp"

namespace orb_lidar_mapper {

struct DeskewedScan {
  std::uint64_t scan_id{};
  std::int64_t reference_stamp_ns{};
  DeskewMethod method{};
  std::vector<Point2> points;
};

struct ScanAssociation {
  std::uint64_t raw_scan_id{};
  std::uint64_t undistorted_scan_id{};
  std::int64_t timestamp_delay_ns{};
};

std::optional<DeskewedScan> deskewWithOdom(
    const ScanValue&, const TimedPoseBuffer&, const StaticLidarMount&, double range_cap_m);

std::optional<DeskewedScan> deskewWithImu(
    const ScanValue&, const std::vector<TimedYawRate>&, double candidate_offset_m,
    double fixed_yaw_rad, double range_cap_m);

std::vector<ScanAssociation> associateUndistortedScans(
    const std::vector<ScanValue>& raw, const std::vector<ScanValue>& undistorted,
    std::int64_t maximum_delay_deviation_ns);

DeskewedScan adaptUndistortedScan(
    const ScanValue&, const ScanAssociation&, double range_cap_m);

}  // namespace orb_lidar_mapper
