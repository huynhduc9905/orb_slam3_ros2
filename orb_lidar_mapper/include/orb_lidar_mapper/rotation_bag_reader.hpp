#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include "orb_lidar_mapper/calibration_types.hpp"

namespace orb_lidar_mapper {

class RotationBagReader {
 public:
  static RotationDataset read(const std::filesystem::path& bag_path);
};

std::vector<MotionInterval> selectStableRotationIntervals(
  const RotationDataset&, double min_abs_omega, double max_abs_omega,
  double max_abs_linear_speed, std::int64_t minimum_duration_ns);

}  // namespace orb_lidar_mapper
