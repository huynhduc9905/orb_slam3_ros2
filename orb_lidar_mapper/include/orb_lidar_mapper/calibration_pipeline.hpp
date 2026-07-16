#pragma once

#include <array>
#include <filesystem>
#include <vector>

#include "orb_lidar_mapper/calibration_analysis.hpp"
#include "orb_lidar_mapper/rotation_bag_reader.hpp"

namespace orb_lidar_mapper {

struct CalibrationConfig {
  std::filesystem::path bag_path;
  std::filesystem::path output_dir;
  bool overwrite{false};
  double min_abs_omega{0.15};
  double max_abs_omega{0.45};
  double max_abs_linear_speed{0.02};
  double range_cap_m{12.0};
  IcpConfig icp;
};

struct CalibrationRun {
  CalibrationConfig config;
  RotationDataset dataset;
  std::vector<CenterSample> center_samples;
  std::array<MethodEstimate, 3> methods;
  SharpnessResult sharpness;
  AggregateResult aggregate;
};

CalibrationRun runCalibration(const CalibrationConfig&);
int resultExitCode(ResultClass);

}  // namespace orb_lidar_mapper
