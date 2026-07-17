#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "orb_lidar_mapper/calibration_deskew.hpp"
#include "orb_lidar_mapper/calibration_types.hpp"
#include "orb_lidar_mapper/rotation_center_estimator.hpp"

namespace orb_lidar_mapper {

struct ConfidenceInterval {
  double low_m{};
  double high_m{};
};

struct MethodEstimate {
  DeskewMethod method{};
  bool reliable{};
  double center_x_m{};
  double center_y_m{};
  double forward_offset_m{};
  ConfidenceInterval confidence_95_m;
  std::size_t accepted_pairs{};
  std::size_t attempted_pairs{};
  std::size_t covered_yaw_sectors{};
  double median_rmse_m{};
  double median_overlap{};
  std::map<std::string, std::size_t> rejection_counts;
};

struct SharpnessPoint {
  double offset_m{};
  double score{};
};

struct SharpnessPair {
  std::size_t source_index{};
  std::size_t target_index{};
};

struct SharpnessResult {
  bool reliable{};
  double best_offset_m{};
  std::vector<SharpnessPoint> coarse;
  std::vector<SharpnessPoint> refined;
  std::string rejection_reason;
};

struct AggregateResult {
  ResultClass classification{ResultClass::kInconclusive};
  double consensus_offset_m{};
  ConfidenceInterval confidence_95_m;
  std::string reason;
};

MethodEstimate robustMethodEstimate(
  DeskewMethod method, const std::vector<CenterSample>& samples,
  std::uint64_t seed = 0x4f52424c49444152ULL);

SharpnessResult evaluateMapSharpness(
    const RotationDataset&, const std::vector<DeskewedScan>&,
    const TimedPoseBuffer&, double consensus_hint);

std::vector<SharpnessPair> selectSharpnessPairs(
    const std::vector<DeskewedScan>&, const TimedPoseBuffer&,
    std::size_t maximum_pairs = 128);

bool sharpnessMinimumIsUnique(const std::vector<SharpnessPoint>&);

AggregateResult classifyCalibration(
  const std::vector<MethodEstimate>&, const SharpnessResult&,
  double recorded_offset_m);

}  // namespace orb_lidar_mapper
