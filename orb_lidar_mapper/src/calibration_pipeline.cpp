#include "orb_lidar_mapper/calibration_pipeline.hpp"

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "orb_lidar_mapper/calibration_deskew.hpp"

namespace orb_lidar_mapper {
namespace {

constexpr std::int64_t kMinimumStableIntervalNs = 2'000'000'000LL;
constexpr std::int64_t kPoseMaximumGapNs = 1'000'000'000LL;
constexpr std::int64_t kAssociationDeviationNs = 100'000'000LL;
constexpr double kPairMinYaw = 10.0 * kPi / 180.0;
constexpr double kPairMaxYaw = 30.0 * kPi / 180.0;
constexpr double kImuStartingOffset = 0.260;
constexpr double kImuConvergenceThreshold = 0.0005;
constexpr std::size_t kMaximumImuIterations = 8;

using ScanMap = std::unordered_map<std::uint64_t, std::size_t>;

TimedPoseBuffer makePoseBuffer(const RotationDataset& dataset) {
  if (dataset.odom_poses.empty()) throw std::runtime_error("no odometry samples");
  const auto retention = dataset.odom_poses.back().stamp_ns -
                         dataset.odom_poses.front().stamp_ns + kPoseMaximumGapNs;
  TimedPoseBuffer buffer(std::max<std::int64_t>(retention, kPoseMaximumGapNs),
                         kPoseMaximumGapNs);
  for (const auto& sample : dataset.odom_poses) {
    if (!buffer.push(sample)) throw std::runtime_error("odometry timestamps are not monotonic");
  }
  return buffer;
}

std::vector<DeskewedScan> deskewOdom(const RotationDataset& dataset,
                                     const TimedPoseBuffer& odom,
                                     double range_cap_m) {
  std::vector<DeskewedScan> result;
  for (const auto& scan : dataset.raw_scans) {
    const auto deskewed = deskewWithOdom(scan, odom, dataset.recorded_mount, range_cap_m);
    if (deskewed && !deskewed->points.empty()) result.push_back(*deskewed);
  }
  return result;
}

std::vector<DeskewedScan> deskewExisting(const RotationDataset& dataset,
                                         double range_cap_m) {
  const auto associations = associateUndistortedScans(
    dataset.raw_scans, dataset.undistorted_scans, kAssociationDeviationNs);
  if (associations.empty()) return {};
  ScanMap undistorted;
  for (std::size_t index = 0; index < dataset.undistorted_scans.size(); ++index) {
    undistorted[dataset.undistorted_scans[index].id] = index;
  }
  std::vector<DeskewedScan> result;
  for (const auto& association : associations) {
    const auto it = undistorted.find(association.undistorted_scan_id);
    if (it == undistorted.end()) continue;
    auto scan = adaptUndistortedScan(dataset.undistorted_scans[it->second], association, range_cap_m);
    if (!scan.points.empty()) result.push_back(std::move(scan));
  }
  return result;
}

std::vector<DeskewedScan> deskewImu(const RotationDataset& dataset,
                                    double offset, double range_cap_m) {
  std::vector<DeskewedScan> result;
  for (const auto& scan : dataset.raw_scans) {
    const auto deskewed = deskewWithImu(
      scan, dataset.imu_yaw_rates, offset, kPi, range_cap_m);
    if (deskewed && !deskewed->points.empty()) result.push_back(*deskewed);
  }
  return result;
}

ScanMap indexById(const std::vector<DeskewedScan>& scans) {
  ScanMap result;
  for (std::size_t index = 0; index < scans.size(); ++index) result[scans[index].scan_id] = index;
  return result;
}

std::vector<CenterSample> estimateSamples(
  DeskewMethod method, const std::vector<DeskewedScan>& scans,
  const std::vector<ScanPair>& schedule, const std::vector<std::uint64_t>& base_ids,
  const PlanarIcp& icp, double max_center_x, double max_abs_center_y) {
  std::vector<CenterSample> result;
  const auto indices = indexById(scans);
  result.reserve(schedule.size());
  for (const auto& scheduled : schedule) {
    CenterSample sample;
    sample.method = method;
    sample.yaw_sector = scheduled.yaw_sector;
    sample.icp.rejection_reason = "preprocessing_failed";
    if (scheduled.source_index >= base_ids.size() || scheduled.target_index >= base_ids.size()) {
      sample.accepted = false;
      sample.rejection_reason = "invalid_common_schedule";
      result.push_back(std::move(sample));
      continue;
    }
    sample.source_scan_id = base_ids[scheduled.source_index];
    sample.target_scan_id = base_ids[scheduled.target_index];
    const auto source = indices.find(sample.source_scan_id);
    const auto target = indices.find(sample.target_scan_id);
    if (source == indices.end() || target == indices.end()) {
      sample.accepted = false;
      sample.rejection_reason = "preprocessing_failed";
      result.push_back(std::move(sample));
      continue;
    }
    ScanPair pair{source->second, target->second, scheduled.yaw_sector,
                  scheduled.expected_source_to_target_yaw_rad};
    sample = estimateRotationCenter(
      method, pair, scans[source->second], scans[target->second], icp,
      max_center_x, max_abs_center_y);
    result.push_back(std::move(sample));
  }
  return result;
}

std::vector<std::uint64_t> orderedIds(const std::vector<DeskewedScan>& scans) {
  std::vector<std::uint64_t> result;
  result.reserve(scans.size());
  for (const auto& scan : scans) result.push_back(scan.scan_id);
  return result;
}

MethodEstimate estimateMethod(const std::vector<CenterSample>& samples,
                              DeskewMethod method) {
  return robustMethodEstimate(method, samples);
}

void validateCalibrationConfig(const CalibrationConfig& config) {
  if (config.bag_path.empty() || config.output_dir.empty() ||
      !std::isfinite(config.range_cap_m) || config.range_cap_m < 0.0 ||
      !std::isfinite(config.min_abs_omega) || !std::isfinite(config.max_abs_omega) ||
      !std::isfinite(config.max_abs_linear_speed)) {
    throw std::invalid_argument("invalid calibration configuration");
  }
}

}  // namespace

std::vector<ScanPair> selectCommonCalibrationSchedule(
  const std::vector<DeskewedScan>& scans, const TimedPoseBuffer& odom,
  const std::vector<MotionInterval>& stable_intervals, std::size_t maximum_pairs) {
  if (maximum_pairs == 0) return {};
  const auto candidates = selectCalibrationPairs(
    scans, odom, stable_intervals, kPairMinYaw, kPairMaxYaw);
  std::array<std::vector<ScanPair>, 8> by_sector;
  for (const auto& candidate : candidates) {
    if (candidate.yaw_sector < by_sector.size()) by_sector[candidate.yaw_sector].push_back(candidate);
  }
  std::vector<ScanPair> result;
  result.reserve(std::min(maximum_pairs, candidates.size()));
  std::array<std::size_t, 8> next{};
  while (result.size() < maximum_pairs) {
    bool added = false;
    for (std::size_t sector = 0; sector < by_sector.size() && result.size() < maximum_pairs; ++sector) {
      if (next[sector] >= by_sector[sector].size()) continue;
      result.push_back(by_sector[sector][next[sector]++]);
      added = true;
    }
    if (!added) break;
  }
  return result;
}

double preliminarySharpnessHint(
  const std::array<MethodEstimate, 3>& methods, double fallback_offset_m) {
  std::vector<double> reliable_offsets;
  for (const auto& method : methods) {
    if (method.reliable && std::isfinite(method.forward_offset_m)) {
      reliable_offsets.push_back(method.forward_offset_m);
    }
  }
  if (reliable_offsets.empty()) return fallback_offset_m;
  std::sort(reliable_offsets.begin(), reliable_offsets.end());
  const std::size_t middle = reliable_offsets.size() / 2;
  if (reliable_offsets.size() % 2 == 0) {
    return (reliable_offsets[middle - 1] + reliable_offsets[middle]) / 2.0;
  }
  return reliable_offsets[middle];
}

MethodEstimate finalizeNonConvergedImuEstimate(std::vector<CenterSample>& samples) {
  for (auto& sample : samples) {
    if (sample.method == DeskewMethod::kImu && sample.accepted) {
      sample.accepted = false;
      sample.rejection_reason = "imu_offset_not_converged";
    }
  }
  auto estimate = estimateMethod(samples, DeskewMethod::kImu);
  estimate.reliable = false;
  return estimate;
}

CalibrationRun runCalibrationDataset(const CalibrationConfig& config,
                                     CalibrationPreparedDataset prepared) {
  validateCalibrationConfig(config);
  CalibrationRun run;
  run.config = config;
  run.dataset = std::move(prepared.dataset);
  const auto odom = makePoseBuffer(run.dataset);
  const auto intervals = selectStableRotationIntervals(
    run.dataset, config.min_abs_omega, config.max_abs_omega,
    config.max_abs_linear_speed, kMinimumStableIntervalNs);
  if (intervals.empty()) throw std::runtime_error("no stable rotation interval");

  const auto odom_scans = std::move(prepared.odom_scans);
  if (odom_scans.size() < 3) throw std::runtime_error("no usable odometry-deskewed scans");
  const auto schedule = selectCommonCalibrationSchedule(odom_scans, odom, intervals);
  if (schedule.empty()) throw std::runtime_error("no calibration scan pairs");
  const auto base_ids = orderedIds(odom_scans);
  PlanarIcp icp(config.icp);

  const auto odom_samples = estimateSamples(
    DeskewMethod::kOdom, odom_scans, schedule, base_ids, icp, 1.0, 0.25);
  const auto existing_scans = std::move(prepared.existing_scans);
  const auto existing_samples = estimateSamples(
    DeskewMethod::kExistingScan, existing_scans, schedule, base_ids, icp, 1.0, 0.25);

  double imu_offset = kImuStartingOffset;
  std::vector<CenterSample> imu_samples;
  MethodEstimate imu_estimate;
  bool imu_converged = false;
  for (std::size_t iteration = 0; iteration < kMaximumImuIterations; ++iteration) {
    ++run.imu_iterations;
    std::vector<DeskewedScan> imu_scans;
    if (prepared.imu_scans_by_iteration.empty()) {
      imu_scans = deskewImu(run.dataset, imu_offset, config.range_cap_m);
    } else {
      imu_scans = prepared.imu_scans_by_iteration[
        std::min(iteration, prepared.imu_scans_by_iteration.size() - 1)];
    }
    imu_samples = estimateSamples(
      DeskewMethod::kImu, imu_scans, schedule, base_ids, icp, 1.0, 0.25);
    imu_estimate = estimateMethod(imu_samples, DeskewMethod::kImu);
    if (imu_estimate.accepted_pairs == 0 || !std::isfinite(imu_estimate.forward_offset_m)) break;
    const double updated = imu_estimate.forward_offset_m;
    if (std::abs(updated - imu_offset) < kImuConvergenceThreshold) {
      imu_offset = updated;
      imu_converged = true;
      break;
    }
    imu_offset = updated;
  }
  if (!imu_converged) {
    imu_estimate = finalizeNonConvergedImuEstimate(imu_samples);
  }

  const auto existing_estimate = estimateMethod(existing_samples, DeskewMethod::kExistingScan);
  const auto odom_estimate = estimateMethod(odom_samples, DeskewMethod::kOdom);
  run.methods = {odom_estimate, imu_estimate, existing_estimate};
  run.center_samples.reserve(odom_samples.size() + imu_samples.size() + existing_samples.size());
  run.center_samples.insert(run.center_samples.end(), odom_samples.begin(), odom_samples.end());
  run.center_samples.insert(run.center_samples.end(), imu_samples.begin(), imu_samples.end());
  run.center_samples.insert(run.center_samples.end(), existing_samples.begin(), existing_samples.end());
  TimedPoseBuffer sharpness_odom = odom;
  if (!prepared.sharpness_odom_poses.empty()) {
    const auto retention = prepared.sharpness_odom_poses.back().stamp_ns -
      prepared.sharpness_odom_poses.front().stamp_ns + kPoseMaximumGapNs;
    sharpness_odom = TimedPoseBuffer(std::max<std::int64_t>(retention, kPoseMaximumGapNs),
                                     kPoseMaximumGapNs);
    for (const auto& pose : prepared.sharpness_odom_poses) sharpness_odom.push(pose);
  }
  run.sharpness = evaluateMapSharpness(
    run.dataset, odom_scans, sharpness_odom, preliminarySharpnessHint(run.methods));
  run.aggregate = classifyCalibration(
    {run.methods[0], run.methods[1], run.methods[2]}, run.sharpness,
    run.dataset.recorded_mount.x_m);
  return run;
}

CalibrationRun runCalibrationDataset(const CalibrationConfig& config,
                                     RotationDataset dataset) {
  validateCalibrationConfig(config);
  CalibrationPreparedDataset prepared;
  prepared.dataset = std::move(dataset);
  const auto odom = makePoseBuffer(prepared.dataset);
  prepared.odom_scans = deskewOdom(prepared.dataset, odom, config.range_cap_m);
  prepared.existing_scans = deskewExisting(prepared.dataset, config.range_cap_m);
  return runCalibrationDataset(config, std::move(prepared));
}

CalibrationRun runCalibration(const CalibrationConfig& config) {
  validateCalibrationConfig(config);
  auto dataset = RotationBagReader::read(config.bag_path);
  return runCalibrationDataset(config, std::move(dataset));
}

int resultExitCode(ResultClass result) {
  switch (result) {
    case ResultClass::kConsistent: return 0;
    case ResultClass::kLikelyOffsetError: return 2;
    case ResultClass::kInconclusive: return 3;
  }
  return 3;
}

}  // namespace orb_lidar_mapper
