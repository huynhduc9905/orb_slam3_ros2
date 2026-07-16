#include "orb_lidar_mapper/rotation_center_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/LU>

namespace orb_lidar_mapper {
namespace {

bool inInterval(std::int64_t stamp, const MotionInterval& interval) {
  return stamp >= interval.start_ns && stamp <= interval.end_ns;
}

std::size_t sectorFor(double progress) {
  const auto sector = static_cast<std::size_t>(std::floor(progress * 8.0));
  return std::min<std::size_t>(sector, 7U);
}

}  // namespace

std::optional<Point2> centerFromTransform(const Pose2& source_to_target) {
  const double c = std::cos(source_to_target.yaw);
  const double s = std::sin(source_to_target.yaw);
  Eigen::Matrix2d matrix;
  matrix << 1.0 - c, s, -s, 1.0 - c;
  if (!std::isfinite(matrix.determinant()) ||
      std::abs(matrix.determinant()) < 1e-4) return std::nullopt;
  const Eigen::Vector2d translation(source_to_target.x, source_to_target.y);
  const Eigen::Vector2d center = matrix.fullPivLu().solve(translation);
  if (!center.allFinite()) return std::nullopt;
  return Point2{center.x(), center.y()};
}

std::vector<ScanPair> selectCalibrationPairs(
    const std::vector<DeskewedScan>& scans, const TimedPoseBuffer& odom,
    const std::vector<MotionInterval>& stable_intervals, double min_yaw,
    double max_yaw) {
  std::vector<ScanPair> pairs;
  if (scans.size() < 2U || min_yaw > max_yaw || min_yaw < 0.0) return pairs;
  for (std::size_t source = 0; source < scans.size(); ++source) {
    for (std::size_t target = source + 1; target < scans.size(); ++target) {
      if (scans[source].method != scans[target].method) continue;
      const auto source_pose = odom.interpolate(scans[source].reference_stamp_ns);
      const auto target_pose = odom.interpolate(scans[target].reference_stamp_ns);
      if (!source_pose || !target_pose) continue;
      const auto interval = std::find_if(
          stable_intervals.begin(), stable_intervals.end(), [&](const auto& candidate) {
            return inInterval(scans[source].reference_stamp_ns, candidate) &&
                   inInterval(scans[target].reference_stamp_ns, candidate);
          });
      if (interval == stable_intervals.end()) continue;
      const double delta = Pose2::normalizeAngle(target_pose->yaw - source_pose->yaw);
      const double absolute_delta = std::abs(delta);
      if (absolute_delta < min_yaw || absolute_delta > max_yaw) continue;
      const auto interval_start = odom.interpolate(interval->start_ns);
      const auto interval_end = odom.interpolate(interval->end_ns);
      if (!interval_start || !interval_end) continue;
      const double interval_yaw = Pose2::normalizeAngle(
          interval_end->yaw - interval_start->yaw);
      if (std::abs(interval_yaw) < std::numeric_limits<double>::epsilon()) continue;
      const double progress = std::clamp(
          Pose2::normalizeAngle(source_pose->yaw - interval_start->yaw) /
              interval_yaw,
          0.0, 1.0);
      pairs.push_back({source, target, sectorFor(progress), delta});
    }
  }
  return pairs;
}

CenterSample estimateRotationCenter(
    DeskewMethod method, const ScanPair& pair, const DeskewedScan& source,
    const DeskewedScan& target, const PlanarIcp& icp, double max_center_x,
    double max_abs_center_y) {
  CenterSample sample;
  sample.method = method;
  sample.source_scan_id = source.scan_id;
  sample.target_scan_id = target.scan_id;
  sample.yaw_sector = pair.yaw_sector;
  sample.icp = icp.align(source.points, target.points, pair.odom_yaw_delta_rad);
  if (!sample.icp.converged) {
    sample.rejection_reason = sample.icp.rejection_reason;
    return sample;
  }
  const auto center = centerFromTransform(sample.icp.source_to_target);
  if (!center) {
    sample.rejection_reason = "singular_center_transform";
    return sample;
  }
  sample.center = *center;
  if (!std::isfinite(center->x) || !std::isfinite(center->y)) {
    sample.rejection_reason = "nonfinite_center";
    return sample;
  }
  if (center->x < 0.0 || center->x > max_center_x ||
      std::abs(center->y) > max_abs_center_y) {
    sample.rejection_reason = "implausible_center";
    return sample;
  }
  sample.accepted = true;
  return sample;
}

}  // namespace orb_lidar_mapper
