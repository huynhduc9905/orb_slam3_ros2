#include "orb_lidar_mapper/scan_deskewer.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace orb_lidar_mapper {
namespace {

std::optional<std::int64_t> rayStamp(const ScanValue& scan, std::size_t index) {
  const long double offset_ns = static_cast<long double>(index) *
                                static_cast<long double>(scan.time_increment) * 1'000'000'000.0L;
  if (!std::isfinite(offset_ns) ||
      offset_ns < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      offset_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  const long double rounded = std::round(offset_ns);
  const long double stamp = static_cast<long double>(scan.stamp_ns) + rounded;
  if (stamp < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      stamp > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(stamp);
}

Point2 transformPoint(const Pose2& pose, double x, double y) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {pose.x + cosine * x - sine * y, pose.y + sine * x + cosine * y};
}

}  // namespace

std::optional<std::vector<Ray2>> ScanDeskewer::deskew(
  const ScanValue& scan, const Pose2& committed_scan_base_pose, const Pose2& base_to_lidar,
  const TimedPoseBuffer& wheels) {
  if (!std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) || scan.range_min < 0.0F ||
      scan.range_min > scan.range_max || !std::isfinite(scan.angle_min) ||
      !std::isfinite(scan.angle_increment) || !std::isfinite(scan.time_increment) ||
      scan.time_increment < 0.0F) {
    return std::nullopt;
  }

  std::vector<Ray2> rays;
  rays.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const float range = scan.ranges[index];
    if (std::isnan(range) || range < scan.range_min ||
        (std::isfinite(range) && range > scan.range_max) || range == -std::numeric_limits<float>::infinity()) {
      continue;
    }
    const double angle = static_cast<double>(scan.angle_min) +
                         static_cast<double>(index) * static_cast<double>(scan.angle_increment);
    if (!std::isfinite(angle)) {
      continue;
    }
    const auto stamp_ns = rayStamp(scan, index);
    if (!stamp_ns) {
      return std::nullopt;
    }
    const auto relative_wheel = wheels.relative(scan.stamp_ns, *stamp_ns);
    if (!relative_wheel) {
      return std::nullopt;
    }

    const Pose2 lidar_pose = committed_scan_base_pose * *relative_wheel * base_to_lidar;
    const double direction_x = std::cos(lidar_pose.yaw + angle);
    const double direction_y = std::sin(lidar_pose.yaw + angle);
    const Point2 origin{lidar_pose.x, lidar_pose.y};
    if (std::isinf(range)) {
      rays.push_back({origin, {origin.x + static_cast<double>(scan.range_max) * direction_x,
                               origin.y + static_cast<double>(scan.range_max) * direction_y}, false});
    } else {
      rays.push_back({origin, transformPoint(lidar_pose, std::cos(angle) * range,
                                               std::sin(angle) * range), true});
    }
  }
  return rays;
}

}  // namespace orb_lidar_mapper
