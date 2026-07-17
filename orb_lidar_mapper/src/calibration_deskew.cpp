#include "orb_lidar_mapper/calibration_deskew.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <limits>

namespace orb_lidar_mapper {
namespace {

constexpr std::int64_t kMaximumImuGapNs = 20'000'000LL;
constexpr double kMountTolerance = 1e-9;

bool validScanMetadata(const ScanValue& scan) {
  return std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment) &&
         std::isfinite(scan.time_increment) && scan.time_increment >= 0.0F &&
         std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
         scan.range_max >= scan.range_min &&
         !scan.ranges.empty();
}

bool validMount(const StaticLidarMount& mount) {
  return std::isfinite(mount.x_m) && std::isfinite(mount.y_m) &&
         std::isfinite(mount.z_m) && std::isfinite(mount.yaw_rad) &&
         std::abs(mount.y_m) <= kMountTolerance &&
         std::abs(Pose2::normalizeAngle(mount.yaw_rad - kPi)) <= kMountTolerance;
}

bool validRange(double range, const ScanValue& scan, double range_cap_m) {
  if (!std::isfinite(range) || !std::isfinite(range_cap_m) || range_cap_m < 0.0) {
    return false;
  }
  const double lower = std::max(static_cast<double>(scan.range_min), 0.15);
  const double upper = std::min(static_cast<double>(scan.range_max), range_cap_m);
  return lower <= upper && range >= lower && range <= upper;
}

std::optional<std::int64_t> checkedAdd(std::int64_t left, std::int64_t right) {
  if ((right > 0 && left > std::numeric_limits<std::int64_t>::max() - right) ||
      (right < 0 && left < std::numeric_limits<std::int64_t>::min() - right)) {
    return std::nullopt;
  }
  return left + right;
}

std::optional<std::int64_t> checkedSubtract(std::int64_t left, std::int64_t right) {
  if ((right > 0 && left < std::numeric_limits<std::int64_t>::min() + right) ||
      (right < 0 && left > std::numeric_limits<std::int64_t>::max() + right)) {
    return std::nullopt;
  }
  return left - right;
}

std::optional<std::int64_t> rayStamp(const ScanValue& scan, std::size_t index) {
  std::array<char, 32> text{};
  const auto formatted = std::to_chars(
      text.data(), text.data() + text.size(), scan.time_increment);
  if (formatted.ec != std::errc{}) {
    return std::nullopt;
  }
  double canonical_increment{};
  const auto parsed = std::from_chars(text.data(), formatted.ptr, canonical_increment);
  if (parsed.ec != std::errc{} || parsed.ptr != formatted.ptr) {
    return std::nullopt;
  }
  const long double offset_ns = static_cast<long double>(index) *
                                static_cast<long double>(canonical_increment) *
                                1'000'000'000.0L;
  const long double stamp = static_cast<long double>(scan.stamp_ns) +
                            std::round(offset_ns);
  if (!std::isfinite(offset_ns) ||
      stamp < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      stamp > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(stamp);
}

std::optional<std::int64_t> scanEndStamp(const ScanValue& scan) {
  return rayStamp(scan, scan.ranges.size() - 1U);
}

Point2 transformPoint(const Pose2& pose, double x, double y) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * x - s * y, pose.y + s * x + c * y};
}

std::optional<std::int64_t> midpointStamp(const ScanValue& scan) {
  const auto end = scanEndStamp(scan);
  if (!end) {
    return std::nullopt;
  }
  const auto half_sum = checkedAdd(scan.stamp_ns / 2, *end / 2);
  if (!half_sum) {
    return std::nullopt;
  }
  return checkedAdd(*half_sum, (scan.stamp_ns % 2 + *end % 2) / 2);
}

Point2 correctedPoint(const Pose2& midpoint_base, const Pose2& ray_base,
                      const Pose2& base_to_lidar, double angle, double range) {
  const Pose2 midpoint_lidar = midpoint_base * base_to_lidar;
  const Pose2 ray_lidar = ray_base * base_to_lidar;
  const Point2 raw_point{std::cos(angle) * range, std::sin(angle) * range};
  const Point2 world_point = transformPoint(ray_lidar, raw_point.x, raw_point.y);
  return transformPoint(midpoint_lidar.inverse(), world_point.x, world_point.y);
}

std::optional<DeskewedScan> makeOdomResult(
    const ScanValue& scan, const TimedPoseBuffer& odom,
    const StaticLidarMount& mount, double range_cap_m) {
  if (!validScanMetadata(scan) || !validMount(mount)) {
    return std::nullopt;
  }
  const auto midpoint = midpointStamp(scan);
  if (!midpoint) {
    return std::nullopt;
  }
  const auto midpoint_base = odom.interpolate(*midpoint);
  if (!midpoint_base) {
    return std::nullopt;
  }

  DeskewedScan result{scan.id, *midpoint, DeskewMethod::kOdom, {}};
  result.points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!validRange(range, scan, range_cap_m)) {
      continue;
    }
    const auto stamp = rayStamp(scan, index);
    if (!stamp) {
      return std::nullopt;
    }
    const auto ray_base = odom.interpolate(*stamp);
    if (!ray_base) {
      return std::nullopt;
    }
    const double angle = static_cast<double>(scan.angle_min) +
                         static_cast<double>(index) * scan.angle_increment;
    if (!std::isfinite(angle)) {
      return std::nullopt;
    }
    result.points.push_back(correctedPoint(*midpoint_base, *ray_base,
                                           Pose2{mount.x_m, mount.y_m, mount.yaw_rad},
                                           angle, range));
  }
  return result;
}

bool imuCoverage(const std::vector<TimedYawRate>& rates, std::int64_t start,
                 std::int64_t end) {
  if (rates.empty() || rates.front().stamp_ns > start || rates.back().stamp_ns < end) {
    return false;
  }
  for (std::size_t index = 1; index < rates.size(); ++index) {
    if (rates[index].stamp_ns <= rates[index - 1].stamp_ns ||
        !std::isfinite(rates[index - 1].omega_rad_s) ||
        (rates[index].stamp_ns > start && rates[index - 1].stamp_ns < end &&
         rates[index].stamp_ns - rates[index - 1].stamp_ns > kMaximumImuGapNs)) {
      return false;
    }
  }
  return std::isfinite(rates.front().omega_rad_s) &&
         std::isfinite(rates.back().omega_rad_s);
}

double interpolatedRate(const TimedYawRate& earlier, const TimedYawRate& later,
                        std::int64_t stamp_ns) {
  if (stamp_ns == earlier.stamp_ns) {
    return earlier.omega_rad_s;
  }
  if (stamp_ns == later.stamp_ns) {
    return later.omega_rad_s;
  }
  const double alpha = static_cast<double>(stamp_ns - earlier.stamp_ns) /
                       static_cast<double>(later.stamp_ns - earlier.stamp_ns);
  return earlier.omega_rad_s + alpha * (later.omega_rad_s - earlier.omega_rad_s);
}

std::optional<double> integratedYaw(const std::vector<TimedYawRate>& rates,
                                    std::int64_t start, std::int64_t target) {
  if (target < start || !imuCoverage(rates, start, target)) {
    return std::nullopt;
  }
  double yaw = 0.0;
  std::int64_t previous = start;
  double previous_rate = 0.0;
  std::size_t segment = 1;
  while (segment < rates.size() && rates[segment].stamp_ns <= start) {
    ++segment;
  }
  if (segment == rates.size()) {
    previous_rate = rates.back().omega_rad_s;
  } else {
    previous_rate = interpolatedRate(rates[segment - 1], rates[segment], start);
  }
  while (previous < target) {
    const std::int64_t next = segment < rates.size()
      ? std::min(target, rates[segment].stamp_ns) : target;
    const double next_rate = segment < rates.size()
      ? interpolatedRate(rates[segment - 1], rates[segment], next) : previous_rate;
    yaw += 0.5 * (previous_rate + next_rate) *
           static_cast<double>(next - previous) / 1'000'000'000.0;
    previous = next;
    previous_rate = next_rate;
    while (segment < rates.size() && rates[segment].stamp_ns <= previous) {
      ++segment;
    }
  }
  return yaw;
}

std::optional<DeskewedScan> makeImuResult(
    const ScanValue& scan, const std::vector<TimedYawRate>& rates,
    double candidate_offset_m, double fixed_yaw_rad, double range_cap_m) {
  if (!validScanMetadata(scan) || !std::isfinite(candidate_offset_m) ||
      !std::isfinite(fixed_yaw_rad) ||
      std::abs(Pose2::normalizeAngle(fixed_yaw_rad - kPi)) > kMountTolerance) {
    return std::nullopt;
  }
  const auto end = scanEndStamp(scan);
  const auto midpoint = midpointStamp(scan);
  if (!end || !midpoint || !imuCoverage(rates, scan.stamp_ns, *end)) {
    return std::nullopt;
  }
  const auto midpoint_yaw = integratedYaw(rates, scan.stamp_ns, *midpoint);
  if (!midpoint_yaw) {
    return std::nullopt;
  }
  const Pose2 base_to_lidar{candidate_offset_m, 0.0, fixed_yaw_rad};
  const Pose2 midpoint_base{0.0, 0.0, *midpoint_yaw};
  DeskewedScan result{scan.id, *midpoint, DeskewMethod::kImu, {}};
  result.points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!validRange(range, scan, range_cap_m)) {
      continue;
    }
    const auto stamp = rayStamp(scan, index);
    if (!stamp) {
      return std::nullopt;
    }
    const auto ray_yaw = integratedYaw(rates, scan.stamp_ns, *stamp);
    if (!ray_yaw) {
      return std::nullopt;
    }
    result.points.push_back(correctedPoint(midpoint_base, Pose2{0.0, 0.0, *ray_yaw},
                                           base_to_lidar,
                                           static_cast<double>(scan.angle_min) +
                                             static_cast<double>(index) * scan.angle_increment,
                                           range));
  }
  return result;
}

std::int64_t median(std::vector<std::int64_t> values) {
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

}  // namespace

std::optional<DeskewedScan> deskewWithOdom(
    const ScanValue& scan, const TimedPoseBuffer& odom,
    const StaticLidarMount& mount, double range_cap_m) {
  return makeOdomResult(scan, odom, mount, range_cap_m);
}

std::optional<DeskewedScan> deskewWithImu(
    const ScanValue& scan, const std::vector<TimedYawRate>& rates,
    double candidate_offset_m, double fixed_yaw_rad, double range_cap_m) {
  return makeImuResult(scan, rates, candidate_offset_m, fixed_yaw_rad, range_cap_m);
}

std::vector<ScanAssociation> associateUndistortedScans(
    const std::vector<ScanValue>& raw, const std::vector<ScanValue>& undistorted,
    std::int64_t maximum_delay_deviation_ns) {
  if (raw.empty() || raw.size() != undistorted.size() ||
      maximum_delay_deviation_ns < 0) {
    return {};
  }
  std::vector<std::int64_t> delays;
  delays.reserve(raw.size());
  for (std::size_t index = 0; index < raw.size(); ++index) {
    const auto delay = checkedSubtract(undistorted[index].stamp_ns,
                                       raw[index].stamp_ns);
    if (!delay) {
      return {};
    }
    delays.push_back(*delay);
  }
  const std::int64_t robust_delay = median(delays);
  std::vector<ScanAssociation> result;
  result.reserve(raw.size());
  for (std::size_t index = 0; index < raw.size(); ++index) {
    const auto deviation = checkedSubtract(delays[index], robust_delay);
    if (!deviation || *deviation < -maximum_delay_deviation_ns ||
        *deviation > maximum_delay_deviation_ns) {
      return {};
    }
    result.push_back({raw[index].id, undistorted[index].id, robust_delay});
  }
  return result;
}

DeskewedScan adaptUndistortedScan(
    const ScanValue& scan, const ScanAssociation& association, double range_cap_m) {
  DeskewedScan result{0, 0, DeskewMethod::kExistingScan, {}};
  if (!validScanMetadata(scan) || scan.id != association.undistorted_scan_id ||
      !std::isfinite(range_cap_m) || range_cap_m < 0.0) {
    return result;
  }
  const auto raw_reference = checkedSubtract(scan.stamp_ns,
                                             association.timestamp_delay_ns);
  if (!raw_reference) {
    return result;
  }
  result.scan_id = association.raw_scan_id;
  result.reference_stamp_ns = *raw_reference;
  result.points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!validRange(range, scan, range_cap_m)) {
      continue;
    }
    const double angle = static_cast<double>(scan.angle_min) +
                         static_cast<double>(index) * scan.angle_increment;
    if (!std::isfinite(angle)) {
      continue;
    }
    result.points.push_back({std::cos(angle) * range, std::sin(angle) * range});
  }
  return result;
}

}  // namespace orb_lidar_mapper
