#include "orb_lidar_mapper/scan_deskewer.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace orb_lidar_mapper {
namespace {

constexpr double kMaxUsableRangeM = 20.0;

std::uint64_t orderedDuration(std::int64_t earlier_ns, std::int64_t later_ns) {
  return static_cast<std::uint64_t>(later_ns) - static_cast<std::uint64_t>(earlier_ns);
}

bool validScan(const ScanValue& scan) {
  return std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
    scan.range_min >= 0.0F && scan.range_min <= scan.range_max &&
    std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment) &&
    std::isfinite(scan.time_increment) && scan.time_increment >= 0.0F;
}

std::optional<std::pair<Pose2, double>> bracketedBasePose(
  const ScanMotionBracket& bracket, const Pose2& wheel_pose, std::int64_t stamp_ns) {
  if (stamp_ns < bracket.start_stamp_ns || stamp_ns > bracket.end_stamp_ns) {
    return std::nullopt;
  }
  const auto duration = orderedDuration(bracket.start_stamp_ns, bracket.end_stamp_ns);
  const double alpha = duration == 0 ? 0.0 :
    static_cast<double>(orderedDuration(bracket.start_stamp_ns, stamp_ns)) /
    static_cast<double>(duration);
  const Pose2 predicted = bracket.start_map_pose *
    bracket.start_wheel_pose.inverse() * wheel_pose;
  const Pose2 predicted_end = bracket.start_map_pose *
    bracket.start_wheel_pose.inverse() * bracket.end_wheel_pose;
  const Pose2 residual = predicted_end.inverse() * bracket.end_map_pose;
  return std::make_pair(predicted * residual.pow(alpha), alpha);
}

std::optional<std::int64_t> rayStampWithIncrement(
  const ScanValue& scan, std::size_t index, long double time_increment) {
  const long double offset_ns = static_cast<long double>(index) *
                                time_increment * 1'000'000'000.0L;
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

std::optional<std::int64_t> rayStamp(const ScanValue& scan, std::size_t index) {
  return rayStampWithIncrement(scan, index, scan.time_increment);
}

std::optional<long double> bracketedTimeIncrement(const ScanValue& scan) {
  // Use the shortest decimal that round-trips to the float. This removes the
  // binary tail from nominal cadences such as 0.05F without imposing a fixed
  // time grid on sub-microsecond or non-integral-microsecond scans.
  std::array<char, 32> text{};
  const auto formatted = std::to_chars(
    text.data(), text.data() + text.size(), scan.time_increment);
  if (formatted.ec != std::errc{}) {
    return std::nullopt;
  }
  double time_increment{};
  const auto parsed = std::from_chars(text.data(), formatted.ptr, time_increment);
  if (parsed.ec != std::errc{} || parsed.ptr != formatted.ptr) {
    return std::nullopt;
  }
  const std::size_t final_index = scan.ranges.empty() ? 0U : scan.ranges.size() - 1U;
  const auto raw_end = rayStamp(scan, final_index);
  const auto canonical_end = rayStampWithIncrement(scan, final_index, time_increment);
  if (!raw_end || !canonical_end) {
    return std::nullopt;
  }
  const auto tail = *raw_end <= *canonical_end
    ? orderedDuration(*raw_end, *canonical_end)
    : orderedDuration(*canonical_end, *raw_end);
  return tail <= 1U ? static_cast<long double>(time_increment) :
    static_cast<long double>(scan.time_increment);
}

Point2 transformPoint(const Pose2& pose, double x, double y) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {pose.x + cosine * x - sine * y, pose.y + sine * x + cosine * y};
}

// Decide once per scan whether IMU covers the full sweep [scan.stamp, last ray].
// Partial coverage → pure wheel for the whole scan (never mix mid-scan).
bool imuCoversFullSweep(const ImuYawBuffer* imu, const ScanValue& scan,
                        long double time_increment) {
  if (imu == nullptr || scan.ranges.empty()) {
    return false;
  }
  const std::size_t last_index = scan.ranges.size() - 1U;
  const auto last_stamp = rayStampWithIncrement(scan, last_index, time_increment);
  if (!last_stamp) {
    return false;
  }
  return imu->covers(scan.stamp_ns, *last_stamp);
}

// Wheel xy from odometry + IMU integrated yaw over [scan.stamp, t].
// Returns nullopt if wheels or IMU integration unavailable for this ray.
std::optional<Pose2> fuseWheelXyImuYaw(const TimedPoseBuffer& wheels,
                                       const ImuYawBuffer& imu,
                                       std::int64_t scan_stamp_ns,
                                       std::int64_t ray_stamp_ns) {
  const auto wheel_0 = wheels.interpolate(scan_stamp_ns);
  const auto wheel_t = wheels.interpolate(ray_stamp_ns);
  if (!wheel_0 || !wheel_t) {
    return std::nullopt;
  }
  const auto dyaw = imu.integratedYaw(scan_stamp_ns, ray_stamp_ns);
  if (!dyaw) {
    return std::nullopt;
  }
  const Pose2 rel = wheel_0->inverse() * *wheel_t;
  return *wheel_0 * Pose2{rel.x, rel.y, *dyaw};
}

}  // namespace

std::optional<std::vector<Ray2>> ScanDeskewer::deskew(
  const ScanValue& scan, const Pose2& committed_scan_base_pose, const Pose2& base_to_lidar,
  const TimedPoseBuffer& wheels, const ImuYawBuffer* imu) {
  if (!validScan(scan)) {
    return std::nullopt;
  }

  // Beams that are NaN, +/-inf (no-return / physically blocked angles),
  // below range_min, above range_max, or beyond the cap are ignored entirely:

  const bool use_imu =
    imuCoversFullSweep(imu, scan, static_cast<long double>(scan.time_increment));

  std::vector<Ray2> rays;
  rays.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const float range = scan.ranges[index];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max ||
        static_cast<double>(range) > kMaxUsableRangeM) {
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

    std::optional<Pose2> relative_motion;
    if (use_imu) {
      // Fused relative: Pose2{rel.x, rel.y, dyaw} = inv(wheel_0)*fused.
      const auto fused =
        fuseWheelXyImuYaw(wheels, *imu, scan.stamp_ns, *stamp_ns);
      if (!fused) {
        continue;
      }
      const auto wheel_0 = wheels.interpolate(scan.stamp_ns);
      if (!wheel_0) {
        continue;
      }
      relative_motion = wheel_0->inverse() * *fused;
    } else {
      relative_motion = wheels.relative(scan.stamp_ns, *stamp_ns);
      if (!relative_motion) {
        // A single ray whose timestamp cannot be bracketed by wheel odometry
        // within the configured gap (e.g. the sweep tail outruns the wheel
        // buffer during live replay) must not discard the whole scan. Skip only
        // the un-interpolatable ray, exactly as NaN/out-of-range beams are
        // skipped above; the remaining well-covered rays still build the map.
        continue;
      }
    }

    const Pose2 lidar_pose = committed_scan_base_pose * *relative_motion * base_to_lidar;
    rays.push_back({{lidar_pose.x, lidar_pose.y},
                    transformPoint(lidar_pose, std::cos(angle) * range,
                                   std::sin(angle) * range),
                    true});
  }
  return rays;
}

std::optional<BracketedDeskewResult> ScanDeskewer::deskewBracketed(
  const ScanValue& scan, const Pose2& base_to_lidar,
  const TimedPoseBuffer& wheels, const ScanMotionBracket& bracket,
  const ImuYawBuffer* imu) {
  if (!validScan(scan) || bracket.end_stamp_ns < bracket.start_stamp_ns ||
      scan.stamp_ns < bracket.start_stamp_ns || scan.stamp_ns > bracket.end_stamp_ns) {
    return std::nullopt;
  }

  const auto time_increment = bracketedTimeIncrement(scan);
  if (!time_increment) {
    return std::nullopt;
  }

  // residual^α is predicted_end^{-1} * end_map. When fusing IMU yaw into ray
  // motion, predicted_end must use the same fused end motion so α=1 lands on
  // ORB end_map even if IMU yaw ≠ wheel yaw. Require IMU coverage through the
  // ORB end stamp (may be after last ray); if fuse to that stamp fails, fall
  // back to wheel-only for the whole scan — never hard-reject for IMU alone.
  bool use_imu = imuCoversFullSweep(imu, scan, *time_increment) && imu != nullptr &&
    imu->covers(scan.stamp_ns, bracket.end_stamp_ns);
  ScanMotionBracket effective = bracket;
  if (use_imu) {
    const auto fused_end =
      fuseWheelXyImuYaw(wheels, *imu, scan.stamp_ns, bracket.end_stamp_ns);
    if (!fused_end) {
      use_imu = false;
    } else {
      effective.end_wheel_pose = *fused_end;
    }
  }

  const auto reference_wheel = wheels.interpolate(scan.stamp_ns);
  if (!reference_wheel) {
    return std::nullopt;
  }
  // Reference uses true wheel at scan start (dyaw=0 when fusing).
  const auto reference = bracketedBasePose(effective, *reference_wheel, scan.stamp_ns);
  if (!reference) {
    return std::nullopt;
  }

  BracketedDeskewResult result;
  result.rays.reserve(scan.ranges.size());
  result.ray_motions.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const float range = scan.ranges[index];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max ||
        static_cast<double>(range) > kMaxUsableRangeM) {
      continue;
    }
    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(index) * static_cast<double>(scan.angle_increment);
    if (!std::isfinite(angle)) {
      continue;
    }
    const auto stamp_ns = rayStampWithIncrement(scan, index, *time_increment);
    if (!stamp_ns) {
      return std::nullopt;
    }
    const auto wheel_pose = wheels.interpolate(*stamp_ns);
    if (!wheel_pose) {
      // Deferred committed scans require complete motion coverage. Never
      // archive a directional prefix when one valid ray cannot be bracketed.
      return std::nullopt;
    }

    // Motion for bracketedBasePose: fused when IMU covers full sweep.
    Pose2 motion_pose = *wheel_pose;
    if (use_imu) {
      const auto fused =
        fuseWheelXyImuYaw(wheels, *imu, scan.stamp_ns, *stamp_ns);
      if (!fused) {
        // Coverage claimed full sweep but this ray failed — fall back is not
        // mid-scan mixable; abort IMU path is already gated. Treat as wheel
        // hard-fail only when wheels also cannot supply the ray (checked above).
        return std::nullopt;
      }
      motion_pose = *fused;
    }

    const auto ray_base = bracketedBasePose(effective, motion_pose, *stamp_ns);
    if (!ray_base) {
      return std::nullopt;
    }

    const Pose2 reference_to_ray = reference->first.inverse() * ray_base->first;
    const Pose2 lidar_pose = reference_to_ray * base_to_lidar;
    const Point2 lidar_end{std::cos(angle) * range, std::sin(angle) * range};
    result.rays.push_back({
      {lidar_pose.x, lidar_pose.y},
      transformPoint(lidar_pose, lidar_end.x, lidar_end.y),
      true});
    // Archive stores the true wheel interpolate for rebuild fidelity.
    result.ray_motions.push_back({*wheel_pose, ray_base->second, lidar_end, true});
  }
  return result;
}

}  // namespace orb_lidar_mapper
