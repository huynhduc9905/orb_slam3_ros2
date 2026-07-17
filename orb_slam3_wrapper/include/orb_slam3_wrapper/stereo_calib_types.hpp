#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace orb_slam3_wrapper {

constexpr double kPi = 3.14159265358979323846;

struct Point2 {
  double x{};
  double y{};
};

struct Pose2 {
  double x{};
  double y{};
  double yaw{};

  static double normalizeAngle(double a);
  Pose2 inverse() const;
  Pose2 compose(const Pose2& b) const;  // this * b
};

// Implied base_link → camera_link translation xy (z kept from recorded TF).
struct MountXy {
  double x_m{};
  double y_m{};
};

enum class ResultClass { kConsistent, kLikelyOffsetError, kInconclusive };

struct MotionInterval {
  std::int64_t start_ns{};
  std::int64_t end_ns{};
};

struct StereoCenterSample {
  std::size_t source_index{};
  std::size_t target_index{};
  std::size_t yaw_sector{};
  bool accepted{};
  Point2 center;    // horizontal-left lever arm sample
  MountXy mount_xy;  // mapped sample
  std::string rejection_reason;
};

struct StereoEstimate {
  bool reliable{};
  MountXy median_xy;
  MountXy ci_half_width;  // bootstrap 95% half-width on x and y
  std::size_t accepted_pairs{};
  std::size_t sectors_used{};
  std::vector<std::string> unreliable_reasons;
};

struct StereoAggregate {
  ResultClass result_class{ResultClass::kInconclusive};
  StereoEstimate estimate;
  MountXy recorded_xy;
  MountXy delta_xy;
  std::string summary;
};

// Defaults (provisional from spec)
struct StereoThresholds {
  std::size_t min_accepted_pairs{40};
  std::size_t min_sectors{6};
  double max_ci_half_width_m{0.015};
  double agreement_floor_m{0.010};
  double max_abs_center_m{1.0};
  double min_pair_yaw_rad{10.0 * kPi / 180.0};
  double max_pair_yaw_rad{30.0 * kPi / 180.0};
};

}  // namespace orb_slam3_wrapper
