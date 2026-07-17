#pragma once

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

}  // namespace orb_slam3_wrapper
