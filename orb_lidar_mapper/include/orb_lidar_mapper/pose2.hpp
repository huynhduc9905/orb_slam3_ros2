#pragma once

namespace orb_lidar_mapper {

struct Twist2 {
  double vx{};
  double vy{};
  double omega{};
};

struct Pose2 {
  double x{};
  double y{};
  double yaw{};

  static double normalizeAngle(double angle);
  static Pose2 exp(const Twist2& twist);
  Twist2 log() const;
  Pose2 inverse() const;
  Pose2 operator*(const Pose2& rhs) const;
  Pose2 pow(double alpha) const;
  static Pose2 interpolate(const Pose2& a, const Pose2& b, double alpha);
  bool isApprox(const Pose2& rhs, double epsilon) const;
};

}  // namespace orb_lidar_mapper
