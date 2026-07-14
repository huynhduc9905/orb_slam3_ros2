#include "orb_lidar_mapper/pose2.hpp"

#include <algorithm>
#include <cmath>

namespace orb_lidar_mapper {
namespace {

constexpr double kSmallAngle = 1e-8;

double sinc(double angle) {
  if (std::abs(angle) < kSmallAngle) {
    const double angle_squared = angle * angle;
    return 1.0 - angle_squared / 6.0;
  }
  return std::sin(angle) / angle;
}

double cosc(double angle) {
  if (std::abs(angle) < kSmallAngle) {
    return angle / 2.0;
  }
  return (1.0 - std::cos(angle)) / angle;
}

}  // namespace

double Pose2::normalizeAngle(double angle) {
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

Pose2 Pose2::exp(const Twist2& twist) {
  const double a = sinc(twist.omega);
  const double b = cosc(twist.omega);
  return {
    a * twist.vx - b * twist.vy,
    b * twist.vx + a * twist.vy,
    normalizeAngle(twist.omega),
  };
}

Twist2 Pose2::log() const {
  const double omega = normalizeAngle(yaw);
  const double a = sinc(omega);
  const double b = cosc(omega);
  const double determinant = a * a + b * b;
  return {
    (a * x + b * y) / determinant,
    (-b * x + a * y) / determinant,
    omega,
  };
}

Pose2 Pose2::inverse() const {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
    -cosine * x - sine * y,
    sine * x - cosine * y,
    normalizeAngle(-yaw),
  };
}

Pose2 Pose2::operator*(const Pose2& rhs) const {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
    x + cosine * rhs.x - sine * rhs.y,
    y + sine * rhs.x + cosine * rhs.y,
    normalizeAngle(yaw + rhs.yaw),
  };
}

Pose2 Pose2::pow(double alpha) const {
  alpha = std::clamp(alpha, 0.0, 1.0);
  Twist2 twist = log();
  twist.vx *= alpha;
  twist.vy *= alpha;
  twist.omega *= alpha;
  return exp(twist);
}

Pose2 Pose2::interpolate(const Pose2& a, const Pose2& b, double alpha) {
  return a * (a.inverse() * b).pow(alpha);
}

bool Pose2::isApprox(const Pose2& rhs, double epsilon) const {
  return std::abs(x - rhs.x) <= epsilon &&
         std::abs(y - rhs.y) <= epsilon &&
         std::abs(normalizeAngle(yaw - rhs.yaw)) <= epsilon;
}

}  // namespace orb_lidar_mapper
