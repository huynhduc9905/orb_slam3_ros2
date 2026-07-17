#include "orb_slam3_wrapper/stereo_rotation_center.hpp"

#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>

namespace orb_slam3_wrapper {

double Pose2::normalizeAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
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

Pose2 Pose2::compose(const Pose2& b) const {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
      x + cosine * b.x - sine * b.y,
      y + sine * b.x + cosine * b.y,
      normalizeAngle(yaw + b.yaw),
  };
}

std::optional<Point2> centerFromTransform(const Pose2& source_to_target) {
  const double c = std::cos(source_to_target.yaw);
  const double s = std::sin(source_to_target.yaw);
  Eigen::Matrix2d matrix;
  // (I - R) for R = [[c, -s], [s, c]]: [[1-c, s], [-s, 1-c]]
  matrix << 1.0 - c, s, -s, 1.0 - c;
  if (!std::isfinite(matrix.determinant()) ||
      std::abs(matrix.determinant()) < 1e-4) {
    return std::nullopt;
  }
  const Eigen::Vector2d translation(source_to_target.x, source_to_target.y);
  const Eigen::Vector2d center = matrix.fullPivLu().solve(translation);
  if (!center.allFinite()) {
    return std::nullopt;
  }
  return Point2{center.x(), center.y()};
}

}  // namespace orb_slam3_wrapper
