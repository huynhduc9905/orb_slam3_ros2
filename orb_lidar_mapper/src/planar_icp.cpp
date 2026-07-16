#include "orb_lidar_mapper/planar_icp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>

namespace orb_lidar_mapper {
namespace {

using PclPoint = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PclPoint>;

struct Match {
  double squared_distance{};
  std::size_t source{};
  std::size_t target{};
};

PclCloud::Ptr toPcl(const std::vector<Point2>& points) {
  auto cloud = std::make_shared<PclCloud>();
  cloud->reserve(points.size());
  for (const auto& point : points) cloud->push_back({static_cast<float>(point.x),
                                                      static_cast<float>(point.y), 0.0F});
  return cloud;
}

Point2 transformPoint(const Eigen::Matrix4f& matrix, Point2 point) {
  return {matrix(0, 0) * point.x + matrix(0, 1) * point.y + matrix(0, 3),
          matrix(1, 0) * point.x + matrix(1, 1) * point.y + matrix(1, 3)};
}

std::vector<Match> recomputeMatches(const std::vector<Point2>& source,
                                    const std::vector<Point2>& target,
                                    const Eigen::Matrix4f& transform,
                                    double max_distance) {
  std::vector<Match> matches;
  const double limit = max_distance * max_distance;
  for (std::size_t i = 0; i < source.size(); ++i) {
    const auto transformed = transformPoint(transform, source[i]);
    Match best{std::numeric_limits<double>::infinity(), i, 0};
    for (std::size_t j = 0; j < target.size(); ++j) {
      const double dx = transformed.x - target[j].x;
      const double dy = transformed.y - target[j].y;
      const double distance = dx * dx + dy * dy;
      if (distance < best.squared_distance ||
          (distance == best.squared_distance && j < best.target)) {
        best = {distance, i, j};
      }
    }
    if (best.squared_distance <= limit) matches.push_back(best);
  }
  std::sort(matches.begin(), matches.end(), [](const Match& a, const Match& b) {
    if (a.squared_distance != b.squared_distance)
      return a.squared_distance < b.squared_distance;
    if (a.source != b.source) return a.source < b.source;
    return a.target < b.target;
  });
  return matches;
}

double yawFrom(const Eigen::Matrix4f& matrix) {
  return std::atan2(static_cast<double>(matrix(1, 0)),
                    static_cast<double>(matrix(0, 0)));
}

Eigen::Matrix4f initialGuess(const std::vector<Point2>& source,
                             const std::vector<Point2>& target,
                             double yaw) {
  const auto centroid = [](const std::vector<Point2>& points) {
    Point2 result{};
    for (const auto point : points) {
      result.x += point.x;
      result.y += point.y;
    }
    result.x /= static_cast<double>(points.size());
    result.y /= static_cast<double>(points.size());
    return result;
  };
  const Point2 source_centroid = centroid(source);
  const Point2 target_centroid = centroid(target);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess(0, 0) = static_cast<float>(c);
  guess(0, 1) = static_cast<float>(-s);
  guess(1, 0) = static_cast<float>(s);
  guess(1, 1) = static_cast<float>(c);
  guess(0, 3) = static_cast<float>(target_centroid.x - c * source_centroid.x +
                                   s * source_centroid.y);
  guess(1, 3) = static_cast<float>(target_centroid.y - s * source_centroid.x -
                                   c * source_centroid.y);
  return guess;
}

bool geometryIsDegenerate(const std::vector<Point2>& points) {
  if (points.size() < 3U) return true;
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  Point2 mean{};
  for (const auto point : points) {
    mean.x += point.x;
    mean.y += point.y;
  }
  mean.x /= points.size();
  mean.y /= points.size();
  for (const auto point : points) {
    const Eigen::Vector2d delta(point.x - mean.x, point.y - mean.y);
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) return true;
  const auto values = solver.eigenvalues();
  if (values(0) <= std::numeric_limits<double>::epsilon()) return true;
  const double condition = values(1) / values(0);
  if (!std::isfinite(condition) || condition > 1e4) return true;
  const double isotropy = std::abs(values(1) - values(0)) /
                          std::max(values(1), std::numeric_limits<double>::min());
  return isotropy < 1e-3;
}

}  // namespace

IcpResult PlanarIcp::align(const std::vector<Point2>& source,
                           const std::vector<Point2>& target,
                           double expected_yaw_rad) const {
  IcpResult result;
  if (source.size() < 4U || target.size() < 4U) {
    result.rejection_reason = "insufficient_points";
    return result;
  }
  if (geometryIsDegenerate(source) || geometryIsDegenerate(target)) {
    result.rejection_reason = "degenerate_geometry";
    return result;
  }

  const auto source_cloud = toPcl(source);
  const auto target_cloud = toPcl(target);
  pcl::IterativeClosestPoint<PclPoint, PclPoint> icp;
  using Estimator = pcl::registration::TransformationEstimation2D<
      PclPoint, PclPoint, float>;
  icp.setTransformationEstimation(std::make_shared<Estimator>());
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  icp.setMaximumIterations(config_.max_iterations);
  icp.setMaxCorrespondenceDistance(config_.max_correspondence_m);
  icp.setTransformationEpsilon(1e-10);
  PclCloud output;
  const auto guess = initialGuess(source, target, expected_yaw_rad);
  icp.align(output, guess);
  if (!icp.hasConverged()) {
    // A broad diagnostic pass is used only to distinguish a valid pair with
    // the wrong odometry yaw from an otherwise unalignable pair. All accepted
    // correspondences and quality metrics below still use the configured cap.
    icp.setMaxCorrespondenceDistance(std::max(config_.max_correspondence_m, 2.0));
    icp.align(output, guess);
    if (!icp.hasConverged()) {
      result.rejection_reason = "icp_not_converged";
      return result;
    }
  }

  const Eigen::Matrix4f transform = icp.getFinalTransformation();
  result.source_to_target = {transform(0, 3), transform(1, 3), yawFrom(transform)};
  const double yaw_error = std::abs(Pose2::normalizeAngle(
      result.source_to_target.yaw - expected_yaw_rad));
  if (!std::isfinite(yaw_error) || yaw_error > config_.maximum_yaw_error_rad) {
    result.rejection_reason = "yaw_disagreement";
    return result;
  }

  auto matches = recomputeMatches(source, target, transform,
                                  config_.max_correspondence_m);
  const std::size_t trim = matches.size() / 5U;
  if (matches.size() <= trim) {
    result.rejection_reason = "insufficient_correspondences";
    return result;
  }
  matches.resize(matches.size() - trim);
  result.correspondence_count = matches.size();
  result.overlap_ratio = static_cast<double>(matches.size()) /
                         static_cast<double>(std::min(source.size(), target.size()));
  double squared_sum = 0.0;
  for (const auto& match : matches) squared_sum += match.squared_distance;
  result.trimmed_rmse_m = std::sqrt(squared_sum / matches.size());
  if (result.overlap_ratio < config_.minimum_overlap) {
    result.rejection_reason = "insufficient_overlap";
    return result;
  }
  if (!std::isfinite(result.trimmed_rmse_m) ||
      result.trimmed_rmse_m > config_.maximum_trimmed_rmse_m) {
    result.rejection_reason = "trimmed_rmse";
    return result;
  }
  result.converged = true;
  return result;
}

}  // namespace orb_lidar_mapper
