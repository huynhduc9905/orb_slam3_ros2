#include "orb_lidar_mapper/calibration_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <utility>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace orb_lidar_mapper {
namespace {

constexpr std::size_t kBootstrapSamples = 2000;
constexpr std::size_t kMinimumAccepted = 40;
constexpr std::size_t kMinimumSectors = 6;
constexpr double kMaximumCiHalfWidth = 0.015;
constexpr double kMaximumMedianY = 0.020;
constexpr double kMethodSpread = 0.015;
constexpr double kTrimFraction = 0.20;

double median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if (values.size() % 2 != 0) return upper;
  const auto lower = std::max_element(values.begin(), middle);
  return (*lower + upper) / 2.0;
}

double weightedMedian(const std::vector<double>& values,
                      const std::vector<double>& weights) {
  std::vector<std::pair<double, double>> ordered;
  ordered.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (std::isfinite(values[i]) && std::isfinite(weights[i]) && weights[i] > 0.0) {
      ordered.emplace_back(values[i], weights[i]);
    }
  }
  if (ordered.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(ordered.begin(), ordered.end());
  double total = 0.0;
  for (const auto& item : ordered) total += item.second;
  double accumulated = 0.0;
  for (const auto& item : ordered) {
    accumulated += item.second;
    if (accumulated >= total / 2.0) return item.first;
  }
  return ordered.back().first;
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - static_cast<double>(lower);
  return values[lower] * (1.0 - alpha) + values[upper] * alpha;
}

bool finiteSample(const CenterSample& sample) {
  return std::isfinite(sample.center.x) && std::isfinite(sample.center.y) &&
    std::isfinite(sample.icp.trimmed_rmse_m) && std::isfinite(sample.icp.overlap_ratio);
}

MethodEstimate robustMethodEstimateImpl(
  DeskewMethod method, const std::vector<CenterSample>& samples, std::uint64_t seed) {
  MethodEstimate result;
  result.method = method;
  result.attempted_pairs = samples.size();
  std::vector<const CenterSample*> accepted;
  for (const auto& sample : samples) {
    if (sample.method != method) {
      ++result.rejection_counts["method_mismatch"];
    } else if (!sample.accepted) {
      ++result.rejection_counts[sample.rejection_reason.empty() ? "rejected" : sample.rejection_reason];
    } else if (!finiteSample(sample)) {
      ++result.rejection_counts["nonfinite_sample"];
    } else {
      accepted.push_back(&sample);
    }
  }
  result.accepted_pairs = accepted.size();
  if (accepted.empty()) {
    result.rejection_counts["no_accepted_pairs"]++;
    return result;
  }

  std::vector<double> xs, ys, rmses, overlaps;
  std::vector<std::size_t> sectors;
  xs.reserve(accepted.size()); ys.reserve(accepted.size());
  rmses.reserve(accepted.size()); overlaps.reserve(accepted.size());
  for (const auto* sample : accepted) {
    xs.push_back(sample->center.x); ys.push_back(sample->center.y);
    rmses.push_back(sample->icp.trimmed_rmse_m); overlaps.push_back(sample->icp.overlap_ratio);
    sectors.push_back(sample->yaw_sector);
  }
  const double raw_median_x = median(xs);
  const double raw_median_y = median(ys);
  std::vector<double> deviations;
  deviations.reserve(xs.size());
  for (const double value : xs) deviations.push_back(std::abs(value - raw_median_x));
  const double mad = median(deviations);
  const double scale = std::max(1.4826 * mad, 1e-9);
  std::vector<double> weights;
  weights.reserve(xs.size());
  for (const double value : xs) {
    const double standardized = std::abs(value - raw_median_x) / scale;
    weights.push_back(1.0 / (1.0 + standardized * standardized));
  }
  result.forward_offset_m = weightedMedian(xs, weights);
  result.center_x_m = result.forward_offset_m;
  result.center_y_m = weightedMedian(ys, weights);
  result.median_rmse_m = median(rmses);
  result.median_overlap = median(overlaps);
  for (const auto sector : sectors) if (sector < 8) {
    result.covered_yaw_sectors = std::max(result.covered_yaw_sectors, sector + 1);
  }
  std::vector<bool> present(8, false);
  for (const auto sector : sectors) if (sector < present.size()) present[sector] = true;
  result.covered_yaw_sectors = std::count(present.begin(), present.end(), true);

  std::mt19937_64 generator(seed);
  std::uniform_int_distribution<std::size_t> pick(0, xs.size() - 1);
  std::vector<double> bootstrap;
  bootstrap.reserve(kBootstrapSamples);
  std::vector<double> resample;
  std::vector<double> resample_weights(xs.size(), 1.0);
  for (std::size_t iteration = 0; iteration < kBootstrapSamples; ++iteration) {
    resample.clear();
    for (std::size_t i = 0; i < xs.size(); ++i) resample.push_back(xs[pick(generator)]);
    bootstrap.push_back(median(std::move(resample)));
  }
  result.confidence_95_m = {percentile(bootstrap, 0.025), percentile(bootstrap, 0.975)};
  const double half_width = (result.confidence_95_m.high_m - result.confidence_95_m.low_m) / 2.0;
  result.reliable = accepted.size() >= kMinimumAccepted &&
    result.covered_yaw_sectors >= kMinimumSectors && std::isfinite(half_width) &&
    half_width <= kMaximumCiHalfWidth && std::abs(result.center_y_m) <= kMaximumMedianY;
  if (accepted.size() < kMinimumAccepted) result.rejection_counts["insufficient_accepted_pairs"]++;
  if (result.covered_yaw_sectors < kMinimumSectors) result.rejection_counts["insufficient_yaw_sectors"]++;
  if (!std::isfinite(half_width) || half_width > kMaximumCiHalfWidth) {
    result.rejection_counts["confidence_interval_too_wide"]++;
  }
  if (!std::isfinite(result.center_y_m) || std::abs(result.center_y_m) > kMaximumMedianY) {
    result.rejection_counts["median_center_y_out_of_bounds"]++;
  }
  return result;
}

struct WorldScan {
  std::size_t sector{};
  std::vector<pcl::PointXYZ> points;
};

std::vector<pcl::PointXYZ> transformPoints(const DeskewedScan& scan,
                                            const Pose2& odom_pose,
                                            double offset) {
  const Pose2 mount{offset, 0.0, kPi};
  const Pose2 world = odom_pose * mount;
  std::vector<pcl::PointXYZ> points;
  points.reserve(scan.points.size());
  for (const auto& point : scan.points) {
    points.emplace_back(
      static_cast<float>(world.x + std::cos(world.yaw) * point.x - std::sin(world.yaw) * point.y),
      static_cast<float>(world.y + std::sin(world.yaw) * point.x + std::cos(world.yaw) * point.y),
      0.0f);
  }
  return points;
}

double sharpnessScore(const std::vector<DeskewedScan>& scans,
                      const TimedPoseBuffer& odom, double offset) {
  std::vector<WorldScan> world_scans;
  std::vector<double> yaws;
  for (const auto& scan : scans) {
    const auto pose = odom.interpolate(scan.reference_stamp_ns);
    if (!pose || scan.points.empty()) continue;
    const double yaw = Pose2::normalizeAngle(pose->yaw - (yaws.empty() ? pose->yaw : yaws.front()));
    double normalized = yaw;
    if (normalized < 0.0) normalized += 2.0 * kPi;
    const std::size_t sector = std::min<std::size_t>(7, static_cast<std::size_t>(normalized / (2.0 * kPi / 8.0)));
    world_scans.push_back({sector, transformPoints(scan, *pose, offset)});
    yaws.push_back(pose->yaw);
  }
  if (world_scans.size() < 3) return std::numeric_limits<double>::infinity();

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds(8);
  for (std::size_t sector = 0; sector < clouds.size(); ++sector) {
    clouds[sector] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }
  for (const auto& scan : world_scans) {
    clouds[scan.sector]->points.insert(
      clouds[scan.sector]->points.end(), scan.points.begin(), scan.points.end());
    clouds[scan.sector]->width = static_cast<std::uint32_t>(clouds[scan.sector]->points.size());
    clouds[scan.sector]->height = 1;
  }
  std::vector<double> residuals;
  for (std::size_t source_sector = 0; source_sector < 8; ++source_sector) {
    for (const auto& point : clouds[source_sector]->points) {
      double nearest = std::numeric_limits<double>::infinity();
      for (std::size_t target_sector = 0; target_sector < 8; ++target_sector) {
        const std::size_t separation = (target_sector + 8 - source_sector) % 8;
        if (target_sector == source_sector || separation == 1 || separation == 7 || clouds[target_sector]->empty()) continue;
        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(clouds[target_sector]);
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        if (tree.nearestKSearch(point, 1, indices, distances) > 0) nearest = std::min(nearest, std::sqrt(static_cast<double>(distances[0])));
      }
      if (std::isfinite(nearest)) residuals.push_back(nearest);
    }
  }
  if (residuals.size() < 5) return std::numeric_limits<double>::infinity();
  std::sort(residuals.begin(), residuals.end());
  const auto keep = std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(residuals.size() * (1.0 - kTrimFraction))));
  return std::accumulate(residuals.begin(), residuals.begin() + keep, 0.0) / static_cast<double>(keep);
}

std::vector<SharpnessPoint> makeGrid(const std::vector<DeskewedScan>& scans,
                                     const TimedPoseBuffer& odom,
                                     double start, double end, double step) {
  std::vector<SharpnessPoint> points;
  for (double offset = start; offset <= end + step * 0.1; offset += step) {
    points.push_back({offset, sharpnessScore(scans, odom, offset)});
  }
  return points;
}

const SharpnessPoint* minimumPoint(const std::vector<SharpnessPoint>& points) {
  const SharpnessPoint* best = nullptr;
  for (const auto& point : points) if (std::isfinite(point.score) && (!best || point.score < best->score)) best = &point;
  return best;
}

double gridScoreAt(const std::vector<SharpnessPoint>& points, double offset) {
  const auto it = std::min_element(points.begin(), points.end(), [offset](const auto& a, const auto& b) {
    return std::abs(a.offset_m - offset) < std::abs(b.offset_m - offset);
  });
  return it == points.end() || std::abs(it->offset_m - offset) > 1e-7 ? std::numeric_limits<double>::infinity() : it->score;
}

}  // namespace

MethodEstimate robustMethodEstimate(
  DeskewMethod method, const std::vector<CenterSample>& samples, std::uint64_t seed) {
  return robustMethodEstimateImpl(method, samples, seed);
}

SharpnessResult evaluateMapSharpness(
  const RotationDataset&, const std::vector<DeskewedScan>& scans,
  const TimedPoseBuffer& odom, double consensus_hint) {
  SharpnessResult result;
  result.coarse = makeGrid(scans, odom, 0.180, 0.340, 0.002);
  const auto* coarse_min = minimumPoint(result.coarse);
  if (!coarse_min) {
    result.rejection_reason = "insufficient_sharpness_correspondences";
    return result;
  }
  const double refine_start = std::max(0.180, coarse_min->offset_m - 0.002);
  const double refine_end = std::min(0.340, coarse_min->offset_m + 0.002);
  result.refined = makeGrid(scans, odom, refine_start, refine_end, 0.00025);
  const auto* best = minimumPoint(result.refined);
  if (!best) {
    result.rejection_reason = "insufficient_sharpness_correspondences";
    return result;
  }
  result.best_offset_m = best->offset_m;
  if (best->offset_m <= 0.180 + 1e-9 || best->offset_m >= 0.340 - 1e-9) {
    result.rejection_reason = "sharpness_boundary";
    return result;
  }
  double second_best = std::numeric_limits<double>::infinity();
  for (const auto& point : result.refined) {
    if (&point != best) second_best = std::min(second_best, point.score);
  }
  if (second_best <= best->score * (1.0 + 1e-6)) {
    result.rejection_reason = "sharpness_non_unique";
    return result;
  }
  const double left = gridScoreAt(result.refined, best->offset_m - 0.020);
  const double right = gridScoreAt(result.refined, best->offset_m + 0.020);
  if (!std::isfinite(left) || !std::isfinite(right) ||
      best->score > left * 0.97 || best->score > right * 0.97) {
    result.rejection_reason = "sharpness_not_three_percent_sharper";
    return result;
  }
  if (std::abs(best->offset_m - consensus_hint) > 0.010) {
    result.rejection_reason = "sharpness_consensus_disagreement";
    return result;
  }
  result.reliable = true;
  return result;
}

AggregateResult classifyCalibration(
  const std::vector<MethodEstimate>& estimates, const SharpnessResult& sharpness,
  double recorded_offset_m) {
  AggregateResult result;
  std::vector<const MethodEstimate*> reliable;
  for (const auto& estimate : estimates) if (estimate.reliable) reliable.push_back(&estimate);
  if (reliable.size() < 2) {
    result.reason = "insufficient_reliable_methods";
    return result;
  }
  double minimum = reliable.front()->forward_offset_m;
  double maximum = minimum;
  double weight_sum = 0.0;
  double weighted_sum = 0.0;
  double narrowest = std::numeric_limits<double>::infinity();
  for (const auto* estimate : reliable) {
    minimum = std::min(minimum, estimate->forward_offset_m);
    maximum = std::max(maximum, estimate->forward_offset_m);
    const double half_width = std::max(1e-6, (estimate->confidence_95_m.high_m - estimate->confidence_95_m.low_m) / 2.0);
    const double weight = 1.0 / (half_width * half_width);
    weighted_sum += weight * estimate->forward_offset_m;
    weight_sum += weight;
    narrowest = std::min(narrowest, half_width);
  }
  if (maximum - minimum > kMethodSpread) {
    result.reason = "reliable_method_spread_exceeds_0.015m";
    return result;
  }
  result.consensus_offset_m = weighted_sum / weight_sum;
  const double aggregate_half_width = std::max(narrowest, 1.96 / std::sqrt(weight_sum));
  result.confidence_95_m = {result.consensus_offset_m - aggregate_half_width,
                            result.consensus_offset_m + aggregate_half_width};
  if (!sharpness.reliable) {
    result.reason = sharpness.rejection_reason.empty() ? "sharpness_unreliable" : sharpness.rejection_reason;
    return result;
  }
  if (std::abs(sharpness.best_offset_m - result.consensus_offset_m) > 0.010) {
    result.reason = "sharpness_consensus_disagreement";
    return result;
  }
  const double delta = std::abs(result.consensus_offset_m - recorded_offset_m);
  const double threshold = std::max(0.010, aggregate_half_width);
  result.classification = delta <= threshold ? ResultClass::kConsistent : ResultClass::kLikelyOffsetError;
  result.reason = result.classification == ResultClass::kConsistent ? "consistent" : "likely_offset_error";
  return result;
}

}  // namespace orb_lidar_mapper
