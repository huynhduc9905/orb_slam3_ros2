#include "orb_lidar_mapper/calibration_analysis.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <set>
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

constexpr std::size_t kMaximumSharpnessPairs = 128;
constexpr std::size_t kMaximumPointsPerSharpnessScan = 96;

std::size_t sectorForYaw(double yaw, double origin) {
  double relative = Pose2::normalizeAngle(yaw - origin);
  if (relative < 0.0) relative += 2.0 * kPi;
  return std::min<std::size_t>(7, static_cast<std::size_t>(
      relative / (2.0 * kPi / 8.0)));
}

std::size_t sectorSeparation(std::size_t first, std::size_t second) {
  const auto forward = (second + 8U - first) % 8U;
  return std::min(forward, 8U - forward);
}

std::vector<std::size_t> deterministicPointIndices(std::size_t count) {
  const auto kept = std::min(count, kMaximumPointsPerSharpnessScan);
  std::vector<std::size_t> indices;
  indices.reserve(kept);
  if (kept == 0U) return indices;
  if (kept == 1U) {
    indices.push_back(0U);
    return indices;
  }
  for (std::size_t i = 0; i < kept; ++i) {
    indices.push_back(i * (count - 1U) / (kept - 1U));
  }
  return indices;
}

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
                      const TimedPoseBuffer& odom,
                      const std::vector<SharpnessPair>& pairs,
                      double offset) {
  std::vector<double> pair_scores;
  pair_scores.reserve(pairs.size());
  for (const auto& pair : pairs) {
    if (pair.source_index >= scans.size() || pair.target_index >= scans.size()) continue;
    const auto source_pose = odom.interpolate(scans[pair.source_index].reference_stamp_ns);
    const auto target_pose = odom.interpolate(scans[pair.target_index].reference_stamp_ns);
    if (!source_pose || !target_pose || scans[pair.source_index].points.empty() ||
        scans[pair.target_index].points.empty()) continue;
    const auto source_points = transformPoints(scans[pair.source_index], *source_pose, offset);
    const auto target_points = transformPoints(scans[pair.target_index], *target_pose, offset);
    const auto source_indices = deterministicPointIndices(source_points.size());
    const auto target_indices = deterministicPointIndices(target_points.size());
    if (source_indices.size() < 5U || target_indices.size() < 5U) continue;
    auto target_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    target_cloud->reserve(target_indices.size());
    for (const auto index : target_indices) target_cloud->push_back(target_points[index]);
    target_cloud->width = static_cast<std::uint32_t>(target_cloud->size());
    target_cloud->height = 1;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target_cloud);
    std::vector<double> residuals;
    residuals.reserve(source_indices.size());
    for (const auto index : source_indices) {
      std::vector<int> indices(1);
      std::vector<float> distances(1);
      if (tree.nearestKSearch(source_points[index], 1, indices, distances) > 0) {
        residuals.push_back(std::sqrt(static_cast<double>(distances[0])));
      }
    }
    if (residuals.size() < 5U) continue;
    std::sort(residuals.begin(), residuals.end());
    const auto trim = static_cast<std::size_t>(std::floor(residuals.size() * kTrimFraction));
    const auto keep = residuals.size() - std::min(trim, residuals.size() - 1U);
    pair_scores.push_back(std::accumulate(
        residuals.begin(), residuals.begin() + keep, 0.0) / static_cast<double>(keep));
  }
  if (pair_scores.empty()) return std::numeric_limits<double>::infinity();
  return std::accumulate(pair_scores.begin(), pair_scores.end(), 0.0) /
         static_cast<double>(pair_scores.size());
}

std::vector<SharpnessPoint> makeGrid(const std::vector<DeskewedScan>& scans,
                                     const TimedPoseBuffer& odom,
                                     const std::vector<SharpnessPair>& pairs,
                                     double start, double end, double step) {
  std::vector<SharpnessPoint> points;
  for (double offset = start; offset <= end + step * 0.1; offset += step) {
    points.push_back({offset, sharpnessScore(scans, odom, pairs, offset)});
  }
  return points;
}

const SharpnessPoint* minimumPoint(const std::vector<SharpnessPoint>& points) {
  const SharpnessPoint* best = nullptr;
  for (const auto& point : points) if (std::isfinite(point.score) && (!best || point.score < best->score)) best = &point;
  return best;
}

bool sharpnessMinimumIsUniqueImpl(const std::vector<SharpnessPoint>& points) {
  const auto* best = minimumPoint(points);
  if (!best) return false;
  double second_best = std::numeric_limits<double>::infinity();
  for (const auto& point : points) {
    if (&point != best) second_best = std::min(second_best, point.score);
  }
  return second_best > best->score * (1.0 + 1e-6);
}

}  // namespace

std::vector<SharpnessPair> selectSharpnessPairs(
    const std::vector<DeskewedScan>& scans, const TimedPoseBuffer& odom,
    std::size_t maximum_pairs) {
  if (maximum_pairs == 0U) return {};
  struct ValidScan {
    std::size_t index{};
    Pose2 pose;
    std::size_t sector{};
  };
  std::vector<ValidScan> valid;
  for (std::size_t index = 0; index < scans.size(); ++index) {
    if (scans[index].points.empty()) continue;
    const auto pose = odom.interpolate(scans[index].reference_stamp_ns);
    if (pose) valid.push_back({index, *pose, 0});
  }
  if (valid.size() < 3U) return {};
  const double origin = valid.front().pose.yaw;
  for (auto& scan : valid) scan.sector = sectorForYaw(scan.pose.yaw, origin);

  std::array<std::vector<SharpnessPair>, 8> by_sector;
  for (std::size_t source = 0; source < valid.size(); ++source) {
    for (std::size_t target = source + 2U; target < valid.size(); ++target) {
      const auto source_id = scans[valid[source].index].scan_id;
      const auto target_id = scans[valid[target].index].scan_id;
      if (target_id <= source_id || target_id - source_id <= 1U) continue;
      if (sectorSeparation(valid[source].sector, valid[target].sector) < 2U) continue;
      by_sector[valid[source].sector].push_back(
          {valid[source].index, valid[target].index});
    }
  }
  std::vector<SharpnessPair> result;
  result.reserve(std::min(maximum_pairs, kMaximumSharpnessPairs));
  std::array<std::size_t, 8> next{};
  const auto bound = std::min(maximum_pairs, kMaximumSharpnessPairs);
  while (result.size() < bound) {
    bool added = false;
    for (std::size_t sector = 0; sector < by_sector.size() && result.size() < bound; ++sector) {
      if (next[sector] >= by_sector[sector].size()) continue;
      result.push_back(by_sector[sector][next[sector]++]);
      added = true;
    }
    if (!added) break;
  }
  return result;
}

bool sharpnessMinimumIsUnique(const std::vector<SharpnessPoint>& points) {
  return sharpnessMinimumIsUniqueImpl(points);
}

MethodEstimate robustMethodEstimate(
  DeskewMethod method, const std::vector<CenterSample>& samples, std::uint64_t seed) {
  return robustMethodEstimateImpl(method, samples, seed);
}

SharpnessResult evaluateMapSharpness(
    const RotationDataset&, const std::vector<DeskewedScan>& scans,
    const TimedPoseBuffer& odom, double consensus_hint) {
  SharpnessResult result;
  const auto pairs = selectSharpnessPairs(scans, odom);
  result.coarse = makeGrid(scans, odom, pairs, 0.180, 0.340, 0.002);
  const auto* coarse_min = minimumPoint(result.coarse);
  if (!coarse_min) {
    result.rejection_reason = "insufficient_sharpness_correspondences";
    return result;
  }
  const double refine_start = std::max(0.180, coarse_min->offset_m - 0.002);
  const double refine_end = std::min(0.340, coarse_min->offset_m + 0.002);
  result.refined = makeGrid(scans, odom, pairs, refine_start, refine_end, 0.00025);
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
  if (!sharpnessMinimumIsUnique(result.refined)) {
    result.rejection_reason = "sharpness_non_unique";
    return result;
  }
  const double left = sharpnessScore(scans, odom, pairs, best->offset_m - 0.020);
  const double right = sharpnessScore(scans, odom, pairs, best->offset_m + 0.020);
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
  std::set<DeskewMethod> reliable_methods;
  for (const auto& estimate : estimates) {
    if (!estimate.reliable) continue;
    if (!reliable_methods.insert(estimate.method).second) {
      result.reason = "duplicate_method_estimate";
      return result;
    }
    reliable.push_back(&estimate);
  }
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
