#include "orb_slam3_wrapper/stereo_calib_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace orb_slam3_wrapper {
namespace {

constexpr std::size_t kBootstrapSamples = 2000;
constexpr std::size_t kNumYawSectors = 8;

bool inInterval(std::int64_t stamp, const MotionInterval& interval) {
  return stamp >= interval.start_ns && stamp <= interval.end_ns;
}

bool inAnyInterval(std::int64_t stamp,
                   const std::vector<MotionInterval>& intervals) {
  if (intervals.empty()) {
    return true;
  }
  for (const auto& interval : intervals) {
    if (inInterval(stamp, interval)) {
      return true;
    }
  }
  return false;
}

std::size_t sectorForYaw(double yaw) {
  double heading = Pose2::normalizeAngle(yaw);
  if (heading < 0.0) {
    heading += 2.0 * kPi;
  }
  const auto sector = static_cast<std::size_t>(
      std::floor(heading * static_cast<double>(kNumYawSectors) / (2.0 * kPi)));
  return std::min(sector, kNumYawSectors - 1U);
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if (values.size() % 2 != 0) {
    return upper;
  }
  const auto lower = std::max_element(values.begin(), middle);
  return (*lower + upper) / 2.0;
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - static_cast<double>(lower);
  return values[lower] * (1.0 - alpha) + values[upper] * alpha;
}

bool finiteMount(const MountXy& xy) {
  return std::isfinite(xy.x_m) && std::isfinite(xy.y_m);
}

}  // namespace

std::vector<std::pair<std::size_t, std::size_t>> selectPosePairs(
    const std::vector<PlanarPose>& poses,
    const std::vector<MotionInterval>& intervals, double min_yaw,
    double max_yaw) {
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  if (poses.size() < 2U || min_yaw > max_yaw || min_yaw < 0.0) {
    return pairs;
  }
  for (std::size_t source = 0; source < poses.size(); ++source) {
    if (!poses[source].valid) {
      continue;
    }
    if (!inAnyInterval(poses[source].stamp_ns, intervals)) {
      continue;
    }
    for (std::size_t target = source + 1; target < poses.size(); ++target) {
      if (!poses[target].valid) {
        continue;
      }
      if (!inAnyInterval(poses[target].stamp_ns, intervals)) {
        continue;
      }
      // Both must share a common interval when intervals are provided.
      if (!intervals.empty()) {
        const bool share = std::any_of(
            intervals.begin(), intervals.end(), [&](const MotionInterval& iv) {
              return inInterval(poses[source].stamp_ns, iv) &&
                     inInterval(poses[target].stamp_ns, iv);
            });
        if (!share) {
          continue;
        }
      }
      const double delta =
          Pose2::normalizeAngle(poses[target].pose.yaw - poses[source].pose.yaw);
      const double absolute_delta = std::abs(delta);
      if (absolute_delta < min_yaw || absolute_delta > max_yaw) {
        continue;
      }
      pairs.emplace_back(source, target);
    }
  }
  return pairs;
}

StereoEstimate robustEstimate(const std::vector<StereoCenterSample>& samples,
                              std::uint64_t seed,
                              const StereoThresholds& thresholds) {
  StereoEstimate result;
  std::vector<const StereoCenterSample*> accepted;
  accepted.reserve(samples.size());
  for (const auto& sample : samples) {
    if (!sample.accepted) {
      continue;
    }
    if (!finiteMount(sample.mount_xy)) {
      continue;
    }
    accepted.push_back(&sample);
  }
  result.accepted_pairs = accepted.size();

  std::set<std::size_t> sectors;
  for (const auto* sample : accepted) {
    if (sample->yaw_sector < kNumYawSectors) {
      sectors.insert(sample->yaw_sector);
    }
  }
  result.sectors_used = sectors.size();

  if (accepted.empty()) {
    result.unreliable_reasons.push_back("no_accepted_pairs");
    result.reliable = false;
    return result;
  }

  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(accepted.size());
  ys.reserve(accepted.size());
  for (const auto* sample : accepted) {
    xs.push_back(sample->mount_xy.x_m);
    ys.push_back(sample->mount_xy.y_m);
  }
  result.median_xy.x_m = median(xs);
  result.median_xy.y_m = median(ys);

  // Bootstrap 95% CI half-width on median of each coordinate.
  std::mt19937_64 generator(seed);
  std::uniform_int_distribution<std::size_t> pick(0, xs.size() - 1);
  std::vector<double> boot_x;
  std::vector<double> boot_y;
  boot_x.reserve(kBootstrapSamples);
  boot_y.reserve(kBootstrapSamples);
  std::vector<double> resample_x;
  std::vector<double> resample_y;
  resample_x.reserve(xs.size());
  resample_y.reserve(ys.size());
  for (std::size_t iteration = 0; iteration < kBootstrapSamples; ++iteration) {
    resample_x.clear();
    resample_y.clear();
    for (std::size_t i = 0; i < xs.size(); ++i) {
      const std::size_t index = pick(generator);
      resample_x.push_back(xs[index]);
      resample_y.push_back(ys[index]);
    }
    boot_x.push_back(median(std::move(resample_x)));
    boot_y.push_back(median(std::move(resample_y)));
  }
  const double x_lo = percentile(boot_x, 0.025);
  const double x_hi = percentile(boot_x, 0.975);
  const double y_lo = percentile(boot_y, 0.025);
  const double y_hi = percentile(boot_y, 0.975);
  result.ci_half_width.x_m = (x_hi - x_lo) / 2.0;
  result.ci_half_width.y_m = (y_hi - y_lo) / 2.0;

  if (result.accepted_pairs < thresholds.min_accepted_pairs) {
    result.unreliable_reasons.push_back("insufficient_accepted_pairs");
  }
  if (result.sectors_used < thresholds.min_sectors) {
    result.unreliable_reasons.push_back("insufficient_yaw_sectors");
  }
  if (!std::isfinite(result.ci_half_width.x_m) ||
      result.ci_half_width.x_m > thresholds.max_ci_half_width_m) {
    result.unreliable_reasons.push_back("confidence_interval_x_too_wide");
  }
  if (!std::isfinite(result.ci_half_width.y_m) ||
      result.ci_half_width.y_m > thresholds.max_ci_half_width_m) {
    result.unreliable_reasons.push_back("confidence_interval_y_too_wide");
  }
  if (!std::isfinite(result.median_xy.x_m) ||
      !std::isfinite(result.median_xy.y_m) ||
      std::abs(result.median_xy.x_m) > thresholds.max_abs_center_m ||
      std::abs(result.median_xy.y_m) > thresholds.max_abs_center_m) {
    result.unreliable_reasons.push_back("median_out_of_bounds");
  }

  result.reliable = result.unreliable_reasons.empty();
  return result;
}

StereoAggregate classify(const StereoEstimate& estimate,
                         const MountXy& recorded_xy,
                         const StereoThresholds& thresholds) {
  StereoAggregate agg;
  agg.estimate = estimate;
  agg.recorded_xy = recorded_xy;
  agg.delta_xy.x_m = estimate.median_xy.x_m - recorded_xy.x_m;
  agg.delta_xy.y_m = estimate.median_xy.y_m - recorded_xy.y_m;

  if (!estimate.reliable) {
    agg.result_class = ResultClass::kInconclusive;
    agg.summary = estimate.unreliable_reasons.empty()
                      ? "inconclusive: estimate unreliable"
                      : "inconclusive: " + estimate.unreliable_reasons.front();
    return agg;
  }

  const double linf =
      std::max(std::abs(agg.delta_xy.x_m), std::abs(agg.delta_xy.y_m));
  const double ci_max =
      std::max(estimate.ci_half_width.x_m, estimate.ci_half_width.y_m);
  const double threshold = std::max(thresholds.agreement_floor_m, ci_max);

  if (linf <= threshold) {
    agg.result_class = ResultClass::kConsistent;
    agg.summary = "consistent";
  } else {
    agg.result_class = ResultClass::kLikelyOffsetError;
    agg.summary = "likely_offset_error";
  }
  return agg;
}

int resultExitCode(ResultClass result_class) {
  switch (result_class) {
    case ResultClass::kConsistent:
      return 0;
    case ResultClass::kLikelyOffsetError:
      return 2;
    case ResultClass::kInconclusive:
      return 3;
  }
  return 3;
}

}  // namespace orb_slam3_wrapper
