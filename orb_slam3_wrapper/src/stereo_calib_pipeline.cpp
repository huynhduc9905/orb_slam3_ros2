#include "orb_slam3_wrapper/stereo_calib_pipeline.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "orb_slam3_wrapper/backend.hpp"
#include "orb_slam3_wrapper/calibration.hpp"
#include "orb_slam3_wrapper/stereo_calib_analysis.hpp"
#include "orb_slam3_wrapper/stereo_rotation_center.hpp"

namespace orb_slam3_wrapper {
namespace {

// ORB_SLAM3::Tracking::eTrackingState::OK
constexpr int kTrackingOk = 2;

void reportStage(int percent, const char* label) {
  std::cerr << '[' << std::setw(3) << percent << "%] " << label << '\n'
            << std::flush;
}

void validateConfig(const StereoCalibConfig& config) {
  if (config.bag_path.empty()) {
    throw std::invalid_argument("stereo calib: bag_path is empty");
  }
  if (config.vocabulary_file.empty()) {
    throw std::invalid_argument("stereo calib: vocabulary_file is empty");
  }
  if (config.settings_file.empty()) {
    throw std::invalid_argument("stereo calib: settings_file is empty");
  }
  if (!std::isfinite(config.max_tracking_loss_fraction) ||
      config.max_tracking_loss_fraction < 0.0 ||
      config.max_tracking_loss_fraction > 1.0) {
    throw std::invalid_argument(
        "stereo calib: max_tracking_loss_fraction out of range");
  }
}

// Build planar left-optical pose from projectToHorizontal base pose + optical
// offset in base (observability-correct lever arm for pure spin).
PlanarPose planarLeftFromBasePlanar(const PlanarPose& base_planar,
                                    const Eigen::Isometry3d& T_base_optical) {
  PlanarPose left;
  left.stamp_ns = base_planar.stamp_ns;
  left.height_m = base_planar.height_m;
  left.valid = base_planar.valid;
  if (!base_planar.valid) {
    return left;
  }
  const double c = std::cos(base_planar.pose.yaw);
  const double s = std::sin(base_planar.pose.yaw);
  const Eigen::Vector3d p = T_base_optical.translation();
  left.pose.x = base_planar.pose.x + c * p.x() - s * p.y();
  left.pose.y = base_planar.pose.y + s * p.x() + c * p.y();
  left.pose.yaw = base_planar.pose.yaw;
  left.valid = std::isfinite(left.pose.x) && std::isfinite(left.pose.y) &&
               std::isfinite(left.pose.yaw) && std::isfinite(left.height_m);
  return left;
}

std::vector<StereoCenterSample> buildSamples(
    const std::vector<PlanarPose>& planar_left,
    const StaticCameraMount& mount, const StereoThresholds& thresholds) {
  const auto pairs =
      selectPosePairs(planar_left, /*intervals=*/{}, thresholds.min_pair_yaw_rad,
                      thresholds.max_pair_yaw_rad);

  std::vector<StereoCenterSample> samples;
  samples.reserve(pairs.size());
  for (const auto& [src, tgt] : pairs) {
    StereoCenterSample sample;
    sample.source_index = src;
    sample.target_index = tgt;
    sample.yaw_sector = sectorForYaw(planar_left[src].pose.yaw);

    const Pose2 source_to_target =
        planar_left[tgt].pose.inverse().compose(planar_left[src].pose);
    const auto center = centerFromTransform(source_to_target);
    if (!center.has_value()) {
      sample.accepted = false;
      sample.rejection_reason = "ill_conditioned_transform";
      samples.push_back(sample);
      continue;
    }
    if (!std::isfinite(center->x) || !std::isfinite(center->y) ||
        std::abs(center->x) > thresholds.max_abs_center_m ||
        std::abs(center->y) > thresholds.max_abs_center_m) {
      sample.accepted = false;
      sample.center = *center;
      sample.rejection_reason = "center_out_of_bounds";
      samples.push_back(sample);
      continue;
    }
    sample.accepted = true;
    sample.center = *center;
    sample.mount_xy = impliedCameraLinkXy(*center, mount);
    samples.push_back(sample);
  }
  return samples;
}

}  // namespace

Eigen::Isometry3d sophusSe3fToIsometry(const Sophus::SE3f& T_world_camera) {
  Eigen::Isometry3d result;
  result.matrix() = T_world_camera.matrix().cast<double>();
  return result;
}

PlanarPose planarLeftFromBasePose(int64_t stamp_ns,
                                  const Eigen::Isometry3d& T_world_base,
                                  const Eigen::Isometry3d& T_base_optical) {
  // Reuse projectToHorizontal with identity optical so T_world_base is used as
  // the "base" pose (T_world_optical=T_world_base, T_base_optical=I → base).
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const PlanarPose base =
      projectToHorizontal(stamp_ns, T_world_base, identity);
  return planarLeftFromBasePlanar(base, T_base_optical);
}

StereoCalibRun estimateFromPlanarPoses(
    const StereoThresholds& thresholds, const StaticCameraMount& mount,
    const MountXy& recorded_xy, const std::vector<PlanarPose>& planar_left,
    std::uint64_t seed) {
  StereoCalibRun run;
  run.config.thresholds = thresholds;
  run.planar = planar_left;
  run.samples = buildSamples(planar_left, mount, thresholds);
  const auto estimate = robustEstimate(run.samples, seed, thresholds);
  run.aggregate = classify(estimate, recorded_xy, thresholds);
  return run;
}

void assertTrackingGates(std::size_t tracked_ok, std::size_t tracked_total,
                         const StereoCalibConfig& config) {
  if (tracked_ok < config.min_tracked_frames) {
    throw std::runtime_error(
        "ORB tracking gate failed: tracked_ok=" + std::to_string(tracked_ok) +
        " < min_tracked_frames=" + std::to_string(config.min_tracked_frames));
  }
  const double loss_fraction =
      tracked_total == 0
          ? 1.0
          : 1.0 - static_cast<double>(tracked_ok) /
                     static_cast<double>(tracked_total);
  if (loss_fraction > config.max_tracking_loss_fraction) {
    throw std::runtime_error(
        "ORB tracking gate failed: loss_fraction=" +
        std::to_string(loss_fraction) + " > max_tracking_loss_fraction=" +
        std::to_string(config.max_tracking_loss_fraction));
  }
}

StereoCalibRun runStereoCalibration(const StereoCalibConfig& config) {
  validateConfig(config);

  StereoCalibRun run;
  run.config = config;

  reportStage(5, "bag");
  run.dataset = StereoBagReader::read(config.bag_path);

  reportStage(20, "ORB");
  const auto calibration = Calibration::fromCameraInfo(
      run.dataset.left_info, run.dataset.right_info,
      run.dataset.left_optical_frame, run.dataset.right_optical_frame);

  OrbSlam3Backend backend(config.vocabulary_file.string(),
                          config.settings_file.string());
  std::string error;
  if (!backend.configureCalibration(calibration, error)) {
    throw std::runtime_error("ORB configureCalibration failed: " + error);
  }

  const Eigen::Isometry3d T_base_optical =
      run.dataset.recorded_mount.T_base_left_optical();

  run.trajectory.reserve(run.dataset.frames.size());
  run.planar.reserve(run.dataset.frames.size());
  run.tracked_total = run.dataset.frames.size();
  run.tracked_ok = 0;

  for (const auto& frame : run.dataset.frames) {
    const double stamp_sec =
        static_cast<double>(frame.stamp_ns) * 1e-9;
    const auto snap =
        backend.trackStereo(frame.left_bgr_or_gray, frame.right_bgr_or_gray,
                            stamp_sec);

    TrackedPose tracked;
    tracked.stamp_ns = frame.stamp_ns;
    tracked.tracking_state = snap.tracking_state;
    tracked.pose_valid = snap.pose_valid;
    if (snap.pose_valid) {
      tracked.T_world_optical = sophusSe3fToIsometry(snap.T_world_camera);
    }

    const bool ok =
        snap.pose_valid && snap.tracking_state == kTrackingOk;
    if (ok) {
      ++run.tracked_ok;
      // projectToHorizontal yields BASE planar; lift optical origin for
      // centerFromTransform observability (Task 4 pure-spin lock).
      const PlanarPose base_planar = projectToHorizontal(
          frame.stamp_ns, tracked.T_world_optical, T_base_optical);
      const PlanarPose left =
          planarLeftFromBasePlanar(base_planar, T_base_optical);
      if (left.valid) {
        run.planar.push_back(left);
      }
    }
    run.trajectory.push_back(std::move(tracked));
  }

  assertTrackingGates(run.tracked_ok, run.tracked_total, config);

  reportStage(70, "pairs");
  run.samples = buildSamples(run.planar, run.dataset.recorded_mount,
                             config.thresholds);

  reportStage(90, "estimate");
  const auto estimate =
      robustEstimate(run.samples, /*seed=*/42, config.thresholds);
  run.aggregate = classify(estimate, run.dataset.recorded_camera_link_xy,
                           config.thresholds);

  reportStage(100, "report");
  return run;
}

}  // namespace orb_slam3_wrapper
