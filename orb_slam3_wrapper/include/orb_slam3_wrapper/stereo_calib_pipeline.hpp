#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "orb_slam3_wrapper/mount_xy_mapper.hpp"
#include "orb_slam3_wrapper/planar_pose_projector.hpp"
#include "orb_slam3_wrapper/stereo_bag_reader.hpp"
#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {

struct StereoCalibConfig {
  std::filesystem::path bag_path;
  std::filesystem::path output_dir;
  std::filesystem::path vocabulary_file;
  std::filesystem::path settings_file;
  bool overwrite{false};
  StereoThresholds thresholds{};
  // ORB tracking gates
  double max_tracking_loss_fraction{0.25};
  std::size_t min_tracked_frames{200};
};

struct TrackedPose {
  int64_t stamp_ns{};
  Eigen::Isometry3d T_world_optical{Eigen::Isometry3d::Identity()};
  int tracking_state{};
  bool pose_valid{};
};

struct StereoCalibRun {
  StereoCalibConfig config;
  StereoDataset dataset;
  std::vector<TrackedPose> trajectory;
  std::vector<PlanarPose> planar;  // planar LEFT-optical poses used for pairs
  std::vector<StereoCenterSample> samples;
  StereoAggregate aggregate;
  std::size_t tracked_ok{};
  std::size_t tracked_total{};
};

// Convert Sophus SE3f (ORB) → Eigen Isometry3d.
Eigen::Isometry3d sophusSe3fToIsometry(const Sophus::SE3f& T_world_camera);

// Planar pose of LEFT OPTICAL origin in world horizontal from base pose SE3
// (xy + Z-yaw of base) and fixed T_base_optical:
//   left.xy = base.xy + R_z(base.yaw) * p_optical_in_base.xy
//   left.yaw = base.yaw
PlanarPose planarLeftFromBasePose(int64_t stamp_ns,
                                  const Eigen::Isometry3d& T_world_base,
                                  const Eigen::Isometry3d& T_base_optical);

// Synthetic / unit-test path (stages 4–5): pre-built planar LEFT-optical poses
// → select pairs → centerFromTransform → map xy → robustEstimate → classify.
// Does not call ORB or read bags.
StereoCalibRun estimateFromPlanarPoses(
    const StereoThresholds& thresholds, const StaticCameraMount& mount,
    const MountXy& recorded_xy, const std::vector<PlanarPose>& planar_left,
    std::uint64_t seed = 42);

// Full offline pipeline: bag → ORB track → planar left → estimate.
// Throws on bag/ORB init failure or operational tracking-gate failure.
StereoCalibRun runStereoCalibration(const StereoCalibConfig& config);

// Operational tracking gates (throw on failure). Exposed for unit tests.
void assertTrackingGates(std::size_t tracked_ok, std::size_t tracked_total,
                         const StereoCalibConfig& config);

}  // namespace orb_slam3_wrapper
