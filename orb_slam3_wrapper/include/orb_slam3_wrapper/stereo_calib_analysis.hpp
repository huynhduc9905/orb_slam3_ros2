#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "orb_slam3_wrapper/planar_pose_projector.hpp"
#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {

// Pair selection from planar trajectory.
// Empty intervals = all poses eligible.
// Relative source_to_target for centerFromTransform:
//   source_to_target = pose_target.inverse().compose(pose_source)
// (locks pure-spin recovery of lever arm c = -p for optical origin at body
// offset p spinning about world origin).
std::vector<std::pair<std::size_t, std::size_t>> selectPosePairs(
    const std::vector<PlanarPose>& poses,
    const std::vector<MotionInterval>& intervals, double min_yaw,
    double max_yaw);

StereoEstimate robustEstimate(const std::vector<StereoCenterSample>& samples,
                              std::uint64_t seed,
                              const StereoThresholds& thresholds);

StereoAggregate classify(const StereoEstimate& estimate,
                         const MountXy& recorded_xy,
                         const StereoThresholds& thresholds);

// Scientific exit codes: Consistent=0, LikelyOffsetError=2, Inconclusive=3.
int resultExitCode(ResultClass result_class);

}  // namespace orb_slam3_wrapper
