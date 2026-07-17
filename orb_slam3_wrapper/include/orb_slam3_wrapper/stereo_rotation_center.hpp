#pragma once

#include <optional>

#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {

// Solve planar rotation center c from relative SE(2) source_to_target:
//   p_target = R(yaw) * p_source + t
//   c = (I - R)^{-1} * t
//
// Relative pose convention (locked by pure-spin unit test):
//   source_to_target = pose_target.inverse().compose(pose_source)
// For a camera optical origin fixed at body offset p spinning about the world
// origin, this recovers c = -p in the horizontal-left frame.
std::optional<Point2> centerFromTransform(const Pose2& source_to_target);

}  // namespace orb_slam3_wrapper
