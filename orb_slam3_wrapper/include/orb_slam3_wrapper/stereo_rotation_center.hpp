#pragma once

#include <optional>

#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {

std::optional<Point2> centerFromTransform(const Pose2& source_to_target);

}  // namespace orb_slam3_wrapper
