#pragma once

#include <Eigen/Geometry>

#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {

struct StaticCameraMount {
  Eigen::Isometry3d T_base_camera_link{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_camera_link_left_optical{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_base_left_optical() const {
    return T_base_camera_link * T_camera_link_left_optical;
  }
};

// Lever arm c: planar vector from left-camera horizontal origin to rotation
// center, expressed in the horizontal left-camera frame (same as centerFromTransform).
// For pure spin about base origin, in base horizontal frame the optical origin is at
// p_base, and c_base = -p_base_xy (center at origin). Convert frames carefully.
struct MountXy {
  double x_m{};
  double y_m{};
};

// Given estimated c in horizontal-left frame and fixed mount orientations,
// return implied base_link→camera_link translation xy (z kept from recorded).
MountXy impliedCameraLinkXy(const Point2& center_in_horizontal_left,
                            const StaticCameraMount& recorded);

}  // namespace orb_slam3_wrapper
