#pragma once

#include <cstdint>

#include <Eigen/Geometry>

#include "orb_slam3_wrapper/stereo_calib_types.hpp"

namespace orb_slam3_wrapper {

struct PlanarPose {
  int64_t stamp_ns{};
  Pose2 pose;         // horizontal world: x, y, yaw
  double height_m{};  // vertical residual diagnostic
  bool valid{};
};

// Project ORB left-optical pose into base-aligned horizontal SE(2).
//
// Contract:
//   T_world_base = T_world_optical * T_base_optical.inverse()
//   PlanarPose.pose = {tx, ty, yaw_from_rotation_matrix_Z of T_world_base}
//   height_m = tz of T_world_base
//   valid = true when all extracted values are finite
//
// T_world_optical: left optical frame in ORB world
// T_base_optical: fixed optical mount expressed in base_link (from TF)
PlanarPose projectToHorizontal(int64_t stamp_ns,
                               const Eigen::Isometry3d& T_world_optical,
                               const Eigen::Isometry3d& T_base_optical);

}  // namespace orb_slam3_wrapper
