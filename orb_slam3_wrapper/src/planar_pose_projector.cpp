#include "orb_slam3_wrapper/planar_pose_projector.hpp"

#include <cmath>

namespace orb_slam3_wrapper {

PlanarPose projectToHorizontal(int64_t stamp_ns,
                               const Eigen::Isometry3d& T_world_optical,
                               const Eigen::Isometry3d& T_base_optical) {
  // T_world_base = T_world_optical * T_base_optical.inverse()
  // PlanarPose from T_world_base: x,y,yaw(Z); height_m = tz
  const Eigen::Isometry3d T_world_base =
      T_world_optical * T_base_optical.inverse();
  const Eigen::Vector3d t = T_world_base.translation();
  const Eigen::Matrix3d R = T_world_base.linear();
  // yaw about world Z from rotation matrix columns (atan2 of first column)
  const double yaw = std::atan2(R(1, 0), R(0, 0));

  PlanarPose out;
  out.stamp_ns = stamp_ns;
  out.pose.x = t.x();
  out.pose.y = t.y();
  out.pose.yaw = yaw;
  out.height_m = t.z();
  out.valid = std::isfinite(out.pose.x) && std::isfinite(out.pose.y) &&
              std::isfinite(out.pose.yaw) && std::isfinite(out.height_m);
  return out;
}

}  // namespace orb_slam3_wrapper
