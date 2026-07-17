#include "orb_slam3_wrapper/mount_xy_mapper.hpp"

#include <cmath>

namespace orb_slam3_wrapper {

MountXy impliedCameraLinkXy(const Point2& center_in_horizontal_left,
                            const StaticCameraMount& recorded) {
  // Horizontal-left axes = base xy rotated by yaw of T_base_left_optical only.
  // c is optical_origin → rotation_center in horizontal-left.
  // With rotation center at base origin: optical_origin_in_base_xy = -R * c
  // where R maps horizontal-left vectors into base xy.
  const Eigen::Isometry3d T_bo = recorded.T_base_left_optical();
  const Eigen::Matrix3d R_bo = T_bo.linear();
  const double yaw = std::atan2(R_bo(1, 0), R_bo(0, 0));
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  const double cx = center_in_horizontal_left.x;
  const double cy_cam = center_in_horizontal_left.y;
  // R_z(yaw) * c
  const double rx = cy * cx - sy * cy_cam;
  const double ry = sy * cx + cy * cy_cam;
  const double optical_x = -rx;
  const double optical_y = -ry;

  // T_base_optical.translation = t_bc + R_bc * t_co
  // Free variables: camera_link translation xy (z fixed from recorded).
  // camera_link_xy + (R_bc * t_co).xy = optical_xy_base
  const Eigen::Matrix3d R_bc = recorded.T_base_camera_link.linear();
  const Eigen::Vector3d t_co =
      recorded.T_camera_link_left_optical.translation();
  const Eigen::Vector3d optical_offset = R_bc * t_co;

  MountXy out;
  out.x_m = optical_x - optical_offset.x();
  out.y_m = optical_y - optical_offset.y();
  return out;
}

}  // namespace orb_slam3_wrapper
