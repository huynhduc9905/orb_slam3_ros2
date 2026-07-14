#pragma once

#include <Eigen/Geometry>

#include <sophus/se3.hpp>

namespace orb_slam3_wrapper {

// T_A_B maps coordinates expressed in frame B into frame A.
class PoseConverter {
public:
  explicit PoseConverter(const Eigen::Isometry3d& T_base_left_camera);

  Eigen::Isometry3d anchor(const Sophus::SE3f& T_world_camera);
  Eigen::Isometry3d toBasePose(const Sophus::SE3f& T_world_camera) const;
  // Returns T_reference_current = T_world_reference^-1 * T_world_current.
  Eigen::Isometry3d referenceToFrame(
      const Sophus::SE3f& T_world_reference_camera,
      const Sophus::SE3f& T_world_current_camera) const;
  Eigen::Isometry3d referenceToBaseFrame(
      const Sophus::SE3f& T_world_reference_camera,
      const Sophus::SE3f& T_world_current_camera) const;
  Eigen::Isometry3d referenceToBaseFrame(
      const Sophus::SE3f& T_reference_camera_current_camera) const;
  bool initialized() const noexcept;

private:
  Eigen::Isometry3d worldCameraToBase(const Sophus::SE3f& T_world_camera) const;

  Eigen::Isometry3d T_camera_base_;
  Eigen::Isometry3d T_orb_map_world_;
  bool initialized_{false};
};

}  // namespace orb_slam3_wrapper
