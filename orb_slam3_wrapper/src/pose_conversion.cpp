#include "orb_slam3_wrapper/pose_conversion.hpp"

#include <stdexcept>

namespace orb_slam3_wrapper {
namespace {
Eigen::Isometry3d asIsometry(const Eigen::Matrix4d& matrix) {
  Eigen::Isometry3d result;
  result.matrix() = matrix;
  return result;
}
}

PoseConverter::PoseConverter(const Eigen::Isometry3d& T_base_left_camera)
: T_camera_base_(T_base_left_camera.inverse()), T_orb_map_world_(Eigen::Isometry3d::Identity()) {}

Eigen::Isometry3d PoseConverter::worldCameraToBase(const Sophus::SE3f& T_world_camera) const {
  return asIsometry(T_world_camera.matrix().cast<double>()) * T_camera_base_;
}

Eigen::Isometry3d PoseConverter::anchor(const Sophus::SE3f& T_world_camera) {
  T_orb_map_world_ = worldCameraToBase(T_world_camera).inverse();
  initialized_ = true;
  return toBasePose(T_world_camera);
}

Eigen::Isometry3d PoseConverter::toBasePose(const Sophus::SE3f& T_world_camera) const {
  if (!initialized_) {
    throw std::logic_error("cannot convert an ORB pose before anchoring");
  }
  return T_orb_map_world_ * worldCameraToBase(T_world_camera);
}

Eigen::Isometry3d PoseConverter::referenceToFrame(
    const Sophus::SE3f& T_world_reference_camera,
    const Sophus::SE3f& T_world_current_camera) const {
  return asIsometry((T_world_reference_camera.inverse() * T_world_current_camera).matrix().cast<double>());
}

Eigen::Isometry3d PoseConverter::referenceToBaseFrame(
    const Sophus::SE3f& T_world_reference_camera,
    const Sophus::SE3f& T_world_current_camera) const {
  return toBasePose(T_world_reference_camera).inverse() * toBasePose(T_world_current_camera);
}

bool PoseConverter::initialized() const noexcept { return initialized_; }

}  // namespace orb_slam3_wrapper
