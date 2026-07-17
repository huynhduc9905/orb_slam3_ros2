#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "orb_lidar_mapper/calibration_types.hpp"

namespace orb_lidar_mapper {

struct IcpConfig {
  double max_correspondence_m{0.15};
  int max_iterations{60};
  double minimum_overlap{0.40};
  double maximum_trimmed_rmse_m{0.05};
  double maximum_yaw_error_rad{2.0 * kPi / 180.0};
};

struct IcpResult {
  bool converged{};
  Pose2 source_to_target;
  double overlap_ratio{};
  double trimmed_rmse_m{};
  std::size_t correspondence_count{};
  std::string rejection_reason;
};

class PlanarIcp {
 public:
  explicit PlanarIcp(IcpConfig config) : config_(config) {}

  IcpResult align(const std::vector<Point2>& source,
                  const std::vector<Point2>& target,
                  double expected_yaw_rad) const;

 private:
  IcpConfig config_;
};

}  // namespace orb_lidar_mapper
