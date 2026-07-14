#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <orb_slam3_msgs/msg/tracking_event.hpp>
#include <SystemSnapshots.h>

namespace orb_slam3_wrapper {

std::vector<std::uint8_t> classifyGraphDelta(
    const std::optional<ORB_SLAM3::GraphSnapshot>& previous,
    const ORB_SLAM3::GraphSnapshot& current);

inline std::vector<std::uint8_t> classifyGraphDelta(
    const ORB_SLAM3::GraphSnapshot& previous,
    const ORB_SLAM3::GraphSnapshot& current) {
  return classifyGraphDelta(std::optional<ORB_SLAM3::GraphSnapshot>(previous), current);
}

}  // namespace orb_slam3_wrapper
