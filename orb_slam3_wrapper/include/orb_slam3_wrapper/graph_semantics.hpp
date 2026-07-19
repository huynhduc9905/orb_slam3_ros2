#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <orb_slam3_msgs/msg/tracking_event.hpp>
#include <SystemSnapshots.h>

namespace orb_slam3_wrapper {

struct LoopEdgeEvidence {
  std::uint64_t first_keyframe_id;
  std::uint64_t second_keyframe_id;
  std::uint64_t first_map_id;
  std::uint64_t second_map_id;
  std::uint64_t active_map_id;
  std::string classification;
};

struct GraphDeltaEvidence {
  std::vector<std::uint8_t> event_types;
  std::vector<LoopEdgeEvidence> loop_edges;
  bool map_merged{false};
};

GraphDeltaEvidence classifyGraphDeltaEvidence(
    const std::optional<ORB_SLAM3::GraphSnapshot>& previous,
    const ORB_SLAM3::GraphSnapshot& current);

std::vector<std::uint8_t> classifyGraphDelta(
    const std::optional<ORB_SLAM3::GraphSnapshot>& previous,
    const ORB_SLAM3::GraphSnapshot& current);

inline std::vector<std::uint8_t> classifyGraphDelta(
    const ORB_SLAM3::GraphSnapshot& previous,
    const ORB_SLAM3::GraphSnapshot& current) {
  return classifyGraphDelta(std::optional<ORB_SLAM3::GraphSnapshot>(previous), current);
}

}  // namespace orb_slam3_wrapper
