#include "orb_slam3_wrapper/graph_semantics.hpp"

#include <set>
#include <utility>

namespace orb_slam3_wrapper {
namespace {
using Edge = std::pair<std::uint64_t, std::uint64_t>;

std::set<Edge> edges(const ORB_SLAM3::GraphSnapshot& graph) {
  std::set<Edge> result;
  for (const auto& keyframe : graph.keyframes) {
    for (const auto id : keyframe.loop_edge_ids) {
      result.emplace(std::min(keyframe.id, id), std::max(keyframe.id, id));
    }
  }
  return result;
}

bool hasMultipleMapLineage(const ORB_SLAM3::GraphSnapshot& graph) {
  std::set<std::uint64_t> ids;
  for (const auto& keyframe : graph.keyframes) ids.insert(keyframe.map_id);
  return ids.size() > 1;
}
}  // namespace

std::vector<std::uint8_t> classifyGraphDelta(
    const std::optional<ORB_SLAM3::GraphSnapshot>& previous,
    const ORB_SLAM3::GraphSnapshot& current) {
  std::vector<std::uint8_t> result;
  const auto current_edges = edges(current);
  bool new_loop = false;
  bool merged = false;
  bool created = false;
  if (previous) {
    const auto previous_edges = edges(*previous);
    for (const auto& edge : current_edges) {
      if (!previous_edges.count(edge)) { new_loop = true; break; }
    }
    merged = (!previous->active_map_connected && current.active_map_connected) ||
             (current.active_map_connected && hasMultipleMapLineage(current) &&
              !hasMultipleMapLineage(*previous));
    created = current.active_map_id != previous->active_map_id && !current.active_map_connected;
  }
  if (new_loop) result.push_back(orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
  if (merged) result.push_back(orb_slam3_msgs::msg::TrackingEvent::MAP_MERGED);
  if (created) result.push_back(orb_slam3_msgs::msg::TrackingEvent::MAP_CREATED);
  return result;
}

}  // namespace orb_slam3_wrapper
