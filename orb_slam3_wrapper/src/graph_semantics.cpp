#include "orb_slam3_wrapper/graph_semantics.hpp"

#include <set>
#include <utility>
#include <unordered_map>

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

GraphDeltaEvidence classifyGraphDeltaEvidence(
    const std::optional<ORB_SLAM3::GraphSnapshot>& previous,
    const ORB_SLAM3::GraphSnapshot& current) {
  GraphDeltaEvidence evidence;
  
  const auto current_edges = edges(current);
  std::unordered_map<std::uint64_t, std::uint64_t> map_id_lookup;
  for (const auto& keyframe : current.keyframes) {
      map_id_lookup[keyframe.id] = keyframe.map_id;
  }

  bool new_loop = false;
  bool created = false;
  
  if (previous) {
    const auto previous_edges = edges(*previous);
    for (const auto& edge : current_edges) {
      if (!previous_edges.count(edge)) { 
          new_loop = true; 
          LoopEdgeEvidence ev;
          ev.first_keyframe_id = edge.first;
          ev.second_keyframe_id = edge.second;
          ev.first_map_id = map_id_lookup[edge.first];
          ev.second_map_id = map_id_lookup[edge.second];
          ev.active_map_id = current.active_map_id;
          ev.classification = (ev.first_map_id == current.active_map_id && ev.second_map_id == current.active_map_id) ? "same_map_loop" : "cross_map_loop";
          evidence.loop_edges.push_back(ev);
      }
    }
    evidence.map_merged = (!previous->active_map_connected && current.active_map_connected) ||
             (current.active_map_connected && hasMultipleMapLineage(current) &&
              !hasMultipleMapLineage(*previous));
    created = current.active_map_id != previous->active_map_id && !current.active_map_connected;
  }
  
  if (new_loop) evidence.event_types.push_back(orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
  if (evidence.map_merged) evidence.event_types.push_back(orb_slam3_msgs::msg::TrackingEvent::MAP_MERGED);
  if (created) evidence.event_types.push_back(orb_slam3_msgs::msg::TrackingEvent::MAP_CREATED);
  return evidence;
}

std::vector<std::uint8_t> classifyGraphDelta(
    const std::optional<ORB_SLAM3::GraphSnapshot>& previous,
    const ORB_SLAM3::GraphSnapshot& current) {
  return classifyGraphDeltaEvidence(previous, current).event_types;
}

}  // namespace orb_slam3_wrapper
