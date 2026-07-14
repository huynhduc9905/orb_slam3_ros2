#include <gtest/gtest.h>

#include <orb_slam3_wrapper/graph_semantics.hpp>

namespace {
ORB_SLAM3::GraphSnapshot graph(std::uint64_t revision, std::uint64_t map_id, bool connected) {
  ORB_SLAM3::GraphSnapshot result;
  result.revision = revision;
  result.active_map_id = map_id;
  result.active_map_connected = connected;
  return result;
}
}

TEST(GraphSemantics, RevisionOnlyChangeHasNoSemanticEvent) {
  const auto previous = graph(1, 17, true);
  const auto current = graph(2, 17, true);
  EXPECT_TRUE(orb_slam3_wrapper::classifyGraphDelta(previous, current).empty());
}

TEST(GraphSemantics, NewCanonicalLoopEdgeProducesLoopClosed) {
  auto previous = graph(1, 17, true);
  auto current = graph(2, 17, true);
  ORB_SLAM3::KeyframeSnapshot a; a.id = 10; a.map_id = 17;
  ORB_SLAM3::KeyframeSnapshot b; b.id = 20; b.map_id = 17; b.loop_edge_ids = {10};
  current.keyframes = {a, b};
  EXPECT_EQ(orb_slam3_wrapper::classifyGraphDelta(previous, current),
            (std::vector<std::uint8_t>{orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED}));
}

TEST(GraphSemantics, ConnectedLineageProducesMapMergedAndCanAlsoCloseLoop) {
  auto previous = graph(1, 17, false);
  auto current = graph(2, 23, true);
  ORB_SLAM3::KeyframeSnapshot a; a.id = 10; a.map_id = 17;
  ORB_SLAM3::KeyframeSnapshot b; b.id = 20; b.map_id = 23; b.loop_edge_ids = {10};
  current.keyframes = {a, b};
  const auto events = orb_slam3_wrapper::classifyGraphDelta(previous, current);
  EXPECT_EQ(events, (std::vector<std::uint8_t>{
      orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED,
      orb_slam3_msgs::msg::TrackingEvent::MAP_MERGED}));
}

TEST(GraphSemantics, DisconnectedNewMapProducesMapCreated) {
  const auto previous = graph(1, 17, true);
  const auto current = graph(2, 23, false);
  EXPECT_EQ(orb_slam3_wrapper::classifyGraphDelta(previous, current),
            (std::vector<std::uint8_t>{orb_slam3_msgs::msg::TrackingEvent::MAP_CREATED}));
}
