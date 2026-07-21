#include <gtest/gtest.h>
#include <gmock/gmock.h>

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

TEST(GraphSemantics, FirstSnapshotEstablishesBaselineWithoutEvents) {
  EXPECT_TRUE(orb_slam3_wrapper::classifyGraphDelta(std::nullopt, graph(1, 17, false)).empty());
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

TEST(GraphSemantics, MergeAndLoopEvidenceAreEmittedOnlyOnTheirDeltas) {
  auto previous = graph(1, 17, false);
  ORB_SLAM3::KeyframeSnapshot a; a.id = 10; a.map_id = 17;
  ORB_SLAM3::KeyframeSnapshot b; b.id = 20; b.map_id = 23; b.loop_edge_ids = {10};
  auto merged = graph(2, 23, true); merged.keyframes = {a, b};
  const auto first = orb_slam3_wrapper::classifyGraphDelta(previous, merged);
  EXPECT_EQ(first.size(), 2u);
  EXPECT_TRUE(orb_slam3_wrapper::classifyGraphDelta(merged, graph(3, 23, true)).empty());
  EXPECT_TRUE(orb_slam3_wrapper::classifyGraphDelta(merged, merged).empty());
}

TEST(GraphSemantics, NewCanonicalLoopEdgeProducesLoopClosedEvidence) {
  auto previous = graph(1, 17, true);
  auto current = graph(2, 17, true);
  ORB_SLAM3::KeyframeSnapshot a; a.id = 10; a.map_id = 17;
  ORB_SLAM3::KeyframeSnapshot b; b.id = 20; b.map_id = 17; b.loop_edge_ids = {10};
  current.keyframes = {a, b};

  const auto evidence = orb_slam3_wrapper::classifyGraphDeltaEvidence(previous, current);

  EXPECT_EQ(evidence.loop_edges.size(), 1U);
  EXPECT_EQ(evidence.loop_edges[0].first_keyframe_id, 10U);
  EXPECT_EQ(evidence.loop_edges[0].second_keyframe_id, 20U);
  EXPECT_EQ(evidence.loop_edges[0].classification, "same_map_loop");
  EXPECT_THAT(evidence.event_types,
              ::testing::Contains(orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED));
}

TEST(GraphSemantics, CrossMapLoopProducesCrossMapClassification) {
  auto previous = graph(1, 17, false);
  auto current = graph(2, 23, true);
  ORB_SLAM3::KeyframeSnapshot a; a.id = 10; a.map_id = 17;
  ORB_SLAM3::KeyframeSnapshot b; b.id = 20; b.map_id = 23; b.loop_edge_ids = {10};
  current.keyframes = {a, b};

  const auto evidence = orb_slam3_wrapper::classifyGraphDeltaEvidence(previous, current);

  EXPECT_EQ(evidence.loop_edges.size(), 1U);
  EXPECT_EQ(evidence.loop_edges[0].first_keyframe_id, 10U);
  EXPECT_EQ(evidence.loop_edges[0].second_keyframe_id, 20U);
  EXPECT_EQ(evidence.loop_edges[0].first_map_id, 17U);
  EXPECT_EQ(evidence.loop_edges[0].second_map_id, 23U);
  EXPECT_EQ(evidence.loop_edges[0].active_map_id, 23U);
  EXPECT_EQ(evidence.loop_edges[0].classification, "cross_map_loop");
  EXPECT_THAT(evidence.event_types,
              ::testing::Contains(orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED));
  EXPECT_TRUE(evidence.map_merged);
}
