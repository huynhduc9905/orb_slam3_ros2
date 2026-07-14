#include <gtest/gtest.h>
#include <KeyFrame.h>
#include <Map.h>
#include <SystemSnapshots.h>

TEST(SnapshotValue, OwnsPoseAndRelationshipData) {
  ORB_SLAM3::KeyframeSnapshot keyframe;
  keyframe.id = 42;
  keyframe.map_id = 7;
  keyframe.T_world_camera = Sophus::SE3f();
  keyframe.bad = true;
  keyframe.has_parent = true;
  keyframe.parent_id = 3;
  keyframe.loop_edge_ids = {9, 11};

  ORB_SLAM3::GraphSnapshot copy;
  copy.revision = 5;
  copy.active_map_id = 7;
  copy.active_map_connected = true;
  copy.keyframes.push_back(keyframe);
  keyframe.loop_edge_ids.clear();

  ASSERT_EQ(copy.revision, 5U);
  ASSERT_TRUE(copy.active_map_connected);
  ASSERT_EQ(copy.keyframes.front().id, 42U);
  ASSERT_TRUE(copy.keyframes.front().bad);
  ASSERT_EQ(copy.keyframes.front().loop_edge_ids,
            (std::vector<std::uint64_t>{9, 11}));
}

TEST(SnapshotValue, UsesWorldCameraPoseConvention) {
  Sophus::SE3f T_camera_world(Eigen::Quaternionf::Identity(),
                              Eigen::Vector3f(1.0F, 2.0F, 3.0F));
  Sophus::SE3f T_world_reference_camera(Eigen::Quaternionf::Identity(),
                                        Eigen::Vector3f(4.0F, 0.0F, 0.0F));
  ORB_SLAM3::FrameSnapshot frame;
  frame.pose_valid = true;
  frame.T_world_camera = T_camera_world.inverse();
  frame.T_reference_camera_current_camera =
      T_world_reference_camera.inverse() * frame.T_world_camera;

  EXPECT_TRUE(frame.pose_valid);
  EXPECT_TRUE(frame.T_world_camera.matrix().isApprox(
      T_camera_world.inverse().matrix()));
  EXPECT_TRUE(frame.T_reference_camera_current_camera.matrix().isApprox(
      (T_world_reference_camera.inverse() * T_camera_world.inverse()).matrix()));
}

TEST(SnapshotValue, RetainsErasedKeyframesForGraphTraversal) {
  ORB_SLAM3::Map map;
  ORB_SLAM3::KeyFrame keyframe;
  keyframe.mnId = 42;

  map.AddKeyFrame(&keyframe);
  map.EraseKeyFrame(&keyframe);

  const auto keyframes = map.GetAllKeyFramesIncludingBad();
  ASSERT_EQ(keyframes.size(), 1U);
  EXPECT_EQ(keyframes.front(), &keyframe);
}

TEST(SnapshotGraphState, AdvancesOnlyForMapOrBigChangeAndResetsConnectivity) {
  ORB_SLAM3::SnapshotGraphState state;

  EXPECT_EQ(state.Observe(7, 0), 1U);
  EXPECT_EQ(state.Observe(7, 0), 1U);
  EXPECT_EQ(state.Observe(7, 1), 2U);
  EXPECT_EQ(state.Observe(8, 0), 3U);
  EXPECT_TRUE(state.IsConnected(7));
  EXPECT_FALSE(state.IsConnected(8));

  state.MarkConnected(8);
  EXPECT_TRUE(state.IsConnected(8));
  state.BeginNewEpoch();
  EXPECT_FALSE(state.IsConnected(7));
  EXPECT_FALSE(state.IsConnected(8));
  EXPECT_EQ(state.Observe(8, 0), 4U);
  EXPECT_TRUE(state.IsConnected(8));
}
