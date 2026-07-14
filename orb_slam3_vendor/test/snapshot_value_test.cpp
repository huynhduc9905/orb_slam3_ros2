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

TEST(SnapshotValue, RetainsErasedKeyframeValuesAfterKeyframeLifetimeEnds) {
  ORB_SLAM3::Map map;
  auto* keyframe = new ORB_SLAM3::KeyFrame;
  keyframe->mnId = 42;
  keyframe->SetPose(Sophus::SE3f(Eigen::Quaternionf::Identity(),
                                 Eigen::Vector3f(1.0F, 2.0F, 3.0F)));

  map.AddKeyFrame(keyframe);
  map.EraseKeyFrame(keyframe);
  delete keyframe;

  const auto snapshot = map.GetGraphSnapshotData();
  ASSERT_EQ(snapshot.keyframes.size(), 1U);
  EXPECT_EQ(snapshot.keyframes.front().id, 42U);
  EXPECT_TRUE(snapshot.keyframes.front().bad);
  EXPECT_TRUE(snapshot.keyframes.front().T_world_camera.translation().isApprox(
      Eigen::Vector3f(-1.0F, -2.0F, -3.0F)));
}

TEST(SnapshotGraphState, AdvancesForIdentityPreservingMapReplacement) {
  ORB_SLAM3::SnapshotGraphState state;
  ORB_SLAM3::Map original;
  ORB_SLAM3::Map replacement;
  replacement.ChangeId(original.GetId());

  EXPECT_EQ(state.Observe(original, original.GetId(), 0), 1U);
  EXPECT_EQ(state.Observe(original, original.GetId(), 0), 1U);
  EXPECT_EQ(state.Observe(replacement, replacement.GetId(), 0), 2U);
}

TEST(SnapshotGraphState, KeepsResetCreatedBranchDisconnectedUntilMerged) {
  ORB_SLAM3::SnapshotGraphState state;
  ORB_SLAM3::Map root;
  ORB_SLAM3::Map reset_branch;

  state.SetInitialRootMap(root);
  EXPECT_EQ(state.Observe(root, root.GetId(), 0), 1U);
  EXPECT_TRUE(state.IsConnected(root));
  state.BeginNewEpoch(ORB_SLAM3::SnapshotGraphState::EpochKind::ACTIVE_MAP_RESET);
  EXPECT_EQ(state.Observe(reset_branch, reset_branch.GetId(), 0), 2U);
  EXPECT_FALSE(state.IsConnected(reset_branch));
  state.MarkConnected(reset_branch);
  EXPECT_TRUE(state.IsConnected(reset_branch));
}

TEST(SnapshotGraphState, KeepsAutomaticNewMapDisconnectedBeforeFirstSnapshot) {
  ORB_SLAM3::SnapshotGraphState state;
  ORB_SLAM3::Map original_root;
  ORB_SLAM3::Map automatic_branch;

  state.SetInitialRootMap(original_root);
  EXPECT_EQ(state.Observe(automatic_branch, automatic_branch.GetId(), 0), 1U);
  EXPECT_FALSE(state.IsConnected(automatic_branch));
}
