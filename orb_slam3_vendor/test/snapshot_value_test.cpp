#include <gtest/gtest.h>
#include <KeyFrame.h>
#include <KeyFrameDatabase.h>
#include <Map.h>
#include <SystemSnapshots.h>

#include <condition_variable>
#include <algorithm>
#include <mutex>
#include <thread>

namespace {

class SnapshotCullingGate {
 public:
  void OnSnapshotCapture() {
    std::unique_lock<std::mutex> lock(mutex_);
    snapshot_capture_entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this] { return allow_snapshot_capture_; });
  }

  void OnBadKeyframeConnectionsLocked() {
    std::unique_lock<std::mutex> lock(mutex_);
    bad_keyframe_connections_locked_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this] { return allow_bad_keyframe_connections_locked_; });
  }

  void WaitForSnapshotCapture() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return snapshot_capture_entered_; });
  }

  void WaitForBadKeyframeConnectionsLocked() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return bad_keyframe_connections_locked_; });
  }

  void AllowSnapshotCapture() {
    std::lock_guard<std::mutex> lock(mutex_);
    allow_snapshot_capture_ = true;
    condition_.notify_all();
  }

  void AllowBadKeyframeConnectionsLocked() {
    std::lock_guard<std::mutex> lock(mutex_);
    allow_bad_keyframe_connections_locked_ = true;
    condition_.notify_all();
  }

 private:
  std::condition_variable condition_;
  std::mutex mutex_;
  bool snapshot_capture_entered_{false};
  bool bad_keyframe_connections_locked_{false};
  bool allow_snapshot_capture_{false};
  bool allow_bad_keyframe_connections_locked_{false};
};

}  // namespace

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

TEST(SnapshotConcurrency, CapturesGraphWhileCullingKeyframe) {
  ORB_SLAM3::Map map;
  ORB_SLAM3::KeyFrame initial_keyframe;
  ORB_SLAM3::KeyFrame culled_keyframe;
  ORB_SLAM3::KeyFrameDatabase keyframe_database;
  SnapshotCullingGate gate;

  initial_keyframe.mnId = 1;
  culled_keyframe.mnId = 2;
  culled_keyframe.UpdateMap(&map);
  culled_keyframe.SetKeyFrameDatabaseForTesting(&keyframe_database);
  map.AddKeyFrame(&initial_keyframe);
  map.AddKeyFrame(&culled_keyframe);

  ORB_SLAM3::Map::SetSnapshotCaptureTestHook(
      [&gate] { gate.OnSnapshotCapture(); });
  ORB_SLAM3::KeyFrame::SetBadKeyframeConnectionsLockedTestHook(
      [&gate] { gate.OnBadKeyframeConnectionsLocked(); });

  std::thread culling_thread([&culled_keyframe] { culled_keyframe.SetBadFlag(); });
  gate.WaitForBadKeyframeConnectionsLocked();

  std::thread snapshot_thread([&map] { map.GetGraphSnapshotData(); });
  gate.WaitForSnapshotCapture();

  gate.AllowBadKeyframeConnectionsLocked();
  gate.AllowSnapshotCapture();
  snapshot_thread.join();
  culling_thread.join();

  ORB_SLAM3::Map::SetSnapshotCaptureTestHook({});
  ORB_SLAM3::KeyFrame::SetBadKeyframeConnectionsLockedTestHook({});

  EXPECT_TRUE(culled_keyframe.isBad());
  const auto snapshot = map.GetGraphSnapshotData();
  ASSERT_EQ(snapshot.keyframes.size(), 2U);
  const auto culled_snapshot = std::find_if(
      snapshot.keyframes.begin(), snapshot.keyframes.end(),
      [](const ORB_SLAM3::KeyframeSnapshot& keyframe) { return keyframe.id == 2U; });
  ASSERT_NE(culled_snapshot, snapshot.keyframes.end());
  EXPECT_TRUE(culled_snapshot->bad);
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
