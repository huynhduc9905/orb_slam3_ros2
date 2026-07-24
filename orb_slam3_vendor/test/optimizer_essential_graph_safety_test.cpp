#include <gtest/gtest.h>

#include <KeyFrame.h>
#include <LoopClosing.h>
#include <Map.h>
#include <Optimizer.h>

#include <set>
#include <vector>

namespace {

void AddKeyFrame(ORB_SLAM3::Map& map, ORB_SLAM3::KeyFrame& keyframe,
                 unsigned long id) {
  keyframe.mnId = id;
  keyframe.SetPose(Sophus::SE3f());
  keyframe.UpdateMap(&map);
  map.AddKeyFrame(&keyframe);
}

}  // namespace

TEST(OptimizerEssentialGraphSafety, SkipsNullAndNonMemberLoopEndpoints) {
  ORB_SLAM3::Map map;
  ORB_SLAM3::KeyFrame current_keyframe;
  ORB_SLAM3::KeyFrame resident_keyframe;
  ORB_SLAM3::KeyFrame non_member_with_colliding_id;
  ORB_SLAM3::KeyFrame out_of_range_non_member;

  AddKeyFrame(map, current_keyframe, 0);
  AddKeyFrame(map, resident_keyframe, 1);
  non_member_with_colliding_id.mnId = resident_keyframe.mnId;
  non_member_with_colliding_id.SetPose(Sophus::SE3f());
  out_of_range_non_member.mnId = map.GetMaxKFid() + 100;
  out_of_range_non_member.SetPose(Sophus::SE3f());

  ORB_SLAM3::LoopClosing::KeyFrameAndPose non_corrected;
  ORB_SLAM3::LoopClosing::KeyFrameAndPose corrected;
  std::map<ORB_SLAM3::KeyFrame*, std::set<ORB_SLAM3::KeyFrame*>> loop_connections;
  loop_connections[&current_keyframe].insert(nullptr);
  loop_connections[&current_keyframe].insert(&non_member_with_colliding_id);

  // The normal path must not connect the resident vertex to an unrelated KF
  // merely because their numeric IDs collide.
  EXPECT_NO_THROW(ORB_SLAM3::Optimizer::OptimizeEssentialGraph(
      &map, &non_member_with_colliding_id, &current_keyframe, non_corrected,
      corrected, loop_connections, true));

  // 4-DoF used to dereference the null endpoint and had no membership/bounds
  // gate. This is the initialized-inertial implementation's direct regression.
  EXPECT_NO_THROW(ORB_SLAM3::Optimizer::OptimizeEssentialGraph4DoF(
      &map, &non_member_with_colliding_id, &current_keyframe, non_corrected,
      corrected, loop_connections));

  std::map<ORB_SLAM3::KeyFrame*, std::set<ORB_SLAM3::KeyFrame*>> out_of_range_connections;
  out_of_range_connections[&current_keyframe].insert(&out_of_range_non_member);
  // This exact endpoint used to bypass the weight filter as pLoopKF and then
  // index vScw[nIDj] beyond the map's maximum keyframe ID in 4-DoF.
  EXPECT_NO_THROW(ORB_SLAM3::Optimizer::OptimizeEssentialGraph4DoF(
      &map, &out_of_range_non_member, &current_keyframe, non_corrected,
      corrected, out_of_range_connections));
}


TEST(OptimizerEssentialGraphSafety, SizesMergeStateForForeignKeyframeIds) {
  ORB_SLAM3::Map map;
  ORB_SLAM3::KeyFrame current_keyframe;
  ORB_SLAM3::KeyFrame foreign_non_member;

  AddKeyFrame(map, current_keyframe, 0);
  foreign_non_member.mnId = map.GetMaxKFid() + 100;
  foreign_non_member.SetPose(Sophus::SE3f());

  std::vector<ORB_SLAM3::KeyFrame*> fixed_kfs{&current_keyframe};
  std::vector<ORB_SLAM3::KeyFrame*> fixed_corrected_kfs;
  std::vector<ORB_SLAM3::KeyFrame*> non_fixed_kfs{&foreign_non_member};
  std::vector<ORB_SLAM3::MapPoint*> non_corrected_mps;

  // Merge inputs legitimately span maps and therefore can carry a global KF
  // ID beyond current_keyframe's map-local GetMaxKFid(). The optimizer must
  // size its ID-indexed state from all candidates before adding vertices.
  EXPECT_NO_THROW(ORB_SLAM3::Optimizer::OptimizeEssentialGraph(
      &current_keyframe, fixed_kfs, fixed_corrected_kfs, non_fixed_kfs,
      non_corrected_mps));
}
