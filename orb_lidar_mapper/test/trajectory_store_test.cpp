#include <gtest/gtest.h>

#include "orb_lidar_mapper/trajectory_store.hpp"

namespace orb_lidar_mapper {
namespace {

TrajectoryConfig defaultTrajectoryConfig() {
  return {10'000'000'000LL, 200'000'000LL};
}

FrameAnchor makeOkFrame(std::int64_t stamp_ns, Pose2 pose, std::uint64_t map_id,
                        std::uint64_t keyframe_id) {
  return {stamp_ns, TrackingState::kOk, true, map_id, keyframe_id, pose, Pose2{}, Pose2{}};
}

TEST(TrajectoryStore, PlacesScanFromVisualAnchorAndRelativeWheelMotion) {
  TrajectoryStore store(defaultTrajectoryConfig());
  EXPECT_TRUE(store.addWheel({0, Pose2{0, 0, 0}}));
  EXPECT_TRUE(store.addWheel({100'000'000, Pose2{1, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{10, 5, 0}, 7, 3));

  const auto placement = store.placeScan(100'000'000);
  ASSERT_TRUE(placement.has_value());
  EXPECT_TRUE(placement->pose.isApprox(Pose2{11, 5, 0}, 1e-12));
  EXPECT_TRUE(placement->committed);
}

TEST(TrajectoryStore, LossMakesScansProvisionalUntilRecoveryCorrectsInterval) {
  TrajectoryStore store(defaultTrajectoryConfig());
  for (int i = 0; i <= 4; ++i) {
    ASSERT_TRUE(store.addWheel({i * 100'000'000LL, Pose2{0.5 * i, 0, 0}}));
  }
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1));
  ASSERT_TRUE(store.placeScan(0)->committed);
  const auto before_loss = store.snapshot()->graph_revision;

  store.addTrackedFrame({100'000'000, TrackingState::kLost, false, 7, 1,
                         Pose2{}, Pose2{}, Pose2{}});
  const auto provisional = store.placeScan(200'000'000);
  ASSERT_TRUE(provisional.has_value());
  EXPECT_FALSE(provisional->committed);
  EXPECT_EQ(store.snapshot()->graph_revision, before_loss);

  ASSERT_TRUE(store.placeScan(400'000'000).has_value());
  store.addTrackedFrame(makeOkFrame(400'000'000, Pose2{1.8, 0.4, 0.2}, 7, 2));
  const auto revision = store.snapshot();
  ASSERT_EQ(revision->scans.size(), 3U);
  EXPECT_TRUE(revision->scans[0].pose.isApprox(Pose2{}, 1e-12));
  const Pose2 residual = Pose2{2, 0, 0}.inverse() * Pose2{1.8, 0.4, 0.2};
  EXPECT_TRUE(revision->scans[1].pose.isApprox(Pose2{1, 0, 0} * residual.pow(0.5), 1e-12));
  EXPECT_TRUE(revision->scans[1].committed);
  EXPECT_TRUE(revision->scans[2].pose.isApprox(Pose2{1.8, 0.4, 0.2}, 1e-12));
  EXPECT_EQ(store.unresolvedScanCount(), 0U);

  store.addTrackedFrame({500'000'000, TrackingState::kLost, false, 7, 2,
                         Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.addWheel({500'000'000, Pose2{2.5, 0, 0}}));
  EXPECT_FALSE(store.placeScan(500'000'000)->committed);
  EXPECT_EQ(store.unresolvedScanCount(), 1U);
}

TEST(TrajectoryStore, TimestampFallbackCorrectsStationaryWheelInterval) {
  TrajectoryStore store(defaultTrajectoryConfig());
  ASSERT_TRUE(store.addWheel({0, Pose2{}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{}}));
  ASSERT_TRUE(store.addWheel({200, Pose2{}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1));
  store.addTrackedFrame({0, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(100).has_value());
  store.addTrackedFrame(makeOkFrame(200, Pose2{2, 0, 0}, 7, 2));
  const auto revision = store.snapshot();
  ASSERT_EQ(revision->scans.size(), 1U);
  EXPECT_TRUE(revision->scans[0].pose.isApprox(Pose2{1, 0, 0}, 1e-12));
}

TEST(TrajectoryStore, UsesCumulativeWheelDistanceForCorrectionFraction) {
  TrajectoryStore store(defaultTrajectoryConfig());
  ASSERT_TRUE(store.addWheel({0, Pose2{0, 0, 0}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{1, 0, 0}}));
  ASSERT_TRUE(store.addWheel({200, Pose2{1, 1, 0}}));
  ASSERT_TRUE(store.addWheel({300, Pose2{2, 1, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1));
  store.addTrackedFrame({0, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(100).has_value());
  store.addTrackedFrame(makeOkFrame(300, Pose2{3, 1, 0}, 7, 2));

  const auto revision = store.snapshot();
  ASSERT_EQ(revision->scans.size(), 1U);
  EXPECT_TRUE(revision->scans[0].pose.isApprox(Pose2{4.0 / 3.0, 0, 0}, 1e-12));
}

TEST(TrajectoryStore, RejectsMissingWheelPlacementWithoutStateCorruption) {
  TrajectoryStore store(defaultTrajectoryConfig());
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1));
  EXPECT_FALSE(store.placeScan(100).has_value());
  EXPECT_TRUE(store.snapshot()->scans.empty());
  EXPECT_EQ(store.unresolvedScanCount(), 0U);
}

TEST(TrajectoryStore, GraphSnapshotRecomputesFramesAndClosedIntervals) {
  TrajectoryStore store(defaultTrajectoryConfig());
  for (int i = 0; i <= 4; ++i) {
    ASSERT_TRUE(store.addWheel({i * 100LL, Pose2{0.5 * i, 0, 0}}));
  }
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1));
  ASSERT_TRUE(store.placeScan(0).has_value());
  store.addTrackedFrame({100, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(200).has_value());
  store.addTrackedFrame(makeOkFrame(400, Pose2{2, 0, 0}, 7, 3));
  store.addTrackedFrame(makeOkFrame(400, Pose2{9, 9, 0}, 9, 3));

  const GraphSnapshotValue first{10, 7, {{1, 7, Pose2{}}, {3, 7, Pose2{2, 0, 0}}}};
  ASSERT_TRUE(store.applyGraphSnapshot(first));
  const auto first_snapshot = store.snapshot();
  const GraphSnapshotValue moved{11, 7, {{1, 7, Pose2{}}, {3, 7, Pose2{2.3, -0.2, 0.1}}}};
  ASSERT_TRUE(store.applyGraphSnapshot(moved));
  const auto updated = store.snapshot();
  EXPECT_EQ(updated->graph_revision, 11U);
  EXPECT_TRUE(updated->frames[2].map_pose.isApprox(Pose2{2.3, -0.2, 0.1}, 1e-12));
  EXPECT_FALSE(updated->frames[3].pose_valid);
  EXPECT_TRUE(updated->scans[1].committed);
  EXPECT_FALSE(updated->scans[1].pose.isApprox(first_snapshot->scans[1].pose, 1e-12));
  EXPECT_TRUE(first_snapshot->scans[1].pose.isApprox(first_snapshot->scans[1].pose, 1e-12));
  EXPECT_FALSE(store.applyGraphSnapshot(first));
}

}  // namespace
}  // namespace orb_lidar_mapper
