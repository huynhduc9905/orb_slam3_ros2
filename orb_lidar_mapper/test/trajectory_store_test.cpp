#include <gtest/gtest.h>

#include <limits>

#include "orb_lidar_mapper/trajectory_store.hpp"

namespace orb_lidar_mapper {
namespace {

TrajectoryConfig defaultTrajectoryConfig() {
  return {10'000'000'000LL, 200'000'000LL};
}

FrameAnchor makeOkFrame(std::int64_t stamp_ns, Pose2 pose, std::uint64_t map_id,
                        std::uint64_t keyframe_id, Pose2 wheel_pose = Pose2{}) {
  return {stamp_ns, TrackingState::kOk, true, map_id, keyframe_id, pose, Pose2{}, wheel_pose};
}

TEST(TrajectoryStore, PlacesScanFromVisualAnchorAndRelativeWheelMotion) {
  TrajectoryStore store(defaultTrajectoryConfig());
  EXPECT_TRUE(store.addWheel({100'000'000, Pose2{1, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{10, 5, 0}, 7, 3, Pose2{0, 0, 0}));

  const auto placement = store.placeScan(100'000'000);
  ASSERT_TRUE(placement.has_value());
  EXPECT_TRUE(placement->pose.isApprox(Pose2{11, 5, 0}, 1e-12));
  EXPECT_TRUE(placement->committed);
}

TEST(TrajectoryStore, RecomputesClosedProvisionalScansWhenLateWheelEndpointArrives) {
  TrajectoryStore store(defaultTrajectoryConfig());
  ASSERT_TRUE(store.addWheel({0, Pose2{0, 0, 0}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{1, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1, Pose2{}));
  store.addTrackedFrame({0, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(100).has_value());
  store.addTrackedFrame(makeOkFrame(200, Pose2{1.8, 0, 0}, 7, 2, Pose2{2, 0, 0}));
  ASSERT_EQ(store.unresolvedScanCount(), 1U);

  ASSERT_TRUE(store.addWheel({200, Pose2{2, 0, 0}}));
  const auto revision = store.snapshot();
  ASSERT_EQ(revision->scans.size(), 1U);
  EXPECT_TRUE(revision->scans[0].committed);
  EXPECT_TRUE(revision->scans[0].pose.isApprox(Pose2{0.9, 0, 0}, 1e-12));
}

TEST(TrajectoryStore, RecomputesClosedIntervalWhenNewestWheelEndpointIsReplaced) {
  TrajectoryStore store(defaultTrajectoryConfig());
  ASSERT_TRUE(store.addWheel({0, Pose2{0, 0, 0}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{1, 0, 0}}));
  ASSERT_TRUE(store.addWheel({200, Pose2{2, 0, 0}}));
  ASSERT_TRUE(store.addWheel({300, Pose2{3, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1, Pose2{}));
  store.addTrackedFrame({0, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(250).has_value());
  store.addTrackedFrame(makeOkFrame(300, Pose2{2.7, 0, 0}, 7, 2, Pose2{3, 0, 0}));
  ASSERT_TRUE(store.snapshot()->scans[0].pose.isApprox(Pose2{2.25, 0, 0}, 1e-12));

  ASSERT_TRUE(store.addWheel({300, Pose2{4, 0, 0}}));

  const auto revision = store.snapshot();
  ASSERT_TRUE(revision->scans[0].committed);
  EXPECT_TRUE(revision->scans[0].pose.isApprox(Pose2{2.7, 0, 0}, 1e-12));
}

TEST(TrajectoryStore, BoundsWheelStateDuringAnUnresolvedLossInterval) {
  TrajectoryStore store({10, 1});
  ASSERT_TRUE(store.addWheel({0, Pose2{}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1, Pose2{}));
  store.addTrackedFrame({0, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});

  for (std::int64_t stamp = 1; stamp <= 1'000; ++stamp) {
    ASSERT_TRUE(store.addWheel({stamp, Pose2{static_cast<double>(stamp), 0, 0}}));
  }

  EXPECT_LE(store.wheelStateCount(), 12U);
}

TEST(TrajectoryStore, ClosesStationaryIntervalsAcrossFullTimestampDomainWithoutOverflow) {
  TrajectoryStore store({std::numeric_limits<std::int64_t>::max(),
                         std::numeric_limits<std::int64_t>::max()});
  constexpr auto kMin = std::numeric_limits<std::int64_t>::min();
  constexpr auto kMax = std::numeric_limits<std::int64_t>::max();
  ASSERT_TRUE(store.addWheel({kMin, Pose2{}}));
  ASSERT_TRUE(store.addWheel({kMax, Pose2{}}));
  store.addTrackedFrame(makeOkFrame(kMin, Pose2{}, 7, 1, Pose2{}));
  store.addTrackedFrame({kMin, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(kMax).has_value());
  store.addTrackedFrame(makeOkFrame(kMax, Pose2{2, 0, 0}, 7, 2, Pose2{}));
  ASSERT_TRUE(store.snapshot()->scans[0].committed);
  EXPECT_TRUE(store.snapshot()->scans[0].pose.isApprox(Pose2{2, 0, 0}, 1e-12));
}

TEST(TrajectoryStore, KeepsReversedFallbackIntervalsProvisional) {
  TrajectoryStore store(defaultTrajectoryConfig());
  ASSERT_TRUE(store.addWheel({0, Pose2{}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{}}));
  store.addTrackedFrame(makeOkFrame(100, Pose2{}, 7, 1, Pose2{}));
  store.addTrackedFrame({100, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(100).has_value());
  store.addTrackedFrame(makeOkFrame(0, Pose2{2, 0, 0}, 7, 2, Pose2{}));
  EXPECT_FALSE(store.snapshot()->scans[0].committed);
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
  store.addTrackedFrame(makeOkFrame(400'000'000, Pose2{1.8, 0.4, 0.2}, 7, 2, Pose2{2, 0, 0}));
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
  store.addTrackedFrame(makeOkFrame(200, Pose2{2, 0, 0}, 7, 2, Pose2{}));
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
  store.addTrackedFrame(makeOkFrame(300, Pose2{3, 1, 0}, 7, 2, Pose2{2, 1, 0}));

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
  store.addTrackedFrame(makeOkFrame(400, Pose2{2, 0, 0}, 7, 3, Pose2{2, 0, 0}));
  store.addTrackedFrame(makeOkFrame(400, Pose2{9, 9, 0}, 9, 3, Pose2{2, 0, 0}));

  const GraphSnapshotValue first{10, 7, true, {{1, 7, Pose2{}}, {3, 7, Pose2{2, 0, 0}}}};
  ASSERT_TRUE(store.applyGraphSnapshot(first));
  const auto first_snapshot = store.snapshot();
  const GraphSnapshotValue moved{11, 7, true, {{1, 7, Pose2{}}, {3, 7, Pose2{2.3, -0.2, 0.1}}}};
  ASSERT_TRUE(store.applyGraphSnapshot(moved));
  const auto updated = store.snapshot();
  EXPECT_EQ(updated->graph_revision, 11U);
  EXPECT_TRUE(updated->frames[2].map_pose.isApprox(Pose2{2.3, -0.2, 0.1}, 1e-12));
  EXPECT_FALSE(updated->frames[3].pose_valid);
  EXPECT_TRUE(updated->scans[1].committed);
  EXPECT_FALSE(updated->scans[1].pose.isApprox(first_snapshot->scans[1].pose, 1e-12));
  EXPECT_FALSE(first_snapshot->scans[1].pose.isApprox(updated->scans[1].pose, 1e-12));
  EXPECT_FALSE(store.applyGraphSnapshot(first));
}

TEST(TrajectoryStore, DisconnectedGraphSnapshotMakesEveryDependentScanProvisionalUntilRestored) {
  TrajectoryStore store(defaultTrajectoryConfig());
  ASSERT_TRUE(store.addWheel({0, Pose2{}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{1, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 3, Pose2{}));
  ASSERT_TRUE(store.placeScan(100)->committed);

  ASSERT_TRUE(store.applyGraphSnapshot({10, 7, false, {{3, 7, Pose2{}}}}));
  EXPECT_FALSE(store.snapshot()->frames[0].pose_valid);
  EXPECT_FALSE(store.snapshot()->scans[0].committed);

  ASSERT_TRUE(store.applyGraphSnapshot({11, 7, true, {{3, 7, Pose2{2, 0, 0}}}}));
  EXPECT_TRUE(store.snapshot()->frames[0].pose_valid);
  EXPECT_TRUE(store.snapshot()->scans[0].committed);
  EXPECT_TRUE(store.snapshot()->scans[0].pose.isApprox(Pose2{3, 0, 0}, 1e-12));
}

TEST(TrajectoryStore, GraphRevisionRecomputesEveryScanInMultipleClosedIntervals) {
  TrajectoryStore store(defaultTrajectoryConfig());
  for (int i = 0; i <= 6; ++i) {
    ASSERT_TRUE(store.addWheel({i * 100LL, Pose2{static_cast<double>(i), 0, 0}}));
  }
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1, Pose2{}));
  store.addTrackedFrame({0, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.placeScan(100).has_value());
  store.addTrackedFrame(makeOkFrame(200, Pose2{2, 0, 0}, 7, 3, Pose2{2, 0, 0}));
  ASSERT_TRUE(store.placeScan(200).has_value());
  store.addTrackedFrame({300, TrackingState::kLost, false, 7, 3, Pose2{}, Pose2{}, Pose2{3, 0, 0}});
  ASSERT_TRUE(store.placeScan(400).has_value());
  store.addTrackedFrame(makeOkFrame(500, Pose2{5, 0, 0}, 7, 3, Pose2{5, 0, 0}));
  ASSERT_TRUE(store.placeScan(500).has_value());

  ASSERT_TRUE(store.applyGraphSnapshot({10, 7, true,
    {{1, 7, Pose2{}}, {3, 7, Pose2{2, 0, 0}}}}));
  const auto snapshot_a = store.snapshot();
  ASSERT_TRUE(store.applyGraphSnapshot({11, 7, true,
    {{1, 7, Pose2{}}, {3, 7, Pose2{2.3, -0.2, 0.1}}}}));
  const auto snapshot_b = store.snapshot();

  ASSERT_EQ(snapshot_a->scans.size(), snapshot_b->scans.size());
  for (std::size_t index = 0; index < snapshot_b->scans.size(); ++index) {
    EXPECT_TRUE(snapshot_b->scans[index].committed);
    EXPECT_EQ(snapshot_b->scans[index].graph_revision, 11U);
  }
  EXPECT_FALSE(snapshot_a->scans[1].pose.isApprox(snapshot_b->scans[1].pose, 1e-12));
  EXPECT_FALSE(snapshot_a->scans[2].pose.isApprox(snapshot_b->scans[2].pose, 1e-12));
  EXPECT_FALSE(snapshot_a->scans[3].pose.isApprox(snapshot_b->scans[3].pose, 1e-12));
}

TEST(TrajectoryStore, DuplicateWheelTimestampReplacesSampleDeterministically) {
  TrajectoryStore store({0, 100});
  ASSERT_TRUE(store.addWheel({0, Pose2{}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{1, 0, 0}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{2, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1, Pose2{}));
  const auto placement = store.placeScan(100);
  ASSERT_TRUE(placement.has_value());
  EXPECT_TRUE(placement->pose.isApprox(Pose2{2, 0, 0}, 1e-12));
}

TEST(TrajectoryStore, ReplacesPrunedWheelEndpointForArchivedClosedInterval) {
  TrajectoryStore store({0, 100});
  ASSERT_TRUE(store.addWheel({0, Pose2{0, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(0, Pose2{}, 7, 1, Pose2{}));
  store.addTrackedFrame({0, TrackingState::kLost, false, 7, 1, Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.addWheel({50, Pose2{1, 0, 0}}));
  ASSERT_TRUE(store.placeScan(50).has_value());
  ASSERT_TRUE(store.addWheel({100, Pose2{2, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(100, Pose2{3, 0, 0}, 7, 2, Pose2{2, 0, 0}));
  ASSERT_TRUE(store.snapshot()->scans[0].pose.isApprox(Pose2{1.5, 0, 0}, 1e-12));

  ASSERT_TRUE(store.addWheel({100, Pose2{3, 0, 0}}));

  const auto revision = store.snapshot();
  ASSERT_EQ(revision->scans.size(), 1U);
  EXPECT_TRUE(revision->scans[0].committed);
  EXPECT_TRUE(revision->scans[0].pose.isApprox(Pose2{4.0 / 3.0, 0, 0}, 1e-12));
  EXPECT_EQ(store.wheelStateCount(), 2U);
}

TEST(TrajectoryStore, AdvancesPrunedWheelPredecessorAfterEachNewEndpoint) {
  TrajectoryStore store({0, 100});
  ASSERT_TRUE(store.addWheel({0, Pose2{}}));
  ASSERT_TRUE(store.addWheel({50, Pose2{1, 0, 0}}));
  ASSERT_TRUE(store.addWheel({100, Pose2{3, 0, 0}}));
  store.addTrackedFrame(makeOkFrame(100, Pose2{3, 0, 0}, 7, 1, Pose2{3, 0, 0}));
  store.addTrackedFrame({100, TrackingState::kLost, false, 7, 1,
                         Pose2{}, Pose2{}, Pose2{}});
  ASSERT_TRUE(store.addWheel({125, Pose2{3, 1, 0}}));
  ASSERT_TRUE(store.placeScan(125).has_value());
  ASSERT_TRUE(store.addWheel({150, Pose2{3, 2, 0}}));
  store.addTrackedFrame(makeOkFrame(150, Pose2{7, 2, 0}, 7, 2, Pose2{3, 2, 0}));

  ASSERT_TRUE(store.addWheel({150, Pose2{3, 4, 0}}));

  const auto revision = store.snapshot();
  ASSERT_EQ(revision->scans.size(), 1U);
  EXPECT_TRUE(revision->scans[0].committed);
  EXPECT_TRUE(revision->scans[0].pose.isApprox(Pose2{4, 1, 0}, 1e-12));
  EXPECT_EQ(store.wheelStateCount(), 2U);
}

}  // namespace
}  // namespace orb_lidar_mapper
