#include <gtest/gtest.h>

#include <LoopRevisitPolicy.h>

using orb_slam3_wrapper_fork::kLoopRevisitMinKFGap;
using orb_slam3_wrapper_fork::shouldExcludeConnectedLoopCandidate;

TEST(LoopRevisitPolicy, ExcludesImmediateNeighbor) {
  // gap 1 -> recent neighbor -> exclude (original behavior)
  EXPECT_TRUE(shouldExcludeConnectedLoopCandidate(101, 100));
}

TEST(LoopRevisitPolicy, ExcludesJustBelowThreshold) {
  // gap 19 (< 20) -> still a neighbor -> exclude
  EXPECT_TRUE(shouldExcludeConnectedLoopCandidate(100, 81));
}

TEST(LoopRevisitPolicy, AllowsAtThreshold) {
  // gap 20 (>= 20) -> revisit -> allow as candidate
  EXPECT_FALSE(shouldExcludeConnectedLoopCandidate(100, 80));
}

TEST(LoopRevisitPolicy, AllowsDistantRevisit) {
  // lap-2 current id vs lap-1 revisited id, gap ~40+ -> allow
  EXPECT_FALSE(shouldExcludeConnectedLoopCandidate(120, 78));
}

TEST(LoopRevisitPolicy, SameIdIsRecentNeighbor) {
  // gap 0 -> exclude
  EXPECT_TRUE(shouldExcludeConnectedLoopCandidate(50, 50));
}

TEST(LoopRevisitPolicy, FutureCandidateIsAllowedNotUnderflow) {
  // candidate_id > current_id must NOT be treated as a huge unsigned gap
  // nor as a recent neighbor; policy returns false (allow), no underflow.
  EXPECT_FALSE(shouldExcludeConnectedLoopCandidate(80, 120));
}
