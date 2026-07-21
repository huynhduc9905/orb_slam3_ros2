#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/imu_yaw_buffer.hpp"

namespace orb_lidar_mapper {
namespace {

TEST(ImuYawBuffer, IntegratesConstantOmegaWithTrapezoid) {
  // Constant ω=1.0 rad/s over 0.1 s → yaw ≈ 0.1 rad.
  ImuYawBuffer buffer(10'000'000'000LL);
  constexpr std::int64_t start_ns = 0;
  constexpr std::int64_t end_ns = 100'000'000;  // 0.1 s
  constexpr double omega = 1.0;
  // Dense samples well under 20 ms gap.
  for (std::int64_t t = start_ns; t <= end_ns; t += 5'000'000) {
    ASSERT_TRUE(buffer.push({t, omega}));
  }
  ASSERT_TRUE(buffer.covers(start_ns, end_ns));
  const auto yaw = buffer.integratedYaw(start_ns, end_ns);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, 0.1, 1e-12);
}

TEST(ImuYawBuffer, CoversFalseWhenGapExceedsMax) {
  ImuYawBuffer buffer(10'000'000'000LL, 20'000'000LL);
  ASSERT_TRUE(buffer.push({0, 1.0}));
  ASSERT_TRUE(buffer.push({20'000'001, 1.0}));  // gap > 20 ms
  EXPECT_FALSE(buffer.covers(0, 20'000'001));
  EXPECT_FALSE(buffer.integratedYaw(0, 20'000'001).has_value());
}

TEST(ImuYawBuffer, CoversTrueAtExactMaximumGap) {
  ImuYawBuffer buffer(10'000'000'000LL, 20'000'000LL);
  ASSERT_TRUE(buffer.push({0, 1.0}));
  ASSERT_TRUE(buffer.push({20'000'000, 1.0}));
  EXPECT_TRUE(buffer.covers(0, 20'000'000));
  const auto yaw = buffer.integratedYaw(0, 20'000'000);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, 0.02, 1e-12);
}

TEST(ImuYawBuffer, AveragesDuplicateTimestamps) {
  ImuYawBuffer buffer(10'000'000'000LL);
  ASSERT_TRUE(buffer.push({100, 0.2}));
  ASSERT_TRUE(buffer.push({100, 0.4}));
  EXPECT_EQ(buffer.size(), 1U);
  // Average 0.3 at t=100; integrate with constant average over 10 ms (< max gap).
  ASSERT_TRUE(buffer.push({10'000'100, 0.3}));
  const auto yaw = buffer.integratedYaw(100, 10'000'100);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, 0.3 * 0.01, 1e-12);
}

TEST(ImuYawBuffer, AveragesThreeDuplicateTimestamps) {
  ImuYawBuffer buffer(10'000'000'000LL);
  ASSERT_TRUE(buffer.push({50, 1.0}));
  ASSERT_TRUE(buffer.push({50, 2.0}));
  ASSERT_TRUE(buffer.push({50, 3.0}));
  EXPECT_EQ(buffer.size(), 1U);
  // Mean of 1,2,3 = 2.0; hold over 10 ms to next sample.
  ASSERT_TRUE(buffer.push({10'000'050, 2.0}));
  const auto yaw = buffer.integratedYaw(50, 10'000'050);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, 2.0 * 0.01, 1e-9);
}

TEST(ImuYawBuffer, RejectsNonFiniteOmega) {
  ImuYawBuffer buffer(10'000'000'000LL);
  EXPECT_FALSE(buffer.push({0, std::numeric_limits<double>::quiet_NaN()}));
  EXPECT_FALSE(buffer.push({0, std::numeric_limits<double>::infinity()}));
  EXPECT_EQ(buffer.size(), 0U);
}

TEST(ImuYawBuffer, RejectsStrictlyDecreasingStamp) {
  ImuYawBuffer buffer(10'000'000'000LL);
  ASSERT_TRUE(buffer.push({100, 1.0}));
  EXPECT_FALSE(buffer.push({99, 1.0}));
  EXPECT_EQ(buffer.size(), 1U);
}

TEST(ImuYawBuffer, IntegratedYawZeroForEqualEndpoints) {
  ImuYawBuffer buffer(10'000'000'000LL);
  ASSERT_TRUE(buffer.push({0, 5.0}));
  ASSERT_TRUE(buffer.push({10'000'000, 5.0}));
  const auto yaw = buffer.integratedYaw(5'000'000, 5'000'000);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, 0.0, 1e-15);
}

TEST(ImuYawBuffer, IntegratedYawNulloptWhenTargetBeforeStart) {
  ImuYawBuffer buffer(10'000'000'000LL);
  ASSERT_TRUE(buffer.push({0, 1.0}));
  ASSERT_TRUE(buffer.push({10'000'000, 1.0}));
  EXPECT_FALSE(buffer.integratedYaw(10'000'000, 0).has_value());
}

TEST(ImuYawBuffer, RetainsOnlyConfiguredWindow) {
  ImuYawBuffer buffer(100, 100);
  ASSERT_TRUE(buffer.push({0, 1.0}));
  ASSERT_TRUE(buffer.push({100, 1.0}));
  ASSERT_TRUE(buffer.push({101, 1.0}));
  EXPECT_EQ(buffer.size(), 2U);
  EXPECT_FALSE(buffer.covers(0, 101));
}

TEST(ImuYawBuffer, IntegratesVaryingRatesTrapezoidally) {
  // ω: 0 at t=0, 2 at t=0.01 s → average 1 over 0.01 s → yaw = 0.01.
  // Use 10 ms span so default max_gap (20 ms) still covers.
  ImuYawBuffer buffer(10'000'000'000LL);
  ASSERT_TRUE(buffer.push({0, 0.0}));
  ASSERT_TRUE(buffer.push({10'000'000, 2.0}));
  const auto yaw = buffer.integratedYaw(0, 10'000'000);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, 0.01, 1e-12);
}

TEST(ImuYawBuffer, CoversRequiresBracketOfInterval) {
  ImuYawBuffer buffer(10'000'000'000LL);
  ASSERT_TRUE(buffer.push({10'000'000, 1.0}));
  ASSERT_TRUE(buffer.push({20'000'000, 1.0}));
  EXPECT_FALSE(buffer.covers(0, 20'000'000));
  EXPECT_FALSE(buffer.covers(10'000'000, 30'000'000));
  EXPECT_TRUE(buffer.covers(10'000'000, 20'000'000));
}

TEST(ImuYawBuffer, NewestStampEmptyThenTracksLastSample) {
  ImuYawBuffer buffer(10'000'000'000LL);
  EXPECT_FALSE(buffer.newestStamp().has_value());
  ASSERT_TRUE(buffer.push({10'000'000, 1.0}));
  ASSERT_TRUE(buffer.newestStamp().has_value());
  EXPECT_EQ(*buffer.newestStamp(), 10'000'000);
  ASSERT_TRUE(buffer.push({20'000'000, 1.0}));
  EXPECT_EQ(*buffer.newestStamp(), 20'000'000);
}

}  // namespace
}  // namespace orb_lidar_mapper
