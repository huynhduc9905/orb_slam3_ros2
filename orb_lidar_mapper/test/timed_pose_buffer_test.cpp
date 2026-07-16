#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/timed_pose_buffer.hpp"

namespace orb_lidar_mapper {
namespace {

TEST(TimedPoseBuffer, ReturnsExactSample) {
  TimedPoseBuffer buffer(10'000'000'000LL, 100'000'000LL);
  const Pose2 pose{1.0, 2.0, 0.3};
  EXPECT_TRUE(buffer.push({42, pose}));
  const auto result = buffer.interpolate(42);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->isApprox(pose, 1e-12));
}

TEST(TimedPoseBuffer, InterpolatesBracketedSamples) {
  TimedPoseBuffer buffer(10'000'000'000LL, 100'000'000LL);
  EXPECT_TRUE(buffer.push({0, Pose2{0.0, 0.0, 0.0}}));
  EXPECT_TRUE(buffer.push({100'000'000, Pose2{2.0, 0.0, 0.0}}));
  const auto result = buffer.interpolate(50'000'000);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->isApprox(Pose2{1.0, 0.0, 0.0}, 1e-12));
}

TEST(TimedPoseBuffer, RejectsOutOfOrderSample) {
  TimedPoseBuffer buffer(10'000'000'000LL, 100'000'000LL);
  EXPECT_TRUE(buffer.push({100, Pose2{}}));
  EXPECT_FALSE(buffer.push({99, Pose2{}}));
  EXPECT_EQ(buffer.size(), 1U);
}

TEST(TimedPoseBuffer, ReplacesNewestSampleWithEqualTimestamp) {
  TimedPoseBuffer buffer(100, 100);
  EXPECT_TRUE(buffer.push({42, Pose2{1.0, 0.0, 0.0}}));
  EXPECT_TRUE(buffer.push({42, Pose2{2.0, 0.0, 0.0}}));
  EXPECT_EQ(buffer.size(), 1U);
  const auto result = buffer.interpolate(42);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->isApprox(Pose2{2.0, 0.0, 0.0}, 1e-12));
}

TEST(TimedPoseBuffer, RejectsNegativeConfiguration) {
  EXPECT_THROW(TimedPoseBuffer(-1, 0), std::invalid_argument);
  EXPECT_THROW(TimedPoseBuffer(0, -1), std::invalid_argument);
}

TEST(TimedPoseBuffer, RetainsOnlyConfiguredWindow) {
  TimedPoseBuffer buffer(100, 100);
  EXPECT_TRUE(buffer.push({0, Pose2{}}));
  EXPECT_TRUE(buffer.push({100, Pose2{1.0, 0.0, 0.0}}));
  EXPECT_TRUE(buffer.push({101, Pose2{2.0, 0.0, 0.0}}));
  EXPECT_EQ(buffer.size(), 2U);
  EXPECT_FALSE(buffer.interpolate(0).has_value());
}

TEST(TimedPoseBuffer, RejectsInterpolationAcrossStaleGap) {
  TimedPoseBuffer buffer(10'000'000'000LL, 100'000'000LL);
  buffer.push({0, Pose2{0, 0, 0}});
  buffer.push({200'000'000, Pose2{1, 0, 0}});
  EXPECT_FALSE(buffer.interpolate(100'000'000).has_value());
}

TEST(TimedPoseBuffer, AcceptsInterpolationAtExactMaximumGap) {
  TimedPoseBuffer buffer(100, 100);
  EXPECT_TRUE(buffer.push({0, Pose2{0.0, 0.0, 0.0}}));
  EXPECT_TRUE(buffer.push({100, Pose2{2.0, 0.0, 0.0}}));
  const auto result = buffer.interpolate(50);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->isApprox(Pose2{1.0, 0.0, 0.0}, 1e-12));
}

TEST(TimedPoseBuffer, AcceptsEndpointsAndRejectsOutOfRangeTimestamps) {
  TimedPoseBuffer buffer(100, 100);
  EXPECT_TRUE(buffer.push({10, Pose2{1.0, 0.0, 0.0}}));
  EXPECT_TRUE(buffer.push({20, Pose2{2.0, 0.0, 0.0}}));
  EXPECT_TRUE(buffer.interpolate(10).has_value());
  EXPECT_TRUE(buffer.interpolate(20).has_value());
  EXPECT_FALSE(buffer.interpolate(9).has_value());
  EXPECT_FALSE(buffer.interpolate(21).has_value());
}

TEST(TimedPoseBuffer, RetentionHandlesFullInt64TimestampRange) {
  TimedPoseBuffer buffer(std::numeric_limits<std::int64_t>::max(),
                         std::numeric_limits<std::int64_t>::max());
  EXPECT_TRUE(buffer.push({std::numeric_limits<std::int64_t>::min(), Pose2{}}));
  EXPECT_TRUE(buffer.push({std::numeric_limits<std::int64_t>::max(), Pose2{}}));
  EXPECT_EQ(buffer.size(), 1U);
}

TEST(TimedPoseBuffer, RejectsInterpolationAcrossFullInt64TimestampRange) {
  TimedPoseBuffer buffer(std::numeric_limits<std::int64_t>::max(),
                         std::numeric_limits<std::int64_t>::max());
  EXPECT_TRUE(buffer.push({std::numeric_limits<std::int64_t>::min(), Pose2{}}));
  EXPECT_TRUE(buffer.push({std::numeric_limits<std::int64_t>::max(), Pose2{}}));
  EXPECT_FALSE(buffer.interpolate(0).has_value());
}

TEST(TimedPoseBuffer, RelativeReturnsTransformFromFirstTimeToSecond) {
  TimedPoseBuffer buffer(10'000'000'000LL, 100'000'000LL);
  EXPECT_TRUE(buffer.push({0, Pose2{1.0, 0.0, M_PI_2}}));
  EXPECT_TRUE(buffer.push({100, Pose2{1.0, 2.0, M_PI_2}}));
  const auto result = buffer.relative(0, 100);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->isApprox(Pose2{2.0, 0.0, 0.0}, 1e-12));
}

TEST(TimedPoseBuffer, MeasuresPeakToPeakYawIncludingTurnThenReturn) {
  TimedPoseBuffer buffer(10'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(buffer.push({0, Pose2{0, 0, 0.0}}));
  ASSERT_TRUE(buffer.push({50'000'000, Pose2{0.05, 0, 0.01}}));
  ASSERT_TRUE(buffer.push({100'000'000, Pose2{0.10, 0, 0.0}}));
  const auto excursion = buffer.maximumYawExcursion(0, 100'000'000);
  ASSERT_TRUE(excursion);
  EXPECT_NEAR(*excursion, 0.01, 1e-12);
}

TEST(TimedPoseBuffer, UnwrapsYawAcrossPiForExcursion) {
  TimedPoseBuffer buffer(10'000'000'000LL, 100'000'000LL);
  ASSERT_TRUE(buffer.push({0, Pose2{0, 0, M_PI - 0.004}}));
  ASSERT_TRUE(buffer.push({100'000'000, Pose2{0, 0, -M_PI + 0.004}}));
  const auto excursion = buffer.maximumYawExcursion(0, 100'000'000);
  ASSERT_TRUE(excursion);
  EXPECT_NEAR(*excursion, 0.008, 1e-12);
}

}  // namespace
}  // namespace orb_lidar_mapper
