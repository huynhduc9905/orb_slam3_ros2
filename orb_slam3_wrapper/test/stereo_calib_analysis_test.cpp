#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "orb_slam3_wrapper/mount_xy_mapper.hpp"
#include "orb_slam3_wrapper/planar_pose_projector.hpp"
#include "orb_slam3_wrapper/stereo_calib_analysis.hpp"
#include "orb_slam3_wrapper/stereo_rotation_center.hpp"

namespace orb_slam3_wrapper {
namespace {

Pose2 about(double yaw, Point2 center) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  Pose2 m;
  m.yaw = yaw;
  m.x = center.x - (c * center.x - s * center.y);
  m.y = center.y - (s * center.x + c * center.y);
  return m;
}

PlanarPose makePlanar(int64_t stamp_ns, double x, double y, double yaw) {
  PlanarPose p;
  p.stamp_ns = stamp_ns;
  p.pose = {x, y, yaw};
  p.height_m = 0.0;
  p.valid = true;
  return p;
}

// Pure spin of camera about world origin with fixed body offset p.
// pose = T_world_camera: origin at R(θ)*p, yaw = θ.
std::vector<PlanarPose> pureSpinTrajectory(Point2 optical_in_base, int n,
                                           double dtheta) {
  std::vector<PlanarPose> poses;
  poses.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double th = i * dtheta;
    const double c = std::cos(th), s = std::sin(th);
    const double x = c * optical_in_base.x - s * optical_in_base.y;
    const double y = s * optical_in_base.x + c * optical_in_base.y;
    poses.push_back(makePlanar(static_cast<int64_t>(i) * 1'000'000'000LL, x, y, th));
  }
  return poses;
}

StereoCenterSample acceptedSample(std::size_t sector, MountXy xy) {
  StereoCenterSample s;
  s.yaw_sector = sector;
  s.accepted = true;
  s.mount_xy = xy;
  s.center = {xy.x_m, xy.y_m};
  return s;
}

bool hasReason(const StereoEstimate& est, const std::string& reason) {
  return std::find(est.unreliable_reasons.begin(), est.unreliable_reasons.end(),
                   reason) != est.unreliable_reasons.end();
}

TEST(StereoCalibAnalysis, SectorForYawEightEqualBins) {
  // 8 equal sectors over [0, 2π): sector k covers [k*π/4, (k+1)*π/4).
  for (std::size_t k = 0; k < 8; ++k) {
    const double lo = static_cast<double>(k) * kPi / 4.0;
    const double mid = lo + kPi / 8.0;
    const double near_hi = lo + kPi / 4.0 - 1e-9;
    EXPECT_EQ(sectorForYaw(lo), k) << "lo sector " << k;
    EXPECT_EQ(sectorForYaw(mid), k) << "mid sector " << k;
    EXPECT_EQ(sectorForYaw(near_hi), k) << "near_hi sector " << k;
  }
  // Wrap: 2π and negative angles.
  EXPECT_EQ(sectorForYaw(0.0), 0u);
  EXPECT_EQ(sectorForYaw(2.0 * kPi), 0u);
  EXPECT_EQ(sectorForYaw(-kPi / 8.0), 7u);  // near 2π from below
  EXPECT_EQ(sectorForYaw(-kPi), 4u);        // -π ≡ π → sector 4
  EXPECT_EQ(sectorForYaw(kPi), 4u);
  EXPECT_EQ(sectorForYaw(3.0 * kPi / 2.0), 6u);
}

TEST(StereoCalibAnalysis, RelativePoseFromPureSpinRecoversLeverArm) {
  // Camera optical origin at body offset p; pure rotation about base origin.
  // Lever arm c (optical → center) in horizontal-left = -p when axes align.
  const Point2 p{0.32, 0.05};
  const Point2 expected_c{-0.32, -0.05};

  // Direct about() lock (Task 1 convention).
  const auto direct = centerFromTransform(about(0.40, expected_c));
  ASSERT_TRUE(direct.has_value());
  EXPECT_NEAR(direct->x, expected_c.x, 1e-9);
  EXPECT_NEAR(direct->y, expected_c.y, 1e-9);

  // Trajectory: θ_i with Δθ ≈ 0.20 rad (~11.5°) between neighbors → in band.
  const auto poses = pureSpinTrajectory(p, 32, 0.20);
  const auto pairs =
      selectPosePairs(poses, /*intervals=*/{}, 10.0 * kPi / 180.0,
                      30.0 * kPi / 180.0);
  ASSERT_FALSE(pairs.empty());

  std::size_t ok = 0;
  for (const auto& [src, tgt] : pairs) {
    ASSERT_LT(src, poses.size());
    ASSERT_LT(tgt, poses.size());
    // Locked convention: source_to_target = target.inverse() * source
    const Pose2 source_to_target =
        poses[tgt].pose.inverse().compose(poses[src].pose);
    const auto center = centerFromTransform(source_to_target);
    ASSERT_TRUE(center.has_value()) << "pair " << src << "->" << tgt;
    EXPECT_NEAR(center->x, expected_c.x, 1e-6);
    EXPECT_NEAR(center->y, expected_c.y, 1e-6);
    ++ok;
  }
  EXPECT_GE(ok, 10u);
}

TEST(StereoCalibAnalysis, SelectPosePairsRespectsYawBandAndIntervals) {
  const Point2 p{0.26, 0.0};
  // 0.04 rad steps; pairs spanning 3–7 steps ≈ 0.12–0.28 rad ≈ 7–16° —
  // only some in [10°,30°].
  auto poses = pureSpinTrajectory(p, 17, 0.04);
  const auto all =
      selectPosePairs(poses, {}, 10.0 * kPi / 180.0, 30.0 * kPi / 180.0);
  ASSERT_FALSE(all.empty());
  for (const auto& [src, tgt] : all) {
    EXPECT_LT(src, tgt);
    const double dyaw =
        std::abs(Pose2::normalizeAngle(poses[tgt].pose.yaw - poses[src].pose.yaw));
    EXPECT_GE(dyaw, 10.0 * kPi / 180.0 - 1e-12);
    EXPECT_LE(dyaw, 30.0 * kPi / 180.0 + 1e-12);
  }

  // Interval covering only first half of stamps.
  const std::vector<MotionInterval> half{
      {0, 8'000'000'000LL},
  };
  const auto restricted =
      selectPosePairs(poses, half, 10.0 * kPi / 180.0, 30.0 * kPi / 180.0);
  for (const auto& [src, tgt] : restricted) {
    EXPECT_LE(poses[src].stamp_ns, 8'000'000'000LL);
    EXPECT_LE(poses[tgt].stamp_ns, 8'000'000'000LL);
  }
  EXPECT_LT(restricted.size(), all.size());
}

TEST(StereoCalibAnalysis, SelectPosePairsEmptyIntervalRejectsAll) {
  const auto poses = pureSpinTrajectory({0.26, 0.0}, 10, 0.20);
  const std::vector<MotionInterval> none{
      {100'000'000'000LL, 200'000'000'000LL},
  };
  EXPECT_TRUE(
      selectPosePairs(poses, none, 10.0 * kPi / 180.0, 30.0 * kPi / 180.0)
          .empty());
}

TEST(StereoCalibAnalysis, RobustEstimateReliableOnTightCluster) {
  std::vector<StereoCenterSample> samples;
  // 48 samples across 8 sectors, tight cluster around (0.30, 0.04)
  for (std::size_t i = 0; i < 48; ++i) {
    const double noise = 0.0001 * static_cast<double>(static_cast<int>(i % 5) - 2);
    samples.push_back(acceptedSample(i % 8, {0.30 + noise, 0.04 - noise}));
  }
  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;
  thr.max_ci_half_width_m = 0.015;

  const auto est = robustEstimate(samples, /*seed=*/42, thr);
  EXPECT_TRUE(est.reliable) << [&] {
    std::string s;
    for (const auto& r : est.unreliable_reasons) s += r + ";";
    return s;
  }();
  EXPECT_EQ(est.accepted_pairs, 48u);
  EXPECT_EQ(est.sectors_used, 8u);
  EXPECT_NEAR(est.median_xy.x_m, 0.30, 0.002);
  EXPECT_NEAR(est.median_xy.y_m, 0.04, 0.002);
  EXPECT_LE(est.ci_half_width.x_m, thr.max_ci_half_width_m);
  EXPECT_LE(est.ci_half_width.y_m, thr.max_ci_half_width_m);
}

TEST(StereoCalibAnalysis, RobustEstimateUnreliableWithFewPairs) {
  std::vector<StereoCenterSample> samples;
  for (std::size_t i = 0; i < 10; ++i) {
    samples.push_back(acceptedSample(i % 8, {0.30, 0.04}));
  }
  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;

  const auto est = robustEstimate(samples, 7, thr);
  EXPECT_FALSE(est.reliable);
  EXPECT_EQ(est.accepted_pairs, 10u);
  ASSERT_FALSE(est.unreliable_reasons.empty());
  EXPECT_TRUE(hasReason(est, "insufficient_accepted_pairs"));
}

TEST(StereoCalibAnalysis, RobustEstimateUnreliableWithFewSectors) {
  std::vector<StereoCenterSample> samples;
  for (std::size_t i = 0; i < 50; ++i) {
    samples.push_back(acceptedSample(/*sector=*/0, {0.30, 0.04}));
  }
  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;

  const auto est = robustEstimate(samples, 9, thr);
  EXPECT_FALSE(est.reliable);
  EXPECT_EQ(est.sectors_used, 1u);
  EXPECT_TRUE(hasReason(est, "insufficient_yaw_sectors"));
}

TEST(StereoCalibAnalysis, RobustEstimateUnreliableWithWideCi) {
  // Many pairs / sectors but mount_xy spread so bootstrap CI half-width is large.
  std::vector<StereoCenterSample> samples;
  for (std::size_t i = 0; i < 48; ++i) {
    const double x = (i % 2 == 0) ? 0.10 : 0.50;
    const double y = (i % 2 == 0) ? 0.00 : 0.40;
    samples.push_back(acceptedSample(i % 8, {x, y}));
  }
  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;
  thr.max_ci_half_width_m = 0.015;
  thr.max_abs_center_m = 1.0;

  const auto est = robustEstimate(samples, 11, thr);
  EXPECT_FALSE(est.reliable);
  EXPECT_TRUE(hasReason(est, "confidence_interval_x_too_wide") ||
              hasReason(est, "confidence_interval_y_too_wide"));
}

TEST(StereoCalibAnalysis, RobustEstimateUnreliableWithMaxAbsCenter) {
  std::vector<StereoCenterSample> samples;
  for (std::size_t i = 0; i < 48; ++i) {
    const double noise = 0.0001 * static_cast<double>(static_cast<int>(i % 5) - 2);
    samples.push_back(acceptedSample(i % 8, {1.50 + noise, 0.04 - noise}));
  }
  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;
  thr.max_ci_half_width_m = 0.015;
  thr.max_abs_center_m = 1.0;

  const auto est = robustEstimate(samples, 13, thr);
  EXPECT_FALSE(est.reliable);
  EXPECT_TRUE(hasReason(est, "median_out_of_bounds"));
}

TEST(StereoCalibAnalysis, PureSpinPipelineRecoversKnownMountIdentityChain) {
  // Full chain: pure spin → selectPosePairs → centerFromTransform →
  // impliedCameraLinkXy (identity optical chain) → robustEstimate → classify.
  const Point2 optical_in_base{0.32, 0.05};
  const Point2 expected_c{-0.32, -0.05};
  const MountXy recorded_xy{0.32, 0.05};

  StaticCameraMount identity_mount;
  identity_mount.T_base_camera_link.setIdentity();
  identity_mount.T_camera_link_left_optical.setIdentity();

  // Dense pure spin: Δθ = 0.20 rad between neighbors → pairs in [10°, 30°].
  const auto poses = pureSpinTrajectory(optical_in_base, 64, 0.20);
  StereoThresholds thr;
  thr.min_accepted_pairs = 40;
  thr.min_sectors = 6;
  thr.max_ci_half_width_m = 0.015;
  thr.agreement_floor_m = 0.010;
  thr.max_abs_center_m = 1.0;
  thr.min_pair_yaw_rad = 10.0 * kPi / 180.0;
  thr.max_pair_yaw_rad = 30.0 * kPi / 180.0;

  const auto pairs =
      selectPosePairs(poses, /*intervals=*/{}, thr.min_pair_yaw_rad,
                      thr.max_pair_yaw_rad);
  ASSERT_GE(pairs.size(), thr.min_accepted_pairs);

  std::vector<StereoCenterSample> samples;
  samples.reserve(pairs.size());
  for (const auto& [src, tgt] : pairs) {
    const Pose2 source_to_target =
        poses[tgt].pose.inverse().compose(poses[src].pose);
    const auto center = centerFromTransform(source_to_target);
    ASSERT_TRUE(center.has_value()) << "pair " << src << "->" << tgt;
    EXPECT_NEAR(center->x, expected_c.x, 1e-6);
    EXPECT_NEAR(center->y, expected_c.y, 1e-6);

    StereoCenterSample sample;
    sample.source_index = src;
    sample.target_index = tgt;
    sample.yaw_sector = sectorForYaw(poses[src].pose.yaw);
    sample.accepted = true;
    sample.center = *center;
    sample.mount_xy = impliedCameraLinkXy(*center, identity_mount);
    samples.push_back(sample);
  }

  const auto est = robustEstimate(samples, /*seed=*/42, thr);
  EXPECT_TRUE(est.reliable) << [&] {
    std::string s;
    for (const auto& r : est.unreliable_reasons) s += r + ";";
    return s;
  }();
  EXPECT_GE(est.accepted_pairs, thr.min_accepted_pairs);
  EXPECT_GE(est.sectors_used, thr.min_sectors);
  EXPECT_NEAR(est.median_xy.x_m, recorded_xy.x_m, 1e-4);
  EXPECT_NEAR(est.median_xy.y_m, recorded_xy.y_m, 1e-4);

  const auto agg = classify(est, recorded_xy, thr);
  EXPECT_EQ(agg.result_class, ResultClass::kConsistent);
  EXPECT_EQ(resultExitCode(agg.result_class), 0);
  EXPECT_NEAR(agg.delta_xy.x_m, 0.0, 1e-4);
  EXPECT_NEAR(agg.delta_xy.y_m, 0.0, 1e-4);
}

TEST(StereoCalibAnalysis, ClassifyLinfConsistent) {
  StereoEstimate est;
  est.reliable = true;
  est.median_xy = {0.320, 0.050};
  est.ci_half_width = {0.005, 0.004};
  est.accepted_pairs = 50;
  est.sectors_used = 8;

  StereoThresholds thr;
  thr.agreement_floor_m = 0.010;

  const MountXy recorded{0.325, 0.048};  // L∞ = 0.005 <= floor 0.010
  const auto agg = classify(est, recorded, thr);
  EXPECT_EQ(agg.result_class, ResultClass::kConsistent);
  EXPECT_NEAR(agg.delta_xy.x_m, 0.320 - 0.325, 1e-12);
  EXPECT_NEAR(agg.delta_xy.y_m, 0.050 - 0.048, 1e-12);
  EXPECT_EQ(resultExitCode(agg.result_class), 0);
}

TEST(StereoCalibAnalysis, ClassifyLinfLikelyOffsetError) {
  StereoEstimate est;
  est.reliable = true;
  est.median_xy = {0.400, 0.050};
  est.ci_half_width = {0.005, 0.004};
  est.accepted_pairs = 50;
  est.sectors_used = 8;

  StereoThresholds thr;
  thr.agreement_floor_m = 0.010;
  // threshold = max(0.010, max(0.005,0.004)) = 0.010
  // L∞ vs recorded (0.32, 0.05) = 0.08 > 0.010
  const MountXy recorded{0.320, 0.050};
  const auto agg = classify(est, recorded, thr);
  EXPECT_EQ(agg.result_class, ResultClass::kLikelyOffsetError);
  EXPECT_EQ(resultExitCode(agg.result_class), 2);
}

TEST(StereoCalibAnalysis, ClassifyInconclusiveWhenUnreliable) {
  StereoEstimate est;
  est.reliable = false;
  est.median_xy = {0.30, 0.04};
  est.ci_half_width = {0.05, 0.05};
  est.unreliable_reasons = {"insufficient_accepted_pairs"};

  StereoThresholds thr;
  const MountXy recorded{0.30, 0.04};
  const auto agg = classify(est, recorded, thr);
  EXPECT_EQ(agg.result_class, ResultClass::kInconclusive);
  EXPECT_EQ(resultExitCode(agg.result_class), 3);
}

TEST(StereoCalibAnalysis, ClassifyUsesCiWhenWiderThanFloor) {
  StereoEstimate est;
  est.reliable = true;
  est.median_xy = {0.330, 0.050};
  est.ci_half_width = {0.020, 0.008};  // max CI half-width = 0.020
  est.accepted_pairs = 50;
  est.sectors_used = 8;

  StereoThresholds thr;
  thr.agreement_floor_m = 0.010;
  // threshold = max(0.010, 0.020) = 0.020; L∞ = 0.015 <= 0.020 → consistent
  const MountXy recorded{0.315, 0.050};
  const auto agg = classify(est, recorded, thr);
  EXPECT_EQ(agg.result_class, ResultClass::kConsistent);
}

TEST(StereoCalibAnalysis, ResultExitCodes) {
  EXPECT_EQ(resultExitCode(ResultClass::kConsistent), 0);
  EXPECT_EQ(resultExitCode(ResultClass::kLikelyOffsetError), 2);
  EXPECT_EQ(resultExitCode(ResultClass::kInconclusive), 3);
}

}  // namespace
}  // namespace orb_slam3_wrapper
