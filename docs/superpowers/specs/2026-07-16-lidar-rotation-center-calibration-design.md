# Lidar Rotation-Center Calibration Design

**Date:** 2026-07-16
**Repository:** `/home/duc/robot/src/orb_slam3_ros2`
**Bag fixture:** `/home/duc/robot/bag/inplace-rotate`

## Purpose

Build an offline, read-only calibration check that estimates the planar vector
from the lidar origin to the robot rotation center. The first version uses the
physical mount constraints `base_scan.y = 0` and `base_scan.yaw = pi`, so its
calibrated parameter is only the forward lidar offset `base_scan.x`.

The primary output is the raw measured center in `base_scan`, not only a
pass/fail label. The tool must compare three independently preprocessed lidar
streams and must never modify TF, URDF, the bag, or runtime ROS state.

## Recorded Dataset

The fixture is an MCAP bag lasting 91.35394 seconds. Its useful inputs are:

| Topic | Rate/count | Role |
|---|---:|---|
| `/scan_origin` | 915 scans, about 10 Hz | Raw 3,240-ray scans with per-ray timing |
| `/scan` | 915 scans, about 10 Hz | Existing undistortion output, 1,567 rays |
| `/odom` | 1,827 messages, 20 Hz | Full SE(2) ray deskew reference |
| `/imu` | 18,289 messages, about 200 Hz | Gyro-based ray deskew reference |
| `/tf_static` | 2 messages | Recorded mount transform source |

The active interval contains approximately four in-place turns. Median angular
speed is about 0.296 rad/s and observed linear speed is below 0.005 m/s. Each
raw scan lasts about 98.56 ms, producing approximately 1.69 degrees of motion
during one sweep.

The recorded transform is:

```text
base_link -> base_scan
translation = (0.260, 0.000, 0.100) m
yaw = pi
```

Under the mount constraints, the expected robot rotation center expressed in
`base_scan` is `(0.260, 0.000) m`.

## Observability And Scope

Pure rotation observes the rotation-center vector in lidar coordinates:

```text
c_lidar = -R(yaw)^T * translation
```

It cannot uniquely identify planar translation and yaw as three independent
parameters. This tool therefore treats lateral offset `y = 0` and yaw `= pi`
as exact mount constraints and estimates only forward offset `x`. It still
reports an unconstrained raw `center_y` as a model-quality diagnostic.

Estimating unconstrained `x`, `y`, and yaw requires a later dataset with
translational excitation and is outside this design.

## Architecture

Add one offline executable to `orb_lidar_mapper`:

```bash
ros2 run orb_lidar_mapper lidar_rotation_center_check \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output artifacts/inplace-rotate-calibration
```

The executable reads the MCAP directly through `rosbag2_cpp`. It does not play
the bag, publish topics, start a ROS graph, or change source files.

Components have these responsibilities:

1. `RotationBagReader` extracts scans, odometry, IMU samples, and recorded
   static transforms into timestamped, immutable records.
2. `MotionSegmentSelector` finds stable in-place rotation intervals and rejects
   startup, shutdown, translation, and insufficient-motion intervals.
3. `OdomRayDeskewer` interpolates `/odom` SE(2) at each raw ray timestamp and
   transforms every finite ray to the scan midpoint.
4. `ImuRayDeskewer` integrates `/imu.angular_velocity.z` at each ray timestamp.
   It models the lidar's translational arc using the current candidate offset
   and iterates deskew, ICP, and offset update until stable.
5. `UndistortedScanAdapter` consumes `/scan` without inventing per-ray timing.
   It associates output scans to raw scans by sequence and measured timestamp
   delay.
6. `PlanarIcp` uses PCL's constrained two-dimensional transformation
   estimation. It produces relative SE(2), convergence, overlap, residual, and
   correspondence statistics.
7. `RotationCenterEstimator` converts accepted relative transforms into raw
   center estimates and robustly combines them.
8. `MapSharpnessValidator` independently sweeps candidate forward offsets and
   scores alignment between temporally separated point subsets.
9. `CalibrationReportWriter` writes machine-readable data and a self-contained
   HTML report.

## Common ICP Experiment

All three preprocessing paths use the same stable rotation intervals and the
same scan-pair schedule. Scan pairs are separated by 10 to 30 degrees. This
avoids the numerical instability of inverting `(I - R)` for adjacent scans
while retaining useful overlap.

For an ICP transform mapping a source scan into a target scan,

```text
p_target = R * p_source + t
```

the fixed rotation center in lidar coordinates satisfies:

```text
c = (I - R)^-1 * t
```

The implementation must lock this transform convention with synthetic tests.
It must not silently change signs to make the result resemble the recorded
transform.

Finite ranges satisfying the message bounds and the default calibration cap
`0.15 <= range <= 12.0 m` are converted to planar points. The cap is
configurable and is recorded in every report. NaN and infinity ranges
contribute no points. Deterministic voxel or angular downsampling may be used,
but all methods must report their retained point counts so comparisons are
normalized for `/scan` having fewer rays.

## Three Deskew Paths

### Odom Deskew

For each `/scan_origin` ray, interpolate `/odom` at the ray timestamp and at the
scan midpoint. Apply their relative SE(2) transform to express the ray at the
midpoint. Missing interpolation coverage rejects that scan rather than using a
zero-motion fallback.

### IMU Deskew

Integrate the calibrated `imu_link` Z angular velocity relative to the scan
midpoint. Given a candidate forward offset, model both orientation change and
the lidar origin's arc around the base center. Begin at the recorded 0.260 m
offset, estimate a center through ICP, and repeat until the offset changes by
less than 0.5 mm or a fixed iteration limit is reached. Failure to converge is
reported as inconclusive for this path.

### Existing Undistorted Scan

Use `/scan` as delivered. Its `scan_time` and `time_increment` are zero, so the
tool must not deskew it again. Associate it with raw scan sequence using order
and the robust median timestamp delay; reject ambiguous or missing pairs. Its
ICP schedule is selected by the same `/odom` angular separation used by the raw
paths.

## Robust Estimation And Quality Gates

The initial defaults are fixed before the first real estimate:

- stable angular speed: `0.15 <= |omega| <= 0.45 rad/s`;
- maximum absolute linear speed: `0.02 m/s`;
- scan-pair angular separation: 10 to 30 degrees;
- minimum overlap: 0.40, defined as accepted correspondences divided by the
  smaller downsampled cloud size;
- maximum trimmed ICP RMSE: 0.05 m;
- maximum ICP-versus-odom rotation error: 2 degrees;
- plausible raw center: `0.0 <= center_x <= 1.0 m` and
  `|center_y| <= 0.25 m`.

These defaults may be overridden only through explicit CLI options, and both
defaults and effective values are written to `calibration.json` and the HTML
report. An ICP pair is accepted only when all of these hold:

- both scans belong to a stable rotation interval;
- absolute linear speed is below the effective limit;
- angular separation is inside the effective range;
- ICP converges and meets the effective overlap and residual limits;
- ICP rotation agrees with `/odom` rotation within the effective yaw limit;
- inferred center and all intermediate values are finite;
- inferred center lies inside the effective physical plausibility bound.

The estimator uses a robust location statistic or M-estimator, not a plain
mean. It reports median, dispersion, outlier count, accepted/attempted pairs,
and a deterministic bootstrap 95% confidence interval. Pair selection must
span the full rotation interval so one wall or one room sector cannot dominate.

Every rejection remains countable by reason. A method is reliable only when it
has at least 40 accepted pairs, accepted pairs cover at least six of eight
equal robot-yaw sectors, its bootstrap 95% forward-offset confidence interval
has half-width at most 0.015 m, and `|median(center_y)| <= 0.020 m`. Weak room
geometry or inconsistent clockwise and counterclockwise estimates must fail
closed as `INCONCLUSIVE`.

## Map-Sharpness Cross-Check

The sharpness validator performs a coarse one-dimensional sweep from 0.180 to
0.340 m in 0.002 m steps, followed by a 0.00025 m refinement around the best
coarse candidate. For each candidate, it transforms deskewed scans through
`/odom` and compares temporally separated point subsets using trimmed
nearest-neighbor residuals. Self-neighbors from the same scan or immediately
adjacent scans are excluded.

The report includes the complete score curve and selected minimum. A sharpness
minimum is reliable only when it is unique, its score is at least 3% lower than
the scores 0.020 m to either side, and it lies within 0.010 m of the ICP
consensus. A boundary, flat, or multimodal minimum is inconclusive.

## Result Classification

Each path always reports its raw values first:

```text
method
center_x_m
center_y_m
forward_offset_m
delta_from_recorded_m
confidence_95_m
accepted_pairs
attempted_pairs
icp_residual_m
overlap_ratio
```

At least two of the three methods must be reliable. Reliable method medians
agree only when their maximum forward-offset spread is at most 0.015 m. The
consensus is their inverse-variance weighted estimate, with uncertainty no
narrower than the narrowest contributing method. The aggregate classification
is:

- `CONSISTENT`: reliable methods agree, the sharpness cross-check is reliable,
  and `|consensus - 0.260| <= max(0.010 m, consensus CI half-width)`.
- `LIKELY_OFFSET_ERROR`: reliable methods and the sharpness cross-check agree,
  and `|consensus - 0.260| > max(0.010 m, consensus CI half-width)`.
- `INCONCLUSIVE`: too little valid data, poor geometry, broad uncertainty,
  reliable-method spread above 0.015 m, or an unreliable sharpness curve.

The first real run must expose raw estimates and apply these predeclared
thresholds without tuning them to the observed answer. Later threshold changes
require an explicit configuration change recorded in the report. No result may
be upgraded from `INCONCLUSIVE` merely because it is close to the recorded TF.

## Outputs And Visualization

The output directory contains:

- `calibration.json`: full configuration, bag identity, TF, method results,
  aggregate classification, and rejection counters;
- `centers.csv`: one row per attempted ICP pair, including rejected pairs and
  rejection reason;
- `sharpness.csv`: candidate offset and normalized score per method;
- `report.html`: self-contained, read-only report.

The HTML report shows:

- recorded center `(0.260, 0.000) m` and raw center scatter for all methods;
- median and 95% confidence region per method;
- raw forward offsets and deltas from the recorded value;
- accepted and rejected pair counts and diagnostics;
- candidate-offset sharpness curves;
- comparable map renderings at recorded and estimated offsets;
- explicit disagreement and inconclusive warnings.

The report must remain usable by opening the file directly or serving its
directory over plain HTTP. It contains no control API and performs no writes.

## Error Handling

Exit statuses are exact: `0` for `CONSISTENT`, `2` for
`LIKELY_OFFSET_ERROR`, `3` for `INCONCLUSIVE`, and `1` for an operational
failure. Operational failures include unreadable bags, missing required
topics, invalid recorded TF, malformed scan timing, no stable rotation
interval, or failure to write complete outputs. Scientific outcomes always
write a complete report before returning their status.

Partial output files are written atomically. Existing output directories are
not overwritten unless an explicit overwrite option is supplied. The input
bag is always opened read-only.

## Testing

Pure unit and integration fixtures cover:

- transform convention and center sign;
- synthetic asymmetric rooms with known offsets;
- offsets below and above 0.260 m;
- rotational ray distortion and midpoint deskew;
- odom interpolation and missing-coverage rejection;
- IMU integration, bias, and iterative convergence failure;
- `/scan` timestamp-delay association and zero timing fields;
- NaN, infinity, range limits, and sparse scans;
- ICP outliers and poor-overlap rejection;
- circular rooms, blank walls, and other degenerate geometry;
- robust aggregation, bootstrap reproducibility, and sector balancing;
- map-sharpness minima, flat curves, and multimodal curves;
- JSON/CSV/HTML completeness and atomic writes.

Synthetic acceptance requires recovery of known forward offsets within a
specified test tolerance and must demonstrate that deliberately wrong offsets
produce worse sharpness scores. Tests must include a wrong-sign regression so
`+0.260 m` cannot be accidentally reported as `-0.260 m`.

The real bag is an immutable acceptance fixture. Its first run must report all
three raw estimates and diagnostics without hardcoding the expected measured
answer.

## Non-Goals

- Automatically editing `/tf_static`, URDF, or launch configuration.
- Estimating unconstrained lateral offset or lidar yaw from pure rotation.
- Replacing the live mapper's scan deskew implementation.
- Evaluating camera extrinsics.
- Publishing a canonical `/map` or `map->odom` transform.
