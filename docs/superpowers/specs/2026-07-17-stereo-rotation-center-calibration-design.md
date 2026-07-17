# Stereo ORB-SLAM Rotation-Center Calibration Design

**Date:** 2026-07-17  
**Repository:** `/home/duc/robot/src/orb_slam3_ros2`  
**Primary bag fixture:** `/home/duc/robot/bag/inplace-rotate`  
**Related design:** `docs/superpowers/specs/2026-07-16-lidar-rotation-center-calibration-design.md`

## Purpose

Build an offline, read-only calibration check that:

1. Runs full ORB-SLAM3 stereo tracking on an in-place rotation bag.
2. Records the 6DoF left-camera trajectory.
3. Estimates the planar lever arm from the left camera origin to the robot
   rotation center using pairwise relative SE(2) (same center math as the lidar
   tool).
4. Converts that lever arm into an implied `base_link → camera_link` planar
   translation `(x, y)` and compares it to the bag’s recorded `/tf_static` so
   the operator can judge whether the static camera mount is correct.

The primary output is the raw estimated mount translation and uncertainty, not
only a pass/fail label. The tool must never modify TF, URDF, launch files, the
bag, or runtime ROS state.

## Motivation

The lidar rotation-center tool estimates `base_scan.x` under pure spin. The same
physical assumption (near-pure rotation about a vertical axis through
`base_link` origin) also constrains the camera mount. The inplace-rotate bags
already contain rectified stereo, camera_info, odom/IMU, and a full camera
`/tf_static` tree, and the scene has rich RealSense infra features suitable for
ORB tracking.

A known risk in this repo is that recorded mounts differ by bag profile:

| Source | `base_link → camera_link` |
|---|---|
| inplace-rotate `/tf_static` | ≈ `(0.326, 0.000, 0.173)` m, pitch ≈ −15° |
| full-bag supplemental (`tasterobot_bag.yaml`) | `(0.346, 0.010, 0.100)` m, pitch −0.2618 rad |

This tool validates the **per-bag recorded** camera mount (same trust model as
the lidar tool’s use of recorded `base_scan`).

## Recorded Dataset (v1 fixture)

Primary fixture: `/home/duc/robot/bag/inplace-rotate` (MCAP, ~91.35 s, ~4
in-place turns, |v| ≪ 0.01 m/s, median |ω| ~0.3 rad/s).

Useful topics:

| Topic | Role |
|---|---|
| `/camera/camera/infra1/image_rect_raw` | Left stereo image |
| `/camera/camera/infra2/image_rect_raw` | Right stereo image |
| `/camera/camera/infra1/camera_info` | Left intrinsics / rectified model |
| `/camera/camera/infra2/camera_info` | Right intrinsics + baseline in P |
| `/tf_static` | Recorded camera + lidar mounts (source of truth for comparison) |
| `/odom` | Optional gates only (stable rotation); **not** used for the center estimate in v1 |

Optional later fixtures: `inplace-rotate-1rev`, `inplace-rotate-2rev`,
`inplace-rotate-1rev-015rad`.

Stereo settings already used by the wrapper:

- Config: `orb_slam3_wrapper/config/tasterobot_stereo.yaml`
- Baseline ≈ 0.050188 m
- Frames: `camera_infra1_optical_frame`, `camera_infra2_optical_frame`

## Observability And Scope

### In scope (v1)

- Offline MCAP read; headless full ORB-SLAM3 stereo tracking.
- Log timestamped 6DoF left-camera poses + tracking state.
- Planar SE(2) only: estimate rotation center in the horizontal plane, then
  implied `base_link → camera_link` **translation xy**.
- Fix orientation of `base_link → camera_link` (and the optical chain) from bag
  `/tf_static`; do **not** estimate pitch/roll/yaw or z.
- Single method: ORB stereo trajectory only (no wheel/IMU multi-method
  consensus in v1).
- Self-contained HTML + JSON report; scientific exit codes; fail closed.
- Unit tests for SE(2) center math, frame projection, mount mapping, and
  classification.

### Out of scope (v1)

- Estimating full 6DoF mount or pitch/z.
- Multi-method consensus with `/odom` or `/imu` as independent estimators.
- Auto-writing TF, URDF, launch, or bag metadata.
- Live / online calibration.
- Using ORB map quality as a product (poses only; map is an internal SLAM
  artifact).
- Evaluating lidar mount (already covered by the lidar tool).

### Physical assumption

The robot executes a near-pure spin about the vertical axis through
`base_link` origin. Under that assumption, the left-camera origin traces a
circle (or arc set) whose center is the rotation axis. Relative SE(2) poses
observe the lever arm without requiring a closed full circle.

## Architecture

Add one offline executable in `orb_slam3_wrapper` (preferred: package already
links ORB-SLAM3, stereo settings, and pose conversion). Do **not** place this
tool in `orb_lidar_mapper`; sensor domain and dependencies differ. Shared
pairwise-center math may be duplicated initially or lightly factored later if
both tools stay stable.

```bash
ros2 run orb_slam3_wrapper stereo_rotation_center_check \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output artifacts/inplace-rotate-stereo-calibration \
  [--vocab path/to/ORBvoc.txt] \
  [--settings orb_slam3_wrapper/config/tasterobot_stereo.yaml] \
  [--overwrite]
```

Wrapper script (mirrors lidar):

```bash
tools/run_stereo_rotation_center_calibration.sh \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output tools/stereo-rotation-center-report/inplace-rotate
```

The executable reads the MCAP through `rosbag2_cpp`. It does not play the bag
as a live graph for estimation (though it may construct a minimal ROS context
if ORB-SLAM3/wrapper APIs require it). It must not publish corrected TF or
mutate workspace files.

### Pipeline stages

1. **Bag ingest**  
   Extract stereo image pairs (approx-time aligned), camera_info, `/tf_static`,
   and optionally `/odom` for motion gates. Validate presence of required
   topics and the recorded `base_link → camera_link` edge (and optical chain
   needed to resolve left optical).

2. **Intrinsics / baseline check**  
   Validate rectified pinhole models and stereo baseline against settings
   (reuse concepts from existing `Calibration::fromCameraInfo` /
   metrics gates). Fail operationally if camera_info is unusable.

3. **ORB-SLAM3 stereo tracking**  
   Construct ORB-SLAM3 `System` in stereo mode with vocab + settings. Feed
   every valid stereo pair with timestamps. Record for each successful track:
   - timestamp  
   - `T_world_left` (6DoF, left optical)  
   - tracking state (OK / recently lost / lost)  

   Drop LOST frames from estimation. Report tracking success fraction.

4. **Pose preparation (planar projection)**  
   Build fixed transforms from bag `/tf_static`:
   - `T_base_camera_link` (recorded, full SE3)  
   - `T_camera_link_left_optical` (RealSense chain + optical quaternion)  
   - thus `T_base_left_optical`  

   Define a **horizontal** frame aligned with `base_link` xy (vertical = base
   z). Project each accepted left-optical pose into horizontal SE(2):
   - yaw about vertical  
   - xy of the left optical origin in the horizontal world  

   Reject poses with large residual vertical motion relative to pure planar
   spin (diagnostic + optional pair gate).

5. **Pair selection**  
   Form pose pairs with yaw separation in **[10°, 30°]** (same band as lidar)
   inside stable-rotation intervals:
   - Prefer `/odom` gates when available: low linear speed, sufficient |ω|.  
   - If odom missing: use ORB-only heuristics (high yaw rate, small
     translational inconsistency over short windows).  

   Thin to avoid highly correlated adjacent pairs if needed.

6. **Center estimation (pairwise SE(2))**  
   For relative transform mapping source pose into target pose:

   ```text
   p_target = R * p_source + t
   c = (I - R)^-1 * t
   ```

   `c` is the planar vector from the left-camera horizontal origin to the
   rotation center, expressed in the horizontal left-camera frame at the pair
   epoch (same convention lock as the lidar tool; synthetic tests must freeze
   signs).

   Aggregate accepted `c` samples with robust statistics (median, MAD/IQR,
   bootstrap 95% CI on `c_x` and `c_y`).

7. **Map lever arm → implied `base_link → camera_link` xy**  
   Using fixed recorded orientation and fixed `camera_link → left optical`
   chain, convert estimated planar lever arm into the `base_link → camera_link`
   translation **xy** that places the rotation center at `base_link` origin.

   Report side-by-side:
   - recorded `base_link → camera_link` xy (from bag)  
   - estimated xy  
   - delta and CI  

   Recorded z and R are printed as fixed context only.

8. **Classification + report**  
   Write artifacts, then exit with a scientific class code.

### Components (logical)

| Component | Responsibility |
|---|---|
| `StereoBagReader` | Offline MCAP stereo + camera_info + tf_static (+ optional odom) |
| `StaticCameraMount` | Parse/resolve recorded mount and optical chain |
| `OrbStereoTracker` | Headless ORB-SLAM3 stereo run; pose log |
| `PlanarPoseProjector` | Optical 6DoF → horizontal SE(2) using fixed extrinsics |
| `MotionSegmentSelector` | Stable spin intervals (odom and/or ORB heuristics) |
| `RotationCenterEstimator` | Pairwise `(I−R)⁻¹ t` + robust aggregate (pattern-aligned with lidar) |
| `MountXyMapper` | Lever arm ↔ implied `base_link → camera_link` xy |
| `StereoCalibrationAnalysis` | Reliability + `ResultClass` |
| `StereoCalibrationReportWriter` | JSON/CSV/HTML |

## Frame Conventions

```text
base_link
  └── camera_link                    (recorded xyz + pitch ≈ −15° on fixture)
        └── … RealSense chain …
              └── camera_infra1_optical_frame   (ORB left)
              └── camera_infra2_optical_frame
```

- ORB poses are left optical in ORB world (ROS optical: Z forward, X right, Y
  down after the standard optical quaternion).
- Horizontal plane = `base_link` xy.
- Rotation center for v1 = `base_link` origin (same pure-spin assumption as
  lidar).
- The quantity operators care about comparing to static TF is
  **`base_link → camera_link` translation xy**, not the intermediate lever arm
  alone—but both are written to JSON for debugging.

## Reliability And Exit Codes

### ORB run gates

- Minimum number of successfully tracked stereo frames (exact N tuned after
  first real run; start with a conservative floor, e.g. hundreds of OK frames
  on a multi-turn bag).
- Tracking-loss fraction below a predeclared threshold.
- camera_info / baseline consistent with settings.

### Pair gates

- Δyaw ∈ [10°, 30°].
- Finite, well-conditioned `(I − R)⁻¹`.
- Estimated center inside a plausible box (e.g. \|c\| ≤ 1.0 m).
- Optional: reject large vertical residual after planar projection.

### Reliable estimate (provisional; may loosen after first ORB bag)

Mirror lidar spirit:

- ≥ 40 accepted pairs  
- Adequate yaw-sector coverage (e.g. ≥ 6/8 equal sectors)  
- Bootstrap 95% CI half-width on each of estimated mount x and y ≤ 0.015 m  
  (document as provisional; ORB noise may require a slightly looser v1 default)

### Result classes

Agreement metric is **L∞** on mount xy:  
`max(|Δx|, |Δy|) ≤ max(0.010 m, max(CI_x, CI_y) half-width)`.

| Class | Meaning | Exit |
|---|---|---:|
| `CONSISTENT` | Estimate reliable and L∞ agreement holds vs recorded mount | 0 |
| `LIKELY_OFFSET_ERROR` | Estimate reliable but L∞ disagreement beyond threshold | 2 |
| `INCONCLUSIVE` | Weak tracking, few pairs, broad CI, bad sector coverage, etc. | 3 |
| operational failure | Unreadable bag, missing topics/TF, ORB init failure, write failure | 1 |

### Invariants

- Raw numbers first; classification second.
- Fail closed on missing data or broken assumptions.
- Never auto-edit TF / URDF / launch / bag.
- Write full report before returning scientific exit codes 0/2/3.
- Predeclare thresholds in code and report JSON (no silent retuning per bag).

## Outputs

Under `--output DIR`:

| File | Content |
|---|---|
| `calibration.json` | Recorded mount, estimated xy, delta, CI, pair counts, ORB tracking stats, thresholds, `ResultClass` |
| `centers.csv` | Per-pair center samples and gate metadata |
| `trajectory.csv` | Timestamped 6DoF ORB left poses + tracking state |
| `report.html` | Self-contained summary: comparison table, top-down trajectory plot, center scatter, recorded vs estimated mount diagram, tracking stats |

Generated report directories should be gitignored (same pattern as
`tools/lidar-rotation-center-report/`), with optional sample HTML under
`docs/samples/` after a successful run if desired.

## Testing Strategy

### Unit (required)

1. **Synthetic pure rotation SE(2):** known lever arm `c`; generated relative
   poses recover `c` within tight tolerance; lock `(I−R)⁻¹ t` sign convention.
2. **Planar projection:** optical poses with recorded pitch project to expected
   horizontal SE(2).
3. **Mount mapping:** known lever arm maps to expected
   `base_link → camera_link` xy given fixed R and optical chain.
4. **Classification:** synthetic reliable/unreliable aggregates hit the correct
   `ResultClass` and exit mapping.

### Integration (optional / local)

- Full ORB-SLAM3 on the real inplace-rotate bag is heavy (vocab + runtime). CI
  may skip full-bag ORB and only run unit tests.
- Local first success criterion: run on
  `/home/duc/robot/bag/inplace-rotate`, produce report, and inspect estimated
  xy vs recorded `(0.326, 0.000)` (approx).

## CLI And Operator Workflow

```bash
# after workspace build + source
tools/run_stereo_rotation_center_calibration.sh \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output tools/stereo-rotation-center-report/inplace-rotate

# inspect
xdg-open tools/stereo-rotation-center-report/inplace-rotate/report.html
# or read calibration.json ResultClass / estimated_xy / recorded_xy
```

Progress reporting during the long ORB pass is desirable (stage labels +
percent), following the lidar tool’s UX lessons.

## Non-Goals (restated)

- Not a replacement for factory stereo baseline calibration.
- Not a full extrinsic optimizer (no free pitch/z in v1).
- Not multi-sensor consensus (ORB-only v1).
- Not automatic correction of static TF.

## Open Tuning Items (decide after first real run)

These are implementation parameters with provisional defaults, not design
ambiguities. Agreement metric is already fixed to **L∞** (see Result classes).

- Exact minimum tracked-frame count and max loss fraction.
- Whether CI half-width default stays 0.015 m or loosens for ORB.
- Keyframe thinning density for pair generation.

## Success Criteria For v1 Delivery

1. Offline CLI runs ORB-SLAM3 stereo on `inplace-rotate` without bag play.
2. Produces `calibration.json` + `report.html` with estimated and recorded
   `base_link → camera_link` xy.
3. Unit tests lock center-math convention and mount mapping.
4. Exit codes follow the scientific classes above.
5. No code path writes TF or mutates the bag.

## Relationship To Lidar Tool

| Aspect | Lidar tool | This stereo tool |
|---|---|---|
| Package | `orb_lidar_mapper` | `orb_slam3_wrapper` |
| Sensor stream | deskewed scans + ICP | ORB-SLAM3 stereo poses |
| Center math | `c = (I−R)⁻¹ t` from ICP SE(2) | same formula from ORB SE(2) |
| Reported mount | `base_scan.x` (y,yaw constrained) | `base_link → camera_link` xy (R,z fixed) |
| Methods v1 | Odom / IMU / Existing `/scan` | ORB only |
| Bag fixture | inplace-rotate* | same family first |
| Auto TF edit | never | never |
