# Kiro Handoff — Deferred Straight-Scan Lidar Mapping

**Updated:** 2026-07-16
**Repository:** `/home/duc/robot/src/orb_slam3_ros2`
**Current state:** direct-dashboard and mapper hardening committed on `main`
**Kiro mapper changes:** implemented, validated, and committed after explicit user approval.

## Purpose and Current Result

This handoff updates the earlier Phase-1 handoff with the completed conservative lidar-mapping change requested by the user:

- commit lidar geometry only for stationary or approximately straight scans;
- defer a scan until wheel odometry covers its complete sweep;
- require compatible tracked ORB frames on both sides of the complete sweep;
- use wheel motion for per-ray motion and a distributed residual between the two visual anchors;
- preserve enough per-ray motion metadata to rebuild the map after ORB graph corrections;
- retain the existing wheel-only provisional behavior during tracking loss.

The user viewed the custom dashboard during a fresh replay and reported that the 2D map is better. This is expected: the mapper no longer permanently archives a callback-time partial scan or accepts scans while the robot is meaningfully turning.

Do **not** redesign this as an IMU integration task. The user explicitly chose this conservative wheel + two-visual-anchor approach. The Handsfree and D435i IMU paths remain out of scope unless separately requested.

## Offline Rotation-Center Calibration Result

An independent, read-only calibration check was run against the immutable
`/home/duc/robot/bag/inplace-rotate` MCAP (91.353939991 seconds; 915
`/scan_origin`, 915 `/scan`, 1,827 `/odom`, and 18,289 `/imu` messages). The
first fresh attempt at
`artifacts/inplace-rotate-calibration-20260716` returned operational exit 1
because `/imu` contains contiguous duplicate header timestamps; that failed
output path was not reused. Approved commits `21f0a9f` and
`b4849a1` changed only the reader and its tests so equal contiguous IMU stamps
are combined by a finite-checked `long double` arithmetic mean, while strictly
decreasing stamps still fail. The retry dataset therefore contains 8,569
unique IMU timestamp groups.

The outputs in `artifacts/inplace-rotate-calibration-20260716-retry1` and
`retry2` remain preserved but are scientifically stale. Both were generated
before `7794e53` corrected the source-to-target yaw convention; retry2 changed
only the mobile report layout. Their zero accepted-pair result must not be used
for calibration decisions. After rebuilding approved commit `7794e53`, exactly
one additional immutable-bag run was made into a new path:

```bash
source install/setup.bash
ros2 run orb_lidar_mapper lidar_rotation_center_check \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output "$PWD/artifacts/inplace-rotate-calibration-20260716-retry3"
```

No threshold options were supplied or tuned after seeing the output. All
predeclared gates stayed unchanged: angular speed 0.15–0.45 rad/s, linear
speed at most 0.02 m/s, pair separation 10–30 degrees, overlap at least 0.40,
ICP RMSE at most 0.05 m, ICP/odom yaw error at most 2 degrees, and the existing
reliability, plausibility, consensus, and sharpness gates.

The tool returned accepted scientific exit 3, `INCONCLUSIVE`, with aggregate
reason `sharpness_non_unique`. The recorded center is
`(0.260, 0.000) m`. Every raw method field and rejection counter is:

| Method | Center x/y (m) | Forward/delta (m) | 95% CI x (m) | Pairs | Sectors | RMSE/overlap | Rejections |
|---|---|---|---|---:|---:|---|---|
| Odom | 0.2379652303 / -0.0061142640 | 0.2379652303 / -0.0220347697 | [0.2352081836, 0.2386867630] | 171/512 | 8 | 0.0082904437 / 0.7445378151 | yaw disagreement 200; insufficient overlap 110; implausible center 29; trimmed RMSE 2 |
| IMU | 0.2375527669 / -0.0040162076 | 0.2375527669 / -0.0224472331 | [0.2350004394, 0.2386822376] | 172/512 | 8 | 0.0083020599 / 0.7443506196 | yaw disagreement 198; insufficient overlap 112; implausible center 28; trimmed RMSE 2 |
| Existing `/scan` | 0.2476510983 / 0.0013325828 | 0.2476510983 / -0.0123489017 | [0.2461113510, 0.2500656229] | 172/512 | 8 | 0.0094094900 / 0.7543158596 | yaw disagreement 172; insufficient overlap 134; implausible center 34 |

All three methods are reliable. The raw consensus is 0.24063816538065377 m
with 95% CI [0.23855052962519216, 0.2427258011361154] m. The replacement
temporal sharpness sweep is unreliable: its raw minimum is
0.22200000000000003 m with score 0.337169975947602 and rejection reason
`sharpness_non_unique`. This keeps the aggregate classification inconclusive.

Relative to stale retry2, yaw-disagreement rejections fell from 301 to 200,
302 to 198, and 281 to 172 for Odom, IMU, and Existing `/scan`. This expected
change follows from correcting the sign of the expected lidar
source-to-target yaw. Because yaw is checked before overlap, RMSE, and center
plausibility, valid pairs now reach those later gates and 171/172/172 pairs are
accepted. No threshold was changed.

All four files (`calibration.json`, `centers.csv`, `sharpness.csv`, and
`report.html`) are nonempty in the retry3 artifact. The exact retry3 report was
checked at 1440x900, 768x1024, and 390x844 with numeric JSON agreement,
method/legend colors, signs, warning, map framing, and responsive containment;
there were no browser errors or post-load network requests. At 390x844, every
one of the ten labels and exact values for each of the three methods is visible
and contained in its stacked row, with no table or document horizontal
overflow. Full screenshots were inspected. They show one scientific
presentation limitation: all 1,021 rejected centers lie outside the fixed
scatter canvas, so no red rejected outlines are visible. These do not upgrade
the scientific result.

No offset recommendation is made because the aggregate is inconclusive. No
TF, URDF, source configuration, or bag data changed. The remaining review
Minor is that pair selection still does not inspect interpolated endpoint
twists for per-pair linear-speed gating; addressing it requires a separate
API/data-flow change.

## Current Mapper Behavior

### Accepted normal scans

`MapperNode` queues a raw `LaserScan` and acquisition-time base-to-lidar transform. It processes a normal scan only when all of the following are true:

1. Wheel odometry has valid interpolation coverage for the entire interval from the first to final ray.
2. The peak-to-peak unwrapped wheel-yaw excursion across the complete sweep is no more than `max_scan_yaw_change_rad`.
3. Valid ORB tracked frames bracket the complete scan interval.
4. Both anchors share map ID and graph revision, and every intervening frame remains valid/compatible.
5. Each visual endpoint is within `visual_anchor_max_gap_ms` of its corresponding scan endpoint.

No directional prefix of an incompletely covered scan is archived. A turn-rejected scan is dropped rather than being inserted with incorrect geometry.

### Per-ray deskew and initial map placement

For each accepted ray, `ScanDeskewer::deskewBracketed()` calculates the wheel pose at the ray timestamp. It builds a visually constrained base pose using:

```text
start_map_pose * inverse(start_wheel_pose) * ray_wheel_pose
  * pow(predicted_end.inverse() * end_map_pose, ray_alpha)
```

where `ray_alpha` advances from the visual start frame to the visual end frame. The scan is initially inserted using this geometry and an equivalent bracketed scan pose.

### Graph-corrected rebuilds

`ArchivedScan` retains `ArchivedBracketedMotion`:

- start/end tracked-frame indices;
- base-to-lidar transform;
- each ray's interpolated wheel pose;
- each ray's visual-residual alpha;
- raw lidar-frame endpoint and hit flag.

`MapRebuilder` reconstructs the geometry from the corrected frame poses and preserved ray metadata during a full graph rebuild. It rejects the full rebuild rather than silently counting an archived bracketed scan whose frame metadata cannot be reconstructed.

### LOST and recovery behavior

- Scans whose timestamps fall in a tracked LOST interval remain wheel-only provisional scans.
- A scan received while the mapper observed `was_lost_` is explicitly kept provisional even if delayed tracking callbacks never created a timestamped interval. It cannot become an unrelated committed scan after recovery.
- A closed LOST interval is found by sensor timestamp, not callback arrival order.
- On recovery, LOST scans remain provisional while unresolved; corrected geometry is committed only after a successful recovery rebuild publishes.

### Queue timeout and stability behavior

- Missing wheel coverage expires only after wheel sensor time passes the scan end by `pending_scan_timeout_s`.
- Missing visual brackets expire only after visual sensor time passes the scan end by the same timeout. Advancing wheel time alone does not discard a potentially valid visual bracket.
- The queue is bounded by `pending_scan_limit`; dropping the oldest queue entry increments the timeout/drop diagnostic.
- `MapperNode` now explicitly destroys `MapRebuilder` before its publishers and other node members. This prevents the map-rebuilder worker callback from publishing through destroyed node state during teardown.

## Default Parameters to Tune Carefully

```text
max_scan_yaw_change_rad = 0.005       # approximately 0.286 degrees per scan
visual_anchor_max_gap_ms = 200.0
pending_scan_timeout_s = 2.0
pending_scan_limit = 200
wheel_max_gap_ms = 100.0
wheel_retention_s = 300.0
```

The yaw threshold is intentionally conservative. A roughly 99 ms scan at 0.5–1.0 rad/s turns through approximately 0.05–0.10 rad, so it is rejected by the 0.005 rad default. Increase it only after reviewing actual map effects; doing so weakens the protection against double walls.

Timing parameters are validated for finite values and safe nanosecond conversion.

### Thin-wall occupancy (P0/P1)

Occupancy insert policy was retuned to reduce forward wall thickening. Live and
rebuild share the same `GridConfig`. Defaults on the mapper node:

```text
hit_range_max_m = 10.0     # max range for painting occupied hits
hit_log_odds = 0.55        # softer hit (was 0.85)
miss_log_odds = -0.50      # slightly stronger free (was -0.40)
usable_range_m = 20.0      # miss/clear range (node default; GridConfig alone is 12.0)
```

Hits beyond `hit_range_max_m` are not marked occupied; free-space clearing still
uses `usable_range_m` for normal rays (beyond-hit-cap finite hits clear only to
the hit cap). Diagnostics on `/diagnostics` include `hits_applied` and
`hits_range_skipped`.

### Live IMU deskew (P2)

Handsfree ~200 Hz `/imu` is fused into **live** scan deskew only (relative
motion inside a sweep). ORB remains the sole global pose authority; no
pose-moving ICP.

```text
imu_topic = /imu
enable_imu_deskew = true
imu_retention_s = 300.0     # match wheel_retention_s
imu_max_gap_ms = 20.0       # full-sweep coverage required for fusion
```

Fusion rule when IMU covers the full sweep: wheel **xy** + IMU-integrated
**Δyaw** from scan start to each ray stamp; bracketed residual^α still
uses ORB start/end anchors (residual end uses fused motion so α=1 lands
on ORB `end_map`). Mapper waits for IMU coverage up to
`pending_scan_timeout` when IMU samples have been seen; otherwise falls
back. If coverage fails (gap > 20 ms or missing samples after timeout),
deskew **falls back to wheel-only** and increments
`imu_deskew_fallbacks` — the scan is not rejected for IMU alone.

Diagnostics on `/diagnostics` also include `imu_samples`,
`imu_deskew_fallbacks`, and `enable_imu_deskew` (0/1).

Archive/rebuild still stores per-ray **wheel** poses this phase (P3 would
archive IMU for rebuild). Plan:
`docs/superpowers/plans/2026-07-17-orb-lidar-imu-deskew-p2.md`. Design:
`docs/superpowers/specs/2026-07-17-orb-lidar-thin-walls-follow-orb-design.md`.

**Bag replay (P2):** `forward-and-back-origin` via
`tools/run_full_stack_dashboard.sh` (domain 94, rate 2.0) wrote
`tools/full-stack-report/forward-and-back-origin-imu-deskew/` (gitignored).
End-of-run diagnostics: `imu_samples≈16842`, `imu_deskew_fallbacks≈679`,
`enable_imu_deskew=1`, `scans_committed≈893`, map free/occupied
≈79183/4624. Loop-closure gate still fails on this bag (0 loops; expected
for short out-and-back). Compare wall thickness on the dashboard/map PNGs
vs pre-IMU runs.

## Validation Completed

The mapper was built and tested after the implementation and again after final-review fixes.

- AddressSanitizer mapper integration suite: passed.
- Normal `RelWithDebInfo` build: passed.
- Full `orb_lidar_mapper` package suite: **236 tests, 0 errors, 0 failures, 0 skipped**.
- The order-dependent `MapperNodeTest.CorrectedPathPosesCarryPerScanStamps` crash was fixed by the explicit worker shutdown described above.
- `git diff --check -- orb_lidar_mapper`: passed.

The new mapper changes are concentrated in:

```text
orb_lidar_mapper/include/orb_lidar_mapper/
  map_rebuilder.hpp
  mapper_node.hpp
  scan_deskewer.hpp
  timed_pose_buffer.hpp
  trajectory_store.hpp
orb_lidar_mapper/src/
  map_rebuilder.cpp
  mapper_node.cpp
  scan_deskewer.cpp
  timed_pose_buffer.cpp
  trajectory_store.cpp
```

The existing mapper test files appear modified in the working tree. Do not assume all test-file changes were introduced by this work; preserve unrelated changes and inspect diffs before staging anything.

## Verified Bag Replay and Custom Dashboard

The immutable bag is:

```text
/home/duc/robot/20260713_152907
```

It is an MCAP bag of approximately 221.15 seconds and provides stereo images, `/odom_wheel`, `/scan_origin`, and static TF.

A fresh hardening replay used a clean ROS domain and wrote artifacts to:

```text
/home/duc/robot/src/orb_slam3_ros2/artifacts/direct-dashboard-hardening-run-20260716
```

Current full-suite verification is 383 tests with zero errors, failures, or
skips. At the earlier shutdown-regression replay milestone, all 11 checker
gates passed. Stereo pairing was
6633/6633 (ratio 1.0); tracking initialized with OK ratio 1.0, loop count 1,
zero invalid poses, and no deadlock. The final map was 512x448 at
0.05000000074505806 m resolution, with 80744 free and 5830 occupied cells.
`invalid_tf_committed` and `wheel_only_before_recovery` were both zero. Direct
runtime samples were 28.7 Hz and 30.3 Hz before graph correction; graph
revision 4 was reached and the corrected path populated, with one instantaneous
12.7 Hz sample during correction. Foxglove was absent.

The dashboard was launched with `dashboard_host:=0.0.0.0`, HTTP port `51871`,
and the Tailscale URL `http://100.102.92.45:51871/`. Human visual signoff of
the map remains pending and must not be inferred from machine acceptance.

The replay launch automatically shuts down its nodes after bag playback exits. Start a new replay for a new viewing session; do not run overlapping bag players in the same ROS domain.

## How to Run the Stack and Dashboard

### Prerequisites

From the independent implementation repository:

```bash
cd /home/duc/robot
nix develop
cd src/orb_slam3_ros2
source install/setup.bash
```

Use a fresh ROS domain for each independent replay. Avoid domains already used by another bag player or a live robot.

### Full bag replay with the custom dashboard

For the active Tailscale address on this machine:

```bash
ROS_DOMAIN_ID=83 ros2 launch orb_slam_bringup bag_replay.launch.py \
  bag_path:=/home/duc/robot/20260713_152907 \
  artifact_dir:=$PWD/artifacts/run-next \
  rate:=1.0 \
  ros_domain_id:=83 \
  start_dashboard:=true \
  dashboard_host:=0.0.0.0
```

Open from a Tailscale-connected machine:

```text
http://100.102.92.45:51871/
```

For local-only browsing, bind the dashboard to loopback and open the local URL:

```bash
ROS_DOMAIN_ID=43 ros2 launch orb_slam_bringup bag_replay.launch.py \
  bag_path:=/home/duc/robot/20260713_152907 \
  artifact_dir:=$PWD/artifacts/live-local \
  rate:=1.0 \
  ros_domain_id:=43 \
  start_dashboard:=true \
  dashboard_host:=127.0.0.1
```

```text
http://127.0.0.1:51871/
```

### Important dashboard clarification

The currently verified launch path is the custom, read-only in-graph HTTP dashboard server:

- HTTP port: `51871`
- state endpoint: `/state`
- map rendering: served directly by the custom dashboard
- Foxglove is disabled/ignored for now. The bridge reduced observed ORB tracking from approximately 30 Hz direct to approximately 5 Hz.

Use only the plain HTTP URL above; there is no `?ws=` query.

RPLidar NaN/infinity sectors are intentionally ignored as neither clearing nor
occupied evidence. LOST scans are provisional until recovery succeeds; after a
successful rebuild, corrected geometry is committed. The dashboard shows a
revisioned corrected path and only a current-LOST-interval yellow fallback;
successful coherent recovery publication clears that fallback. Outputs remain
candidate-only (`orb_map`, `/orb_lidar/*`, `/orb_slam3/*`), with no canonical
`/map` or `map->odom`; `tf_static` is trusted and this package is independent
of `tasteRobot2`.

### Replay without the dashboard

```bash
ROS_DOMAIN_ID=44 ros2 launch orb_slam_bringup bag_replay.launch.py \
  bag_path:=/home/duc/robot/20260713_152907 \
  artifact_dir:=$PWD/artifacts/run-next \
  rate:=1.0 \
  ros_domain_id:=44 \
  start_dashboard:=false
```

### Quick health checks while replay is active

```bash
source install/setup.bash
export ROS_DOMAIN_ID=83
ros2 topic list | grep -E '^/(clock|orb_lidar|orb_slam3)'
curl --fail --silent http://100.102.92.45:51871/state
```

Expected topics include `/orb_slam3/tracked_frame`, `/orb_slam3/graph_snapshot`, `/orb_lidar/map`, `/orb_lidar/map_revision`, `/orb_lidar/corrected_path`, and `/orb_lidar/provisional_scan`.

## Repository and Safety Notes

- Work only in `/home/duc/robot/src/orb_slam3_ros2`; the top-level `/home/duc/robot` tree is intentionally dirty.
- Do not use broad `git reset`, `git clean`, or broad staging. The working tree has unrelated pre-existing wrapper/dashboard modifications and untracked dashboard files.
- Do not modify the nested `orb_slam3_vendor/vendor/ORB_SLAM3` pin unless the user explicitly requests upstream changes.
- Do not commit unless the user explicitly asks.
- `artifacts/` is generated output and must not be committed.
- Do not add IMU deskew integration without a separate user request.

## Suggested Next Work

The requested conservative mapping implementation is complete. The next useful activity is operator evaluation rather than a redesign:

1. Complete the pending human visual signoff using the direct Tailscale URL and inspect the immutable hardening artifact.
2. If walls still show meaningful duplication, adjust `max_scan_yaw_change_rad` only in controlled replay experiments and compare the resulting artifacts.
3. If a code review or merge is wanted, review only the intended mapper files, preserve unrelated working-tree changes, then create a commit only with explicit user approval.
