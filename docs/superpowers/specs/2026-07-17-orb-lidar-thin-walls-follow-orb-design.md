# ORB Lidar Thin Walls (Follow ORB) Design

**Date:** 2026-07-17  
**Repository:** `/home/duc/robot/src/orb_slam3_ros2`  
**Package:** `orb_lidar_mapper`  
**Status:** Design approved in session; implementation not started

## Purpose

Keep the product rule that **ORB-SLAM3 owns global pose in `orb_map`** and lidar
only paints geometry in that frame, while making walls **stay thin** on long
pure-forward runs (today: occupied band grows from ~2 pixels toward 6+ over
distance/time).

This is a different failure mode from rotate “two parallel walls”:

| Symptom | Typical cause |
|---|---|
| Forward: one wall **thickens** over time | Lateral pose jitter + sticky hit log-odds |
| Rotate: **two** distinct walls | Lever arm / yaw / committing under spin |

The design improves **occupancy insert policy** first, then **intra-scan deskew**
(Handsfree IMU + wheel) under ORB anchors, and keeps archive + graph rebuild.
It does **not** add online ICP that moves committed poses.

## Problem Statement

### Observed

- Pure forward: wall starts as a thin line (~2 px) and **keeps growing thicker**
  (3, 4, 5, 6… cells). It is **not** a clean second parallel wall like rotate.
- Rotate double-wall remains a separate issue (turn gate + extrinsic); this
  design must not regress it.

### Why current deskew is not enough for thickness

Accepted straight scans already deskew with wheel SE(2) and a visual residual
between two ORB anchors (`ScanDeskewer::deskewBracketed`). On pure forward at
~0.5 m/s, the lidar moves only ~5 cm during a ~99 ms sweep. Intra-scan smear is
millimetre-scale for modest wheel error. Unbounded thickening is dominated by:

1. **Inter-scan** placement jitter (ORB noise/scale, residual bridge, extrinsic).
2. **Occupancy hardening**: hit `+0.85` vs miss `−0.40` so wrong or offset hits
   stick while later hits paint adjacent cells.
3. **Long hit range** (usable ~20 m): small angular error → large lateral offset.

IMU deskew still helps residual motion blur; it is **not** the primary fix for
slow forward thickening.

### Related current behavior (baseline)

- Commit only when wheel covers the full sweep, peak-to-peak wheel yaw excursion
  ≤ `max_scan_yaw_change_rad` (default **0.005 rad**), and two compatible ORB
  frames bracket the sweep.
- No-return / NaN / ±inf beams are ignored (no free-space clearing rays).
- Finite hits still cast free-space miss along the ray, then hit at the endpoint
  (`TiledOccupancyGrid::insert`).
- Live path: `MapRebuilder::appendCommitted`. Full rebuild on graph revision
  re-projects archived rays with corrected ORB keyframe poses.
- Live mapping deskew does **not** use `/imu` today (offline calib does).

## Goals

1. On long pure-forward runs past a planar wall, occupied **thickness plateaus**
   (target ~1–3 cells at 5 cm resolution), not unbounded growth.
2. Lidar map **follows ORB**: graph revision / loop rebuild still moves geometry.
3. No competing lidar global trajectory; no pose-moving online ICP.
4. No regression on turn rejection and no-return beam policy.
5. Phased delivery: occupancy first, IMU deskew next, optional extras later.

## Non-Goals

- Online ICP / scan-match that **moves** the committed scan pose in `orb_map`.
- Lidar odometry or a second map frame.
- IMU-only global localization.
- Auto-edit of TF, URDF, or bags from the mapper.
- Replacing ORB graph rebuild with filter/IMU history as truth.
- Re-enabling free-space rays from RPLidar no-return (NaN/inf) beams.
- Solving stereo/lidar lever-arm calibration (separate tools/specs).
- Raising the yaw gate to map while spinning (only after metrics, later phase).

## Authority Model (Section 1)

**Product rule (frozen):**

> ORB-SLAM3 is the only source of global pose in `orb_map`.  
> Lidar never optimizes a competing trajectory.  
> The map is “ORB pose × deskewed rays × occupancy model.”

| Source | Owns | Does not own |
|---|---|---|
| ORB-SLAM3 | Absolute base pose, graph revisions, loop corrections | Intra-scan continuous motion |
| Handsfree IMU (~200 Hz) | High-rate **relative** motion (esp. yaw) for deskew / bridge | Map frame |
| Wheel odom (`/odom_wheel`) | Translation / SE(2) backup, coverage and yaw-excursion gates | Absolute map pose |
| Occupancy grid | Evidence accumulation under ORB poses | Pose estimation |
| Archive + rebuild | Sensor rays + relative motion metadata; re-paint after ORB changes | Independent lidar SLAM |

```text
ORB tracked frames / graph
        │
        ▼
  pose bridge (wheel + residual^α; later optional IMU-aided relative)
        │
        ▼
  per-ray deskew (wheel today; later IMU+wheel)
        │
        ├─► gate: commit or drop
        │
        ├─► LIVE insert (ORB pose only, improved occupancy)
        │
        └─► ARCHIVE (rays + relative motion metadata)
                    │
                    ▼ graph_revision
              REBUILD = re-deskew + re-insert with corrected ORB
```

### ICP policy (frozen)

| Allowed | Not allowed |
|---|---|
| Offline rotation-center calibration ICP (existing) | Online ICP that moves scan pose for the map |
| Optional later: residual **metric only** or scan **reject** (no pose shift) | Lidar odometry as map frame |

## Occupancy / Insert Policy (Section 2)

Thickness control happens at **insert time**. Pose stays pure ORB (bridge + deskew).

Live and rebuild **share the same insert rules** so corrected maps do not re-fatten
with a more aggressive policy than live.

### Phase 2a — Range cap + log-odds retune (first implementation)

| Parameter | Today (approx.) | Direction |
|---|---|---|
| Hit max range | usable ~20 m | Cap **hits** at `hit_range_max_m` (default in **8–12 m** band; exact default chosen in plan/tests) |
| Beyond hit cap | painted | Ignore ray for hit (and document whether miss stops at same cap) |
| `hit_log_odds` | +0.85 | Lower (e.g. **+0.45 … +0.60**) so one scan cannot own a cell |
| `miss_log_odds` | −0.40 | Slightly stronger free (e.g. **−0.45 … −0.55**) so revisits can carve fat |
| clamp / thresholds | existing | Keep unless tests show need |

**Invariant:** still log-odds occupancy with free miss along the path to a finite
hit; not a hit-only product map.

**No-return beams:** remain fully ignored (neither hit nor miss).

### Phase 2b — Consistency gate (optional)

Before applying a **hit** update (pose unchanged):

- If the local map has **no** established occupied structure → **always accept**
  (bootstrap / new area).
- If the map already has a strong local wall and the endpoint is an **outlier**
  beyond `consistency_tol_m` from that structure → **skip hit only**
  (optional: still apply miss along the ray).

This is **not** ICP: the scan pose is never shifted.

### Phase 2c — Occupied decay (out of default scope)

Mild time decay of occupied cells is **not** required for the first delivery.
Only revisit if 2a/2b are insufficient and static map erasure risk is accepted.

### Archive vs live

| Layer | Stores / applies |
|---|---|
| Archive | Full finite rays + motion metadata (sensor truth); not “thinned” geometry |
| Live insert | Range cap + log-odds + optional consistency gate |
| Rebuild | Same insert policy on re-projected rays |

### Diagnostics (with 2a)

Publish or log counters such as:

- `hits_applied`
- `hits_range_skipped`
- `hits_consistency_skipped` (when 2b enabled)
- existing commit / turn-reject counters remain

## Deskew / IMU / Bridge (Section 3)

### Roles

| Mode | Relative motion | Absolute constraint |
|---|---|---|
| Committed (tracked) | Wheel xy + **IMU-integrated Δyaw** over ray stamps (phase 3a); optional filter later | ORB start/end residual^α |
| LOST / provisional | Wheel + IMU relative only | Not committed until recovery rules |
| Rebuild | Re-run same relative model from archive | Corrected ORB keyframe poses |

Endpoint residual form **stays**:

```text
predicted(t) from relative motion
residual = inv(predicted_end) * end_map_orb
pose(t) = predicted(t) * residual.pow(α)
```

ORB remains authority at the anchors; IMU/wheel only fill the short gap.

### Phase 3a — Live IMU deskew

- Subscribe Handsfree `/imu` (parametrized topic; default `/imu`).
- Integrate `angular_velocity.z` (trapezoid) over ray timestamps; fuse with wheel
  translation for base SE(2) relative motion.
- Reject scan if IMU gap exceeds threshold (e.g. **20 ms**, same spirit as offline
  calib) or stamps are unusable.
- Handle contiguous duplicate IMU stamps (mean), consistent with bag reader fixes.
- Mount still from live TF `base_link → scan frame` at scan time.

**Not:** integrate IMU into a free global pose in `orb_map`.

### Phase 3b — Archive relative motion for rebuild

Extend archive so rebuild can re-deskew with the same model:

- Today: per-ray wheel pose + α + `base_to_lidar` + raw hit.
- Later: IMU samples or an equivalent relative chain over
  `[scan_start, scan_end]`.

Do **not** archive filter world poses as truth.

### Phase 3c — Optional bridge filter

ESKF/Kalman fusing IMU + wheel between two ORB anchors may replace linear
relative interpolation **inside** the residual construction. Filter state is
disposable on graph rebuild.

### Phase 3d — Yaw gate

Keep `max_scan_yaw_change_rad = 0.005` until IMU deskew is validated. Raising
the gate is a **metrics-gated** decision only (rotate bags must not grow
double walls).

### Relation to offline calib deskew

Live deskew may **reuse ideas** from `calibration_deskew` (IMU integrate, gap
reject). It must **not** adopt calib’s midpoint-only / ICP pipeline as the live
archive model. Live remains visual-bracket + archive + rebuild shaped.

### What section 3 does and does not fix

| Symptom | IMU deskew | Occupancy (section 2) |
|---|---|---|
| Forward wall **thickens** over time | Mild | **Primary** |
| Intra-scan blur while moving | **Primary** | Secondary |
| Rotate two walls | Helps only if turns are committed | Weak |
| ORB scale / drift | No (honest follow) | No |

## Phased Roadmap (Section 4)

| Phase | Deliverable | Priority |
|---|---|---|
| **P0** | Occupancy 2a: `hit_range_max_m` + hit/miss log-odds retune; live + rebuild | **First code** |
| **P1** | Diagnostics + thickness metric (ROI FWHM / cell count over time) | With P0 |
| **P2** | IMU deskew 3a in bracketed path + unit/synthetic tests | After P0 baseline |
| **P3** | Archive IMU / relative chain for rebuild (3b) | After P2 |
| **P4** | Consistency gate 2b (optional) | If thickness still grows |
| **P5** | Bridge filter 3c (optional) | If P2 insufficient |
| **P6** | Consider higher yaw gate | Metrics only |

Default first implementation slice after this spec: **P0 + P1**.

## Success Criteria

1. **Forward thickness:** on a long pure-forward bag past a flat wall, occupied
   thickness **plateaus** (target ≤ ~2–3 cells at 5 cm), not unbounded growth.
2. **Authority:** after graph revision, rebuild moves walls with corrected ORB
   keyframes (existing rebuild tests remain valid in spirit).
3. **Discovery:** new obstacles in empty map still appear within 1–2 scans
   (bootstrap not blocked by consistency gate).
4. **Openings:** doors/openings still clear under miss updates within a bounded
   number of revisits after softer hit odds.
5. **Rotate regression:** turn reject and no-return ignore policy unchanged;
   rotate double-wall not made worse by P0–P2.
6. **Philosophy:** no online pose-moving ICP; no second global pose stream.

## Testing Strategy

| Level | Coverage |
|---|---|
| Unit | Range-capped insert; log-odds balance; consistency bootstrap; IMU integrate + gap reject; residual endpoints still match ORB anchors |
| Package suite | Existing `orb_lidar_mapper` tests stay green; rebuilder still shifts hits with corrected poses |
| Synthetic | Wall + known lateral jitter → thickness plateaus under P0; IMU reduces endpoint scatter under P2 |
| Bag replay | Forward mapping bag for thickness before/after P0; `inplace-rotate*` for turn-reject regression |
| Diagnostics | Counters for applied vs skipped hits; turn rejected vs committed |

## Risks And Mitigations

| Risk | Mitigation |
|---|---|
| Softer hits mark obstacles slowly | Empty neighborhood always accepts hits; tune hit floor on bag |
| Range cap hides far structure | Parametrize `hit_range_max_m`; document; consider miss policy separately |
| Consistency gate drops thin poles | Bootstrap when map empty; only skip near strong walls; optional miss still runs |
| IMU skew / gaps | Reject scan rather than undeskewed commit; counters |
| Live vs rebuild diverge | Same insert policy; P3 re-deskew model on rebuild |
| Scope creep to lidar SLAM | Frozen ICP non-goal; PR checklist against pose-moving match |

## Key Code Touchpoints (implementation later)

| Area | Paths |
|---|---|
| Insert / grid | `orb_lidar_mapper/src/tiled_occupancy_grid.cpp`, headers |
| Live + rebuild | `orb_lidar_mapper/src/map_rebuilder.cpp`, `mapper_node.cpp` |
| Deskew | `orb_lidar_mapper/src/scan_deskewer.cpp`, `trajectory_store.cpp` |
| Offline IMU reference (ideas only) | `orb_lidar_mapper/src/calibration_deskew.cpp` |
| Operator notes | `handoff-kiro.md`, package README when behavior ships |

## Open Decisions For Implementation Plan

These are intentionally left to the plan/PR, not blocked on re-brainstorm:

1. Exact default numbers: `hit_range_max_m`, `hit_log_odds`, `miss_log_odds`.
2. Whether miss clearing uses the same range cap as hits.
3. IMU topic name / frame and whether D435i IMU is ever a fallback (default: Handsfree `/imu` only).
4. Thickness ROI metric definition for automated bag checks.

## References

- Current mapper behavior: `handoff-kiro.md` (straight-scan commit, bracketed deskew, rebuild).
- Offline IMU deskew (not live): `docs/superpowers/specs/2026-07-16-lidar-rotation-center-calibration-design.md`.
- Prior hardening: no-return free-ray disable + turn gate (`48eb3b2` era mapper work).
