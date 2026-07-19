# Circle-Run Loop-Closure Observability Design

## Goal

Determine whether ORB-SLAM3 closes the `/home/duc/robot/bag/circle-run` trajectory and, when it does not, identify whether the failure is in core loop detection/correction, wrapper event synthesis, or the downstream map rebuild.

## Scope

This work adds read-only loop-closure evidence and a repeatable three-run circle-bag evaluation. It does not alter ORB-SLAM3 matching thresholds, change loop-closure logic, or apply an upstream patch. Those changes require evidence produced by this design.

## Current State

The wrapper infers `LOOP_CLOSED` from a new graph loop edge between periodic `GraphSnapshot` values. The event does not identify the edge or distinguish same-map loop closure from map merge. The report requires at least one event and a later published map rebuild, but it cannot explain why a loop event is absent.

`circle-run` is a 90.518-second MCAP bag with 2,717 messages on each stereo CameraInfo topic. It must be measured at real-time playback (`rate:=1`) in `benchmark_mode:=off`, because tracking-FPS benchmark modes intentionally change the process topology.

## Design

### Loop Evidence

Extend graph-delta classification to return structured evidence for each newly observed loop edge. Evidence contains the normalized keyframe pair, their map IDs, the active map ID, and a classification:

- `same_map_loop`: both keyframes are in the active map.
- `cross_map_loop`: the edge spans different map IDs.
- `map_merge`: the existing map-lineage transition detects a map connection.

The wrapper publishes the normal `LOOP_CLOSED` event only for new loop edges, preserving existing consumers. Its `detail` field carries a stable, human-readable summary of the edge and classification. It publishes `MAP_MERGED` separately where existing logic detects a merge. Event publication records the graph revision associated with the snapshot.

The metrics recorder continues consuming the existing event topic, but preserves the detail field in `loops` so each artifact has the observed edge and classification.

### Core Runtime Evidence

The circle-run launch must capture standard output from the ORB wrapper into its artifact directory. A post-run evaluator parses this log for core ORB-SLAM3 markers:

- `*Loop detected`
- `BAD LOOP!!!`
- `Local Mapping STOP`
- `Local Mapping RELEASE`
- `*Merge detected`

The evaluator writes `loop_closure_evidence.json` without editing source data. It reports marker counts, whether a stop was left unreleased, the wrapper loop events from `metrics.json`, and a diagnosis:

- `core_loop_unobserved`: core logged a loop but wrapper produced no loop event.
- `loop_rebuild_missing`: wrapper reported a loop but no later published rebuild exists.
- `local_mapping_stop_unreleased`: stop count exceeds release count.
- `no_core_loop_detected`: core emitted neither a loop nor merge marker.
- `observed_and_rebuilt`: wrapper loop evidence exists and every loop has a later published rebuild.

The evaluator may report more than one diagnosis. It must not infer that an absent core log is proof of absence when the log file is unavailable; that condition is reported explicitly as `core_log_missing`.

### Circle-Run Protocol

Add a shell runner that validates the circle bag, launches three independent real-time full-stack runs, and evaluates each result. It uses artifact directories `run-1`, `run-2`, and `run-3` and unique ROS domain IDs. The runner records a top-level `summary.json` containing the per-run evidence path and pass/fail status.

A run passes when all of the following hold:

- ORB initialized and did not report a metrics deadlock.
- Tracking OK ratio after initialization is at least `0.70`.
- `loop_closure_evidence.json` has `observed_and_rebuilt`.
- The evaluator does not report `local_mapping_stop_unreleased`.

The experiment passes when at least two of three runs pass. The runner exits nonzero for an invalid/missing artifact or if fewer than two runs pass.

The stale 6,633-pair acceptance threshold remains outside this experiment. This bag has 2,717 stereo pairs, so the dedicated evaluator uses the bag's measured artifacts rather than the old fixed-bag report gate.

## Upstream Issue Ledger

These items are documented for later, evidence-gated investigation. They are not patched in this scope.

| Upstream item | Risk | Planned evidence gate |
| --- | --- | --- |
| [Issue #911](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/911) | Global bundle adjustment can traverse cyclic keyframe hierarchy and leave local mapping stopped. | Investigate only if `local_mapping_stop_unreleased` appears or a run hangs after a core loop marker. |
| [PR #745](https://github.com/UZ-SLAMLab/ORB_SLAM3/pull/745) | KeyFrameDatabase candidate iteration may fail to advance after a bad keyframe, impairing place recognition. | Compare the exact vendor revision and add a minimal reproduction before considering the unmerged patch. |
| [Issue #394](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/394) | Strict `BAD LOOP!!!` inertial rotation validation rejects loops. | Not applicable to current pure-stereo config unless sensor mode changes. |
| [Issue #718](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/718) | Map merge can create a map-frame pose discontinuity. | Investigate only when evidence classifies a cross-map loop or map merge. |
| [Issue #57](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/57) | A visually obvious path may not have enough viewpoint overlap or geometric inliers to close. | Investigate only after three runs show `no_core_loop_detected`; test controlled feature-extractor variants before core changes. |

## Constraints

- Preserve normal launch behavior and the existing `TrackingEvent` message schema.
- Do not modify ORB-SLAM3 vendor source in this phase.
- Do not relax loop matching/rotation thresholds in this phase.
- Keep each run's artifacts self-contained and do not overwrite prior results.
- Tests use fixture text and synthetic metrics; unit tests do not replay the 1.6 GiB bag.

## Verification

- Unit tests cover graph-edge evidence classification, core-log parsing, diagnosis rules, and two-of-three summary decisions.
- Existing wrapper graph-semantic tests continue to prove legacy event types.
- The `orb_slam_bringup` Python test suite and relevant wrapper C++ tests pass.
- The three-run runner is executed against `/home/duc/robot/bag/circle-run`; its generated summary is the empirical result, not a prerequisite for unit-test success.
