# ORB-SLAM3 ROS 2 Handoff

## Current State

- Workspace: `/home/duc/robot/src/orb_slam3_ros2`
- Branch: `feature/orb-lidar-thin-walls-p0-p1`
- Latest commit: `b3bbdd3 docs: record graph observation evaluation`
- Vendor submodule: `orb_slam3_vendor/vendor/ORB_SLAM3` at `601bec2`
- The end-to-end circle loop-closure acceptance gate is still failing. Do not
  report loop closure as fixed.

## Objective

Make the loop-closure graph evidence emitted by ORB-SLAM3 observable to the
wrapper and trigger the downstream rebuild. The required acceptance gate is at
least two `observed_and_rebuilt` outcomes across three rate-one replays of
`/home/duc/robot/bag/circle-run`.

## Completed Work

### Observability Tooling

The wrapper and bringup tooling now capture graph-semantic evidence and
evaluate it independently of unrelated tracking health:

- `67a7d08 feat: enrich loop closure graph evidence`
- `bfd9aad feat: add loop closure evidence evaluator`
- `ad8f61d fix: make diagnoses independent and use atomic file writing`
- `ffb28b4` and `b0a8d73`: reliable wrapper log capture in bag replay.
- `6609188 feat: add circle loop closure evaluation runner`

Key files:

- `orb_slam3_wrapper/src/wrapper_node.cpp`
- `orb_slam3_wrapper/src/graph_semantics.cpp`
- `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py`
- `tools/run_circle_loop_closure_evaluation.sh`

The evaluator diagnoses `core_loop_unobserved` when ORB-SLAM3 reports a loop
marker but the metrics `loops` array is empty. A qualifying result also needs
a published rebuild at or after the loop graph revision.

### Vendor Notification Ordering

`dacfb81 fix: notify graph change after loop edge insertion` includes vendor
commit `601bec2 fix: notify graph change after loop edge insertion`.

In `LoopClosing::CorrectLoop()`, the single
`mpAtlas->InformNewBigChange()` now follows both reciprocal calls:

```cpp
mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);
mpAtlas->InformNewBigChange();
```

It still precedes `mpLocalMapper->Release()`. A focused System-level vendor
regression was added at:

- `orb_slam3_vendor/test/loop_closing_notification_test.cpp`
- `orb_slam3_vendor/CMakeLists.txt`

The test uses the public `System::MapChanged()` and `System::GetGraphSnapshot()`
path and is green. Do not revert or duplicate this notification.

### Independent Wrapper Graph Observation

`ae309ca fix: poll graph changes independently of frames` moved graph-change
consumption out of `WrapperNode::processStereo()` into a 50 ms wall timer.

Key behavior:

- `graph_timer_` calls `pollGraphChanges()`.
- The timer is the sole consumer of `SlamBackend::mapChanged()`.
- It returns before backend configuration or before a tracked-frame header
  exists, so it does not consume an unusable change.
- It calls `graphSnapshot()` only after `mapChanged()` returns true.
- It routes a new revision through the existing `publishGraph()` semantic
  event path using `last_tracked_.header`.
- It uses the node's default mutually exclusive callback group. No additional
  threads, locks, or callback groups were added.

The new component regressions spin the node after the final stereo callback
and prove a later graph snapshot can publish canonical loop edges and a
`LOOP_CLOSED` event:

- `orb_slam3_wrapper/test/wrapper_component_test.cpp`

The relevant approved design and plan are:

- `docs/superpowers/specs/2026-07-19-wrapper-independent-graph-observation-design.md`
- `docs/superpowers/plans/2026-07-19-wrapper-independent-graph-observation.md`

## Verification Already Run

For `ae309ca`:

```bash
colcon build --packages-select orb_slam3_wrapper
colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R 'wrapper_component_test|graph_semantics_test' \
  --output-on-failure
pytest orb_slam_bringup/test/test_loop_closure_evidence.py -v
```

Results:

- `wrapper_component_test`: 11 passed.
- `graph_semantics_test`: 7 passed.
- `test_loop_closure_evidence.py`: 10 passed.
- Task reviews and whole-branch review approved with no findings.

## Acceptance Evidence

### Baseline Before Vendor Ordering Fix

- Commit: `6609188`
- Artifacts: `artifacts/circle-loop-evaluation-20260719_160436`
- Result: `0/3`
- All trials logged `*Loop detected`, no `BAD LOOP!!!`, balanced local mapping
  STOP/RELEASE counts, and `core_loop_unobserved`.

### After Vendor Ordering Fix

- Artifacts: `artifacts/circle-loop-evaluation-20260719_204100`
- Result: `0/3`
- Graph revisions `0`, `1`, and `2` were observed, but no loop-edge event was
  emitted.

### After Independent Wrapper Timer

- Superproject code commit evaluated: `ae309ca`
- Artifacts: `artifacts/circle-loop-evaluation-20260719_215238`
- Result: `0/3`, required `2/3`
- `summary.json` has `passed_runs: 0` and `passed: false`.
- Every `run-*/loop_closure_evidence.json` says:

```json
{
  "diagnoses": ["core_loop_unobserved"],
  "passed": false
}
```

- Each run logged `*Loop detected`, no `BAD LOOP!!!`, and balanced Local
  Mapping STOP/RELEASE markers.
- No canonical loop edge was extracted/published and no downstream rebuild
  qualified under the evaluator.

The acceptance result is committed in:

- `b3bbdd3 docs: record graph observation evaluation`
- `docs/superpowers/specs/2026-07-19-wrapper-independent-graph-observation-design.md`

## Root-Cause Boundaries

Proven facts:

- Loop candidate detection executes consistently on the circle bag.
- The vendor notification runs after loop-edge insertion in the source and the
  focused System test verifies that ordering contract.
- The wrapper timer independently polls changes and unit tests prove it can
  publish an asynchronously supplied graph snapshot with a loop edge.
- Real bag runs still yield no observable loop-edge delta, even after the
  notification and timer changes.

Not yet proven:

- Whether `CorrectLoop()` actually completes the reciprocal `AddLoopEdge()`
  calls in the real circle-run execution before shutdown.
- Whether the map snapshot contains the reciprocal loop edges when the real
  notification is observed.
- Whether a later graph operation overwrites/removes those loop edges before
  the wrapper captures the snapshot.
- Whether the expected end-to-end loop signal should be represented by a
  different vendor graph mutation for this ORB-SLAM3 path.

Do not apply another speculative fix. Resume with root-cause tracing and add
diagnostic evidence at the vendor-to-wrapper boundary first.

## Recommended Next Investigation

1. Inspect real graph snapshots at every observed revision, including keyframe
   IDs and `loop_edge_ids`, rather than only the final wrapper event metrics.
2. Instrument or test the actual `LoopClosing::CorrectLoop()` concurrent path
   to establish whether it reaches both `AddLoopEdge()` calls during circle
   replay, and whether `Map::GetGraphSnapshotData()` contains them afterward.
3. Correlate `*Loop detected`, `CorrectLoop()` completion, big-change index,
   timer polling, snapshot revision, and extracted edges with timestamps.
4. Only after that evidence, decide whether the defect is a vendor lifecycle,
   snapshot representation, wrapper semantics, or evaluator expectation.

Use the systematic-debugging and brainstorming workflows before any new
production change. User explicitly requested subagent implementation and an
independent review after each task; retry empty/failed subagent work up to
three times before pausing with the blocker.

## Prepared Wrapper-Baseline Retry Plan (Not Implemented)

Later source/artifact tracing identified a high-confidence wrapper-side
hypothesis that supersedes parameter tuning as the next experiment, but it
still needs the prescribed regression and runtime validation before claiming
the end-to-end gate is fixed:

- `System::MapChanged()` stays false while the atlas big-change index is zero
  (`orb_slam3_vendor/vendor/ORB_SLAM3/src/System.cc`). The wrapper therefore
  does not capture a pre-loop graph baseline.
- `CorrectLoop()` adds reciprocal loop edges and then signals the big change
  (`orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc`).
- On the first changed snapshot, `previous_graph_` is empty. The semantic
  classifier only finds loop-edge deltas when a previous snapshot exists
  (`orb_slam3_wrapper/src/graph_semantics.cpp`). This plausibly drops the
  first observable loop edge permanently even though revisions 1 and 2 drive
  downstream map rebuilds.
- Circle-run visual tracking is not currently the bottleneck: all latest runs
  logged `*Loop detected`; run-1 metrics report initialized tracking, no
  losses, and `ok_ratio_after_init: 1.0`. Do not tune ORB parameters or alter
  vendor ORB-SLAM3 for this acceptance failure first. The roughly 41% stereo
  pairing ratio is a separate future robustness investigation.

The complete TDD plan is at the currently untracked file:

- `docs/superpowers/plans/2026-07-19-wrapper-graph-baseline.md`

Its sole task is deliberately limited to:

1. Add `FirstChangedGraphWithLoopEdgeEmitsLoopClosed` in
   `orb_slam3_wrapper/test/wrapper_component_test.cpp`. The fake backend must
   first be spun with `changed=false`, proving a baseline is not published;
   then revision 1 introduces reciprocal loop edges and must emit exactly one
   `LOOP_CLOSED` event.
2. Run the focused component test before production code and observe the RED
   failure.
3. Add `graph_baseline_captured_{false}` in
   `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`. In
   `WrapperNode::pollGraphChanges()`, after existing readiness checks but
   before the timer's sole `mapChanged()` call, capture exactly one
   `previous_graph_ = backend_->graphSnapshot()` baseline. It must not call
   `publishGraph()`, publish events, or update `last_graph_revision_`.
4. Run `colcon build --packages-select orb_slam3_wrapper`, then the focused
   `wrapper_component_test`, then the
   `wrapper_component_test|graph_semantics_test` CTest regex. Run
   `git diff --check` before committing only the three wrapper files.
5. After task review, run a rate-one circle replay with boundary diagnostics
   (snapshot revision, raw canonical loop-edge count, previous-baseline
   presence, and extracted delta count) before claiming the 2/3 acceptance
   gate passes.

### Subagent Execution Blocker

No implementation was attempted by the main agent. Multiple worker/planner
subagent dispatches were made for the exact plan above. The first misrouted to
an unrelated home-directory task; all later retries returned empty and made no
repository edits, builds, tests, or commits. The task report at
`.superpowers/sdd/task-1-report.md` contains only a heading. Before retrying,
verify the subagent harness can actually execute shell/file-edit work. Once it
can, use a fresh implementer, independently review the task diff, and retain
all untracked scratch files.

## Important Files

- `orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc`
- `orb_slam3_vendor/vendor/ORB_SLAM3/src/System.cc`
- `orb_slam3_vendor/vendor/ORB_SLAM3/src/Map.cc`
- `orb_slam3_vendor/test/loop_closing_notification_test.cpp`
- `orb_slam3_wrapper/src/wrapper_node.cpp`
- `orb_slam3_wrapper/src/graph_semantics.cpp`
- `orb_slam3_wrapper/test/wrapper_component_test.cpp`
- `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py`
- `tools/run_circle_loop_closure_evaluation.sh`
- `.git/sdd/progress.md`

## Workspace Hygiene

The current branch has unrelated untracked scratch files. Do not delete,
modify, stage, or commit them. `git status --short` presently includes files
such as `new_test.pyncat`, `result_table.md`, `tf_audit.json`,
`tracking_benchmark.json`, and multiple `tools/check_select*` and
`tools/eval_rates*` scripts. Generated `artifacts/` directories are also not
to be committed.
