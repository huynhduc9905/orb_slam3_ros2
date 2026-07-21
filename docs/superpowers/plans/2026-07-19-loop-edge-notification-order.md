# Loop-Edge Notification Order Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ensure the graph-change notification consumed by the ROS wrapper occurs only after ORB-SLAM3 adds a completed loop edge, so circle-run loop closure is observable and rebuildable.

**Architecture:** `LoopClosing::CorrectLoop()` currently increments the Atlas big-change index before adding the two symmetric loop edges. Move that existing notification immediately after both `AddLoopEdge()` calls, preserving one notification per completed correction. A vendor regression test verifies that a snapshot consumed after the notification includes both edges; the existing wrapper/evidence pipeline and three-run evaluator then validate the behavior on `circle-run`.

**Tech Stack:** ORB-SLAM3 vendor C++17, gtest/ctest, ROS 2 Kilted, Python pytest, bash, existing circle-loop evaluator.

## Global Constraints

- Modify only the notification ordering in `LoopClosing::CorrectLoop()`; do not alter loop matching, geometric thresholds, GBA behavior, map merge behavior, or feature extraction.
- Keep exactly one `mpAtlas->InformNewBigChange()` for each successful same-map loop correction.
- The notification must occur after both symmetric `AddLoopEdge()` calls and before `mpLocalMapper->Release()`.
- Do not apply upstream issues #911, #745, #394, #718, or #57 in this change.
- Write the vendor regression test first and observe its failure before moving the production line.
- Generated bag artifacts, logs, maps, and reports remain untracked.
- Success on `circle-run` is at least two passing runs out of three according to `loop_closure_evidence.json`.

---

## File Structure

- Modify `orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc`: move the one existing completed-loop change notification.
- Modify or create the focused vendor loop-closing test under `orb_slam3_vendor/test/`: prove notification order exposes symmetric loop edges in the next graph snapshot.
- Modify `docs/superpowers/specs/2026-07-19-circle-run-loop-closure-observability-design.md`: append the measured root cause and the intentional scope boundary.
- Generated only: `artifacts/circle-loop-evaluation-*/`: three empirical replay results, not committed.

### Task 1: Add a vendor notification-order regression test

**Files:**
- Modify: `orb_slam3_vendor/test/CMakeLists.txt` if the focused test needs registration.
- Modify or Create: `orb_slam3_vendor/test/loop_closing_notification_test.cpp`

**Interfaces:**
- Consumes: `ORB_SLAM3::Map`, `ORB_SLAM3::Atlas`, `ORB_SLAM3::GraphSnapshot`, and the existing snapshot/map-change test seams.
- Produces: a gtest that fails when the graph-change notification is consumed before the two symmetric loop edges are added.

- [ ] **Step 1: Locate an existing vendor test fixture that can create a map, two keyframes, and a graph snapshot**

Read all of `orb_slam3_vendor/test/system_shutdown_test.cpp`, `orb_slam3_vendor/test/CMakeLists.txt`, and any existing snapshot test. Reuse existing vendor test hooks; do not add a public production test-only API unless no current hook can express the ordering assertion.

- [ ] **Step 2: Write the failing regression test**

The test must model the contract at the `CorrectLoop()` boundary. It must verify these assertions after consuming the single change notification:

```cpp
ASSERT_TRUE(system.MapChanged());
const ORB_SLAM3::GraphSnapshot snapshot = system.GetGraphSnapshot();
ASSERT_EQ(snapshot.revision, 1U);
ASSERT_TRUE(ContainsLoopEdge(snapshot, current_kf_id, matched_kf_id));
ASSERT_TRUE(ContainsLoopEdge(snapshot, matched_kf_id, current_kf_id));
```

`ContainsLoopEdge()` must search the snapshot keyframe with the first ID and assert its `loop_edge_ids` contains the second ID. The fixture must use the same ordering currently present in `CorrectLoop()`: notification first, then both edge additions. The test should therefore fail because the consumed snapshot lacks either edge.

- [ ] **Step 3: Run the focused test to verify red**

Run:

```bash
COLCON_DEFAULTS_FILE=/dev/null colcon build --packages-select orb_slam3_vendor --symlink-install
colcon test --packages-select orb_slam3_vendor \
  --ctest-args -R loop_closing_notification_test --output-on-failure
```

Expected: the new test fails at a `ContainsLoopEdge` assertion, not during fixture setup or compilation.

- [ ] **Step 4: Commit the red test only if repository policy permits failing commits; otherwise leave it unstaged until Task 2**

Do not commit a knowingly failing test unless existing repository history/policy explicitly allows it. Record the observed failure in the task report.

### Task 2: Move the completed-loop notification and verify vendor/wrapper contracts

**Files:**
- Modify: `orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc:1192-1210`
- Modify: `orb_slam3_vendor/test/loop_closing_notification_test.cpp` only if the red test needs fixture cleanup after the minimum production change.
- Modify: `docs/superpowers/specs/2026-07-19-circle-run-loop-closure-observability-design.md`

**Interfaces:**
- Consumes: the red regression test from Task 1.
- Produces: one post-edge `InformNewBigChange()` and a coherent graph snapshot for wrapper event synthesis.

- [ ] **Step 1: Apply the one-line ordering change**

Change this sequence:

```cpp
mpAtlas->InformNewBigChange();

mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);
```

to this exact sequence:

```cpp
mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

mpAtlas->InformNewBigChange();
```

Leave the subsequent GBA thread launch and `mpLocalMapper->Release()` unchanged.

- [ ] **Step 2: Run the focused vendor test to verify green**

Run:

```bash
COLCON_DEFAULTS_FILE=/dev/null colcon build --packages-select orb_slam3_vendor --symlink-install
colcon test --packages-select orb_slam3_vendor \
  --ctest-args -R loop_closing_notification_test --output-on-failure
```

Expected: all tests selected by the regex pass.

- [ ] **Step 3: Run integration contracts**

Run:

```bash
COLCON_DEFAULTS_FILE=/dev/null colcon build \
  --packages-select orb_slam3_vendor orb_slam3_wrapper orb_slam_bringup \
  --symlink-install
pytest orb_slam_bringup/test/test_loop_closure_evidence.py \
  orb_slam_bringup/test/test_bag_profile.py -v
colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R graph_semantics_test --output-on-failure
```

Expected: all commands exit `0`.

- [ ] **Step 4: Update measured-root-cause documentation**

Append this concise statement to the observability design document:

```text
Three real-time circle-run trials at commit 6609188 logged one `*Loop detected`
each, no `BAD LOOP!!!`, and balanced Local Mapping STOP/RELEASE counts, but
recorded zero wrapper loop events. The root cause was the pre-edge
`InformNewBigChange()` ordering in `LoopClosing::CorrectLoop()`; the wrapper
consumed a graph snapshot before `AddLoopEdge()` populated the closure edge.
```

- [ ] **Step 5: Commit**

```bash
git add orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc \
  orb_slam3_vendor/test \
  docs/superpowers/specs/2026-07-19-circle-run-loop-closure-observability-design.md
git commit -m "fix: notify graph change after loop edge insertion"
```

### Task 3: Execute the three-run circle validation and review the branch

**Files:**
- Create: `artifacts/circle-loop-evaluation-YYYYMMDD_HHMMSS/` (untracked)
- Modify: no tracked source files unless the experiment exposes a distinct reproducible defect.

**Interfaces:**
- Consumes: built vendor/wrapper/bringup packages and `/home/duc/robot/bag/circle-run`.
- Produces: three `loop_closure_evidence.json` files and a top-level `summary.json`.

- [ ] **Step 1: Run the evaluator**

Run:

```bash
tools/run_circle_loop_closure_evaluation.sh \
  --bag /home/duc/robot/bag/circle-run \
  --output artifacts/circle-loop-evaluation-$(date +%Y%m%d_%H%M%S) \
  --domain 140
```

Expected: three runs execute at real-time playback and write `summary.json`.

- [ ] **Step 2: Inspect all three evidence files**

Run:

```bash
python3 -m json.tool artifacts/circle-loop-evaluation-*/summary.json
python3 -m json.tool artifacts/circle-loop-evaluation-*/run-1/loop_closure_evidence.json
python3 -m json.tool artifacts/circle-loop-evaluation-*/run-2/loop_closure_evidence.json
python3 -m json.tool artifacts/circle-loop-evaluation-*/run-3/loop_closure_evidence.json
```

Expected passing evidence has `observed_and_rebuilt`, one or more wrapper loop records, no `local_mapping_stop_unreleased`, and top-level `passed_runs >= 2`. If the experiment fails, report its exact diagnosis without a speculative additional source change.

- [ ] **Step 3: Request a whole-branch review**

Create a review package from the pre-fix merge base through the current head and dispatch a reviewer. The reviewer must inspect vendor-ordering scope, vendor test validity, wrapper-evidence interaction, and all test/build evidence. Fix Critical or Important findings before completion.

- [ ] **Step 4: Preserve artifacts but do not commit them**

Run: `git status --short`

Expected: generated `artifacts/circle-loop-evaluation-*` output remains untracked; no artifacts, maps, logs, or bag-derived files are staged.

## Plan Self-Review

- Scope coverage: Task 1 establishes the pre-edge failure contract; Task 2 applies only the ordered notification move and verifies vendor plus integration tests; Task 3 provides the empirical circle-run proof and final review.
- The plan does not alter matching, thresholds, GBA, map merge, feature extraction, or unrelated upstream issues.
- The notification contract is explicit: one notification, after two symmetric edges, before local mapping release.
