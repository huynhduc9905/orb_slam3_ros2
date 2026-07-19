# Circle-Run Loop-Closure Observability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce evidence that distinguishes core ORB-SLAM3 loop detection, wrapper loop observation, and map rebuild completion on three circle-run replays.

**Architecture:** The C++ wrapper enriches its existing graph-delta event evidence without changing the ROS message schema. A small Python evaluator reads wrapper logs and metrics artifacts to produce a diagnosis. A shell runner invokes three isolated real-time runs and aggregates the evaluator output.

**Tech Stack:** ROS 2 Kilted, C++17, Python 3, pytest, gtest, bash, existing `TrackingEvent` and metrics/report artifacts.

## Global Constraints

- Do not modify `orb_slam3_vendor/vendor/ORB_SLAM3` in this plan.
- Do not alter loop-detection or feature-extractor thresholds in this plan.
- Preserve existing `TrackingEvent` type codes and consumers.
- Unit tests must use synthetic metrics and log text, not bag replay.
- The experiment uses `/home/duc/robot/bag/circle-run` at `rate:=1` and `benchmark_mode:=off`.
- The experiment passes only when at least two of three runs satisfy all per-run gates.

---

## File Structure

- Modify `orb_slam3_wrapper/include/orb_slam3_wrapper/graph_semantics.hpp`: expose structured graph-delta evidence while retaining legacy event classification.
- Modify `orb_slam3_wrapper/src/graph_semantics.cpp`: identify normalized new loop edges and their map lineage.
- Modify `orb_slam3_wrapper/src/wrapper_node.cpp`: serialize evidence in existing event detail fields.
- Modify `orb_slam3_wrapper/test/graph_semantics_test.cpp`: verify same-map and cross-map evidence and legacy event types.
- Create `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py`: parse core markers, metrics, rebuild state, and diagnose a run.
- Create `orb_slam_bringup/test/test_loop_closure_evidence.py`: isolated parser and diagnosis tests.
- Modify `orb_slam_bringup/setup.py`: install the evaluator command if existing package patterns use console scripts.
- Create `tools/run_circle_loop_closure_evaluation.sh`: execute and aggregate three independent replays.
- Modify `README.md`: document the evidence artifact and circle-run command.

### Task 1: Document the approved design and upstream issue ledger

**Files:**
- Create: `docs/superpowers/specs/2026-07-19-circle-run-loop-closure-observability-design.md`

**Interfaces:**
- Produces: immutable scope and evidence gates for later implementation tasks.

- [ ] **Step 1: Add the design document**

Include these exact sections: `Goal`, `Scope`, `Current State`, `Design`, `Circle-Run Protocol`, `Upstream Issue Ledger`, `Constraints`, and `Verification`.

The upstream issue table must include links and evidence-gated follow-up conditions for #911, #745, #394, #718, and #57.

- [ ] **Step 2: Inspect for incomplete requirements**

Run: `git diff --check -- docs/superpowers/specs/2026-07-19-circle-run-loop-closure-observability-design.md`

Expected: exit status `0`.

- [ ] **Step 3: Commit the design**

```bash
git add docs/superpowers/specs/2026-07-19-circle-run-loop-closure-observability-design.md
git commit -m "docs: record loop closure investigation design"
```

### Task 2: Enrich wrapper graph-loop evidence

**Files:**
- Modify: `orb_slam3_wrapper/include/orb_slam3_wrapper/graph_semantics.hpp`
- Modify: `orb_slam3_wrapper/src/graph_semantics.cpp`
- Modify: `orb_slam3_wrapper/src/wrapper_node.cpp`
- Modify: `orb_slam3_wrapper/test/graph_semantics_test.cpp`

**Interfaces:**
- Consumes: `ORB_SLAM3::GraphSnapshot`, its keyframe IDs, map IDs, active map ID, and loop-edge IDs.
- Produces: `GraphDeltaEvidence { std::vector<std::uint8_t> event_types; std::vector<LoopEdgeEvidence> loop_edges; bool map_merged; }`.
- `LoopEdgeEvidence` contains `first_keyframe_id`, `second_keyframe_id`, `first_map_id`, `second_map_id`, `active_map_id`, and `classification`.

- [ ] **Step 1: Write failing gtests**

Add tests that construct previous/current snapshots containing a new normalized edge `(10, 20)` and assert:

```cpp
EXPECT_EQ(evidence.loop_edges.size(), 1U);
EXPECT_EQ(evidence.loop_edges[0].first_keyframe_id, 10U);
EXPECT_EQ(evidence.loop_edges[0].second_keyframe_id, 20U);
EXPECT_EQ(evidence.loop_edges[0].classification, "same_map_loop");
EXPECT_THAT(evidence.event_types,
            ::testing::Contains(orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED));
```

Add a cross-map fixture and assert `classification == "cross_map_loop"`. Keep the existing `classifyGraphDelta()` API as a compatibility wrapper over the new classifier.

- [ ] **Step 2: Run the focused test to verify it fails**

Run: `colcon test --packages-select orb_slam3_wrapper --ctest-args -R graph_semantics_test --output-on-failure`

Expected: FAIL because `GraphDeltaEvidence` and `classifyGraphDeltaEvidence()` do not exist.

- [ ] **Step 3: Implement the minimal evidence model**

In `graph_semantics.hpp`, define:

```cpp
struct LoopEdgeEvidence {
  std::uint64_t first_keyframe_id;
  std::uint64_t second_keyframe_id;
  std::uint64_t first_map_id;
  std::uint64_t second_map_id;
  std::uint64_t active_map_id;
  std::string classification;
};

struct GraphDeltaEvidence {
  std::vector<std::uint8_t> event_types;
  std::vector<LoopEdgeEvidence> loop_edges;
  bool map_merged{false};
};

GraphDeltaEvidence classifyGraphDeltaEvidence(
    const std::optional<ORB_SLAM3::GraphSnapshot>& previous,
    const ORB_SLAM3::GraphSnapshot& current);
```

Build an ID-to-map-ID lookup from `current.keyframes`. For each edge absent from the previous normalized edge set, emit one `LoopEdgeEvidence`. Classify it `same_map_loop` only when both map IDs equal `current.active_map_id`; otherwise classify it `cross_map_loop`. Preserve the existing merge condition and legacy vector-returning function.

In `wrapper_node.cpp`, call `classifyGraphDeltaEvidence()`. For each loop edge, publish one existing `LOOP_CLOSED` event with detail formatted exactly as:

```text
loop_edge=<first>-<second> classification=<classification> maps=<first_map>,<second_map> active_map=<active_map>
```

Publish other event types once with their existing details.

- [ ] **Step 4: Run focused tests to verify they pass**

Run: `colcon test --packages-select orb_slam3_wrapper --ctest-args -R graph_semantics_test --output-on-failure`

Expected: all graph-semantics tests pass.

- [ ] **Step 5: Commit**

```bash
git add orb_slam3_wrapper/include/orb_slam3_wrapper/graph_semantics.hpp \
  orb_slam3_wrapper/src/graph_semantics.cpp \
  orb_slam3_wrapper/src/wrapper_node.cpp \
  orb_slam3_wrapper/test/graph_semantics_test.cpp
git commit -m "feat: enrich loop closure graph evidence"
```

### Task 3: Evaluate core and wrapper loop evidence

**Files:**
- Create: `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py`
- Create: `orb_slam_bringup/test/test_loop_closure_evidence.py`
- Modify: `orb_slam_bringup/setup.py`

**Interfaces:**
- Consumes: `artifact_dir/metrics.json` and `artifact_dir/orb_slam3_wrapper.log`.
- Produces: `artifact_dir/loop_closure_evidence.json`.
- CLI: `python3 -m orb_slam_bringup.loop_closure_evidence --artifact-dir PATH` returns `0` when a run passes, `1` for measured gate failure, and `2` for missing/invalid inputs.

- [ ] **Step 1: Write failing parser and diagnosis tests**

Create synthetic metrics and log fixtures. Assert marker counts and diagnoses:

```python
def test_core_loop_without_wrapper_event_is_unobserved(tmp_path: Path):
    write_metrics(tmp_path, loops=[], revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["marker_counts"]["loop_detected"] == 1
    assert "core_loop_unobserved" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_observed_loop_with_published_rebuild_passes(tmp_path: Path):
    write_metrics(tmp_path, loops=[{"graph_revision": 4}],
                  revisions=[{"state": "PUBLISHED", "graph_revision": 4}],
                  initialized=True, ok_ratio=0.9, deadlock=False)
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\nLocal Mapping STOP\nLocal Mapping RELEASE\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["diagnoses"] == ["observed_and_rebuilt"]
    assert evidence["passed"] is True
```

Add tests for `BAD LOOP!!!`, unreleased local mapping stop, missing core log, and wrapper loop lacking a published rebuild.

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_loop_closure_evidence.py -v`

Expected: FAIL during import because the evaluator module does not exist.

- [ ] **Step 3: Implement evaluator**

Implement `evaluate_artifact(artifact_dir: Path) -> dict` with these marker keys:

```python
MARKERS = {
    "loop_detected": "*Loop detected",
    "bad_loop": "BAD LOOP!!!",
    "local_mapping_stop": "Local Mapping STOP",
    "local_mapping_release": "Local Mapping RELEASE",
    "merge_detected": "*Merge detected",
}
```

Load `metrics.json`. Treat a missing/invalid metrics file as an invalid CLI input. Treat a missing wrapper log as a valid measured condition with diagnosis `core_log_missing`.

Calculate `loop_rebuild_missing` using the same rule as `report.py`: every loop `graph_revision` needs a `PUBLISHED` revision at or above it. Add diagnoses in this deterministic order: `core_log_missing`, `no_core_loop_detected`, `core_loop_unobserved`, `loop_rebuild_missing`, `local_mapping_stop_unreleased`; if none apply and all per-run gates pass, emit exactly `["observed_and_rebuilt"]`.

Write the JSON atomically and make `main()` return `0` for `passed`, `1` for measured failure, `2` for invalid input.

Add the console script only if `setup.py` already exposes analogous module commands; otherwise module execution is the supported interface.

- [ ] **Step 4: Run focused tests to verify they pass**

Run: `pytest test/test_loop_closure_evidence.py -v`

Expected: all evaluator tests pass.

- [ ] **Step 5: Commit**

```bash
git add orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py \
  orb_slam_bringup/test/test_loop_closure_evidence.py \
  orb_slam_bringup/setup.py
git commit -m "feat: add loop closure evidence evaluator"
```

### Task 4: Preserve wrapper output in circle-run artifacts

**Files:**
- Modify: `orb_slam_bringup/launch/bag_replay.launch.py`
- Modify: `orb_slam_bringup/test/test_bag_profile.py`

**Interfaces:**
- Consumes: configured `artifact_dir`.
- Produces: `${artifact_dir}/orb_slam3_wrapper.log` for all non-benchmark full-stack replays.

- [ ] **Step 1: Write a failing launch contract test**

Add assertions that the launch source passes an artifact-derived path ending in `orb_slam3_wrapper.log` to the wrapper process and uses output configuration that persists both stdout and stderr.

- [ ] **Step 2: Run the test to verify it fails**

Run: `pytest test/test_bag_profile.py -k wrapper_log -v`

Expected: FAIL because no wrapper artifact log exists.

- [ ] **Step 3: Implement wrapper log capture**

Use the established ROS launch `Node` output mechanism or `ExecuteProcess` redirection pattern already present in the repository. The wrapper's output must still appear on screen and be written to exactly:

```python
wrapper_log_path = str(Path(artifact_dir) / "orb_slam3_wrapper.log")
```

Do not redirect the global launch log and do not change node arguments or loop behavior.

- [ ] **Step 4: Run the focused test to verify it passes**

Run: `pytest test/test_bag_profile.py -k wrapper_log -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add orb_slam_bringup/launch/bag_replay.launch.py orb_slam_bringup/test/test_bag_profile.py
git commit -m "feat: retain wrapper loop closure logs"
```

### Task 5: Add the three-run circle evaluation runner

**Files:**
- Create: `tools/run_circle_loop_closure_evaluation.sh`
- Modify: `orb_slam_bringup/test/test_loop_closure_evidence.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: `--bag PATH`, `--output PATH`, optional `--domain INTEGER`.
- Produces: `run-1`, `run-2`, `run-3` artifact directories and `summary.json` under output.
- Exit status: `0` if two or more evidence files pass, `1` for completed experiment failure, `2` for invalid invocation/setup.

- [ ] **Step 1: Write failing runner contract tests**

Read the shell script as text and assert it contains all required invariants:

```python
assert "rate:=1" in text
assert "benchmark_mode:=off" in text
assert "run-1" in text and "run-2" in text and "run-3" in text
assert "loop_closure_evidence" in text
assert "passed_runs" in text
```

Also add pure-Python tests for `summarize_runs([True, True, False])` returning `{"passed_runs": 2, "passed": True}` and `[True, False, False]` returning `{"passed_runs": 1, "passed": False}`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_loop_closure_evidence.py -v`

Expected: FAIL because the runner and `summarize_runs()` do not exist.

- [ ] **Step 3: Implement the summary helper and shell runner**

In the Python evaluator module implement:

```python
def summarize_runs(evidence_paths: Sequence[Path]) -> dict:
    runs = [json.loads(path.read_text(encoding="utf-8")) for path in evidence_paths]
    passed_runs = sum(bool(run.get("passed")) for run in runs)
    return {
        "runs": [str(path) for path in evidence_paths],
        "passed_runs": passed_runs,
        "required_passes": 2,
        "passed": passed_runs >= 2,
    }
```

The runner must validate the bag, `install/setup.bash`, `ros2`, and `python3`; source the overlay with `set +u`, `source install/setup.bash`, `set -u`; launch three runs under `DOMAIN`, `DOMAIN + 1`, and `DOMAIN + 2`; call the evaluator after each run with `set +e`; and invoke a new `summary` subcommand to atomically write top-level `summary.json`.

The launch command is exactly:

```bash
ros2 launch orb_slam_bringup bag_replay.launch.py \
  "bag_path:=${BAG_PATH}" \
  "artifact_dir:=${RUN_DIR}" \
  "rate:=1" \
  "ros_domain_id:=${CURRENT_DOMAIN}" \
  "start_dashboard:=false" \
  "benchmark_mode:=off"
```

Document the command and its two-of-three gate in `README.md`.

- [ ] **Step 4: Run focused tests and shell syntax check**

Run: `pytest test/test_loop_closure_evidence.py -v && bash -n ../tools/run_circle_loop_closure_evaluation.sh`

Expected: all tests pass and bash exits `0`.

- [ ] **Step 5: Commit**

```bash
git add tools/run_circle_loop_closure_evaluation.sh \
  orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py \
  orb_slam_bringup/test/test_loop_closure_evidence.py README.md
git commit -m "feat: add circle loop closure evaluation runner"
```

### Task 6: Build, execute the experiment, and document measured result

**Files:**
- Create: `artifacts/circle-loop-evaluation-YYYYMMDD/summary.json` (untracked experiment output)
- Modify: `README.md` only if the command/result interpretation needs correction.

**Interfaces:**
- Consumes: built workspace and `/home/duc/robot/bag/circle-run`.
- Produces: three self-contained run artifacts and a top-level summary.

- [ ] **Step 1: Build affected packages**

Run: `COLCON_DEFAULTS_FILE=/dev/null colcon build --packages-select orb_slam3_wrapper orb_slam_bringup --symlink-install`

Expected: both packages finish successfully.

- [ ] **Step 2: Run all focused tests**

Run: `pytest orb_slam_bringup/test/test_loop_closure_evidence.py orb_slam_bringup/test/test_bag_profile.py -v`

Expected: all tests pass.

- [ ] **Step 3: Execute the three-run experiment**

Run:

```bash
tools/run_circle_loop_closure_evaluation.sh \
  --bag /home/duc/robot/bag/circle-run \
  --output artifacts/circle-loop-evaluation-$(date +%Y%m%d_%H%M%S) \
  --domain 130
```

Expected: `summary.json` is written. Its status is evidence, not a precondition: report either the passing two-of-three result or each measured diagnosis.

- [ ] **Step 4: Verify artifacts and report result**

Run: `python3 -m json.tool artifacts/circle-loop-evaluation-*/summary.json`

Expected: output includes exactly three evidence paths, `passed_runs`, `required_passes: 2`, and `passed`.

- [ ] **Step 5: Commit only source/documentation changes if any follow from the experiment**

Do not commit bag artifacts, logs, maps, or generated summaries. If Task 6 changed tracked source/docs, commit only those files:

```bash
git add README.md
git commit -m "docs: clarify circle loop closure evaluation"
```

## Plan Self-Review

- Spec coverage: Task 1 records all upstream issues; Tasks 2-4 add the required core/wrapper/rebuild attribution; Task 5 implements the three-run protocol; Task 6 performs the actual circle-run experiment.
- No vendor source, matching thresholds, or message-schema changes are planned.
- Type consistency: C++ evidence types are scoped to graph semantics; Python evaluator owns metrics/log interpretation and summary aggregation; shell only orchestrates existing launch and evaluator CLI interfaces.
