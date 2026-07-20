# Wrapper Graph Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Capture a non-semantic initial ORB-SLAM3 graph snapshot so a later loop-edge graph change is correctly emitted as `LOOP_CLOSED`.

**Architecture:** `WrapperNode` currently asks the backend for a graph only after `mapChanged()` is true, but ORB-SLAM3 only reports that on the first big change. Add one baseline snapshot acquisition once tracking and backend configuration are ready; retain it for delta comparison without publishing it or emitting semantic events. Existing changed snapshots continue through `publishGraph()`, so downstream graph publication and rebuild behavior is unchanged except that loop edges introduced after the baseline are visible as semantic events.

**Tech Stack:** ROS 2 C++, rclcpp component tests, GoogleTest, ORB-SLAM3 graph snapshots.

## Global Constraints

- Modify only `orb_slam3_wrapper` tracked source and test files for this behavioral change.
- Do not edit the vendored ORB-SLAM3 submodule.
- Do not change evaluator criteria or tune camera/ORB parameters.
- Preserve the existing timer as the sole consumer of `SlamBackend::mapChanged()`.
- The baseline snapshot must not publish `/orb_slam3/graph_snapshot` or semantic events.
- A later changed snapshot with a newly introduced loop edge must publish one canonical `LOOP_CLOSED` event.
- Follow TDD: demonstrate the focused regression fails before production code is written, then pass it and run the focused wrapper test suite.
- Do not stage, modify, or delete existing unrelated untracked scratch files or generated artifacts.

---

### Task 1: Establish and consume an initial graph baseline

**Files:**
- Modify: `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`
- Modify: `orb_slam3_wrapper/src/wrapper_node.cpp`
- Modify: `orb_slam3_wrapper/test/wrapper_component_test.cpp`

**Interfaces:**
- Consumes: `SlamBackend::graphSnapshot() -> ORB_SLAM3::GraphSnapshot`, `SlamBackend::mapChanged() -> bool`.
- Produces: `previous_graph_` contains an initial baseline before a changed snapshot is classified; existing `publishGraph(const ORB_SLAM3::GraphSnapshot&, const std_msgs::msg::Header&)` publishes only changed snapshots.

- [ ] **Step 1: Write the failing component regression**

Add a test alongside `GraphLoopEdgesAreCanonicalizedAndDeduplicated` named `FirstChangedGraphWithLoopEdgeEmitsLoopClosed`. Configure the fake backend and node normally, spin after one successful stereo frame while `changed` is false, and assert that no graph or event has been published. Then make the fake snapshot revision `1`, add two reciprocal loop-edge keyframes, set `changed = true`, spin until one graph is published, and assert exactly one `LOOP_CLOSED` event was published. The test must prove the initial graph was captured only as a non-published baseline before the loop edge appears.

- [ ] **Step 2: Run the regression and verify RED**

Run:

```bash
colcon build --packages-select orb_slam3_wrapper
colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R wrapper_component_test --output-on-failure
```

Expected: the new `FirstChangedGraphWithLoopEdgeEmitsLoopClosed` assertion fails because the existing code does not call `graphSnapshot()` while `changed` is false, leaving `previous_graph_` empty when revision 1 arrives.

- [ ] **Step 3: Add the minimal baseline state and timer behavior**

Add a boolean member initialized false, e.g. `graph_baseline_captured_`. In `WrapperNode::pollGraphChanges()` after the existing readiness guard:

```cpp
if (!graph_baseline_captured_) {
  previous_graph_ = backend_->graphSnapshot();
  graph_baseline_captured_ = true;
}
if (!backend_->mapChanged()) return;
const auto graph = backend_->graphSnapshot();
if (graph.revision != last_graph_revision_) publishGraph(graph, last_tracked_.header);
```

The baseline acquisition must occur before `mapChanged()`, must not call `publishGraph()`, and must not update `last_graph_revision_`. Do not alter `classifyGraphDeltaEvidence()` or vendor code.

- [ ] **Step 4: Run the focused regression and verify GREEN**

Run:

```bash
colcon build --packages-select orb_slam3_wrapper
colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R wrapper_component_test --output-on-failure
```

Expected: `wrapper_component_test` passes, including `FirstChangedGraphWithLoopEdgeEmitsLoopClosed`.

- [ ] **Step 5: Run the wrapper graph semantics coverage**

Run:

```bash
colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R 'wrapper_component_test|graph_semantics_test' --output-on-failure
```

Expected: both test executables pass with no failures.

- [ ] **Step 6: Inspect the patch and commit only tracked task files**

Run:

```bash
git diff --check
git diff -- orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp \
  orb_slam3_wrapper/src/wrapper_node.cpp \
  orb_slam3_wrapper/test/wrapper_component_test.cpp
git add orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp \
  orb_slam3_wrapper/src/wrapper_node.cpp \
  orb_slam3_wrapper/test/wrapper_component_test.cpp \
  docs/superpowers/plans/2026-07-19-wrapper-graph-baseline.md
git commit -m "fix: establish graph baseline before loop changes"
```

Expected: the commit contains only the baseline implementation, its regression test, and this plan; unrelated untracked files remain untouched.
