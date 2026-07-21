# Wrapper Independent Graph Observation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish asynchronous ORB-SLAM3 graph changes, including loop-edge evidence, even when no further stereo frame arrives.

**Architecture:** `WrapperNode` owns a single low-rate wall timer that is the sole consumer of `SlamBackend::mapChanged()`. The existing stereo callback continues to track and record the latest output header; the timer obtains a changed snapshot and sends it through the unchanged `publishGraph()` semantic-delta path. Both callbacks remain in the node's default mutually exclusive callback group, serializing access to the backend and graph state.

**Tech Stack:** ROS 2 `rclcpp` wall timers, C++17, GoogleTest, existing `SlamBackend` fake component fixture, colcon/CTest.

## Global Constraints

- Retain vendor commit `601bec2`; do not alter vendor loop correction, matching, GBA, merging, or feature extraction.
- `WrapperNode`'s timer is the only caller of `SlamBackend::mapChanged()`.
- Do not fetch a graph snapshot unless `mapChanged()` returns true.
- Do not consume map changes before backend configuration and a tracked-frame header are available.
- Use the existing default mutually exclusive callback group; do not add backend threads, locks, or callback groups.
- Preserve existing `publishGraph()` graph-delta classification and event formats.
- Test timer behavior by spinning the node, not by expecting graph publication inside `processStereoForTest()`.
- Preserve unrelated user scratch files and generated artifacts.

---

## File Structure

- Modify: `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`
  - Declare the graph-timer callback and retain its `rclcpp::TimerBase` ownership.
- Modify: `orb_slam3_wrapper/src/wrapper_node.cpp`
  - Create the low-rate timer, move map-change consumption out of stereo processing, and publish changed graphs from the timer.
- Modify: `orb_slam3_wrapper/test/wrapper_component_test.cpp`
  - Drive node timers deterministically and verify asynchronous graph/loop event observation.
- Modify: `docs/superpowers/specs/2026-07-19-wrapper-independent-graph-observation-design.md`
  - Append the measured implementation result after the circle evaluation.

## Task 1: Timer-Driven Graph Observation

**Files:**
- Modify: `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp:54-105`
- Modify: `orb_slam3_wrapper/src/wrapper_node.cpp:114-132,241-247`
- Modify: `orb_slam3_wrapper/test/wrapper_component_test.cpp:17-43,111-151,195-207`

**Interfaces:**
- Consumes: `SlamBackend::mapChanged() -> bool`, `SlamBackend::graphSnapshot() -> ORB_SLAM3::GraphSnapshot`, and the existing `WrapperNode::publishGraph(const ORB_SLAM3::GraphSnapshot&, const std_msgs::msg::Header&)`.
- Produces: `WrapperNode::pollGraphChanges()` invoked by `graph_timer_`; it publishes changed graph snapshots and semantic events using `last_tracked_.header`.

- [ ] **Step 1: Add a component-test helper that spins a timer callback**

Add the following includes and helper near `setInfo()` in `orb_slam3_wrapper/test/wrapper_component_test.cpp`:

```cpp
#include <chrono>
#include <thread>

bool spinUntil(const std::shared_ptr<orb_slam3_wrapper::WrapperNode>& node,
               const std::function<bool()>& ready) {
  for (int i = 0; i < 20 && !ready(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return ready();
}
```

Include `<functional>` as well. The helper does not use an executor shared by other tests and gives a 400 ms bound for the timer to execute.

- [ ] **Step 2: Replace callback-coupled graph assertions with red asynchronous regressions**

Replace `MapChangePublishesOneSnapshotAndConservativeEvent` with a test that first tracks a frame while `backend->changed` is false, then sets `changed = true` and `graph.revision = 4` without invoking `processStereoForTest()` again. Assert the timer has not published synchronously, then spin until the graph is published:

```cpp
node->processStereoForTest(image, image);
EXPECT_EQ(node->graphPublishCountForTest(), 0u);
backend_ptr->graph.revision = 4;
backend_ptr->graph.active_map_id = 17;
backend_ptr->changed = true;
ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));
EXPECT_EQ(node->lastGraphSnapshotForTest().revision, 4u);
```

Replace `GraphLoopEdgesAreCanonicalizedAndDeduplicated` with an asynchronous loop regression. Track a baseline frame with an empty, changed graph at revision 1 and spin until it is published. Then assign the reciprocal loop keyframes and revision 2, set `changed = true`, and spin without another stereo call:

```cpp
ORB_SLAM3::KeyframeSnapshot first;
first.id = 20;
first.map_id = 17;
first.loop_edge_ids = {10};
ORB_SLAM3::KeyframeSnapshot second;
second.id = 10;
second.map_id = 17;
second.loop_edge_ids = {20};
backend_ptr->graph.revision = 2;
backend_ptr->graph.keyframes = {first, second};
backend_ptr->changed = true;
ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 2u; }));
ASSERT_EQ(node->lastGraphSnapshotForTest().loop_edges.size(), 1u);
EXPECT_EQ(node->lastGraphSnapshotForTest().loop_edges[0].from_id, 10u);
EXPECT_EQ(node->lastGraphSnapshotForTest().loop_edges[0].to_id, 20u);
EXPECT_EQ(node->lastTrackingEventForTest().type,
          orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
```

In `ValidCalibrationConfiguresBackendExactlyOnceAndCoheresRevision`, replace the second `processStereoForTest()` and direct revision assertion with `spinUntil()` after setting `changed`, so it continues to test that a timer-published snapshot updates `lastTrackedFrameForTest().graph_revision`.

- [ ] **Step 3: Run the focused component test to verify red**

Run:

```bash
colcon test --packages-select orb_slam3_wrapper --ctest-args -R wrapper_component_test --output-on-failure
```

Expected: the new asynchronous tests fail because no timer exists and `graphPublishCountForTest()` remains zero after `spinUntil()`.

- [ ] **Step 4: Declare the timer callback and ownership**

In `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`, add this private method beside `processStereo()`:

```cpp
void pollGraphChanges();
```

Add timer ownership after `synchronizer_`:

```cpp
rclcpp::TimerBase::SharedPtr graph_timer_;
```

- [ ] **Step 5: Implement the sole graph-change consumer**

In `WrapperNode`'s constructor, create the wall timer after the synchronizer registration and before backend construction:

```cpp
graph_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
  pollGraphChanges();
});
```

Add `<chrono>` to `orb_slam3_wrapper/src/wrapper_node.cpp`.

Implement the callback before `processStereo()`:

```cpp
void WrapperNode::pollGraphChanges() {
  if (!backend_ || !backend_configured_ || last_tracked_.header.frame_id.empty()) return;
  if (!backend_->mapChanged()) return;
  const auto graph = backend_->graphSnapshot();
  if (graph.revision != last_graph_revision_) publishGraph(graph, last_tracked_.header);
}
```

Delete the `graph_to_publish` block from `processStereo()`:

```cpp
std::optional<ORB_SLAM3::GraphSnapshot> graph_to_publish;
if (backend_->mapChanged()) {
  const auto graph = backend_->graphSnapshot();
  if (graph.revision != last_graph_revision_) graph_to_publish = graph;
}
```

Replace the output graph revision assignment with:

```cpp
output.graph_revision = last_graph_revision_;
```

Delete the later conditional publication:

```cpp
if (graph_to_publish) publishGraph(*graph_to_publish, output.header);
```

The default callback group remains mutually exclusive, so the timer and subscriptions cannot concurrently access backend or state.

- [ ] **Step 6: Run the focused component test to verify green**

Run:

```bash
colcon build --packages-select orb_slam3_wrapper
colcon test --packages-select orb_slam3_wrapper --ctest-args -R wrapper_component_test --output-on-failure
```

Expected: build succeeds and all `wrapper_component_test` cases pass, including the asynchronous loop-edge test.

- [ ] **Step 7: Run focused wrapper and evidence contracts**

Run:

```bash
colcon test --packages-select orb_slam3_wrapper --ctest-args -R 'wrapper_component_test|graph_semantics_test' --output-on-failure
pytest orb_slam_bringup/test/test_loop_closure_evidence.py -v
```

Expected: both wrapper CTest targets pass and the loop-evidence Python suite passes.

- [ ] **Step 8: Commit the independently testable wrapper fix**

```bash
git add orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp \
        orb_slam3_wrapper/src/wrapper_node.cpp \
        orb_slam3_wrapper/test/wrapper_component_test.cpp
git commit -m "fix: poll graph changes independently of frames"
```

## Task 2: Circle-Run Acceptance Evidence

**Files:**
- Modify: `docs/superpowers/specs/2026-07-19-wrapper-independent-graph-observation-design.md`
- Create: `artifacts/circle-loop-evaluation-<timestamp>/` (generated, untracked)

**Interfaces:**
- Consumes: `tools/run_circle_loop_closure_evaluation.sh --bag PATH --output PATH --domain INTEGER` and per-run `loop_closure_evidence.json`.
- Produces: a recorded acceptance result whose summary contains `passed_runs >= 2` and whose qualifying run diagnoses contain `observed_and_rebuilt`.

- [ ] **Step 1: Execute the three isolated real-time trials**

Run from the workspace root:

```bash
tools/run_circle_loop_closure_evaluation.sh \
  --bag /home/duc/robot/bag/circle-run \
  --output artifacts/circle-loop-evaluation-$(date +%Y%m%d_%H%M%S) \
  --domain 140
```

Expected: exit status 0 only if at least two of three runs pass. Preserve the exact generated artifact directory for inspection even if the command exits 1.

- [ ] **Step 2: Inspect every run rather than relying only on the exit status**

For the generated output directory, inspect `summary.json`, each
`run-*/loop_closure_evidence.json`, and each `run-*/metrics.json`. Confirm at
least two runs have all of the following:

```json
{
  "diagnoses": ["observed_and_rebuilt"],
  "passed": true
}
```

Confirm their metrics include a non-empty `loops` array and a `PUBLISHED`
map revision whose `graph_revision` is at least the loop's graph revision.

- [ ] **Step 3: Record the measured outcome in the design spec**

Append a `## Acceptance Result` section to
`docs/superpowers/specs/2026-07-19-wrapper-independent-graph-observation-design.md` containing the exact artifact path, superproject commit, pass count, each run's diagnosis, and whether the two-of-three gate passed. If the gate fails, state the evidence precisely and do not make another implementation change in this task.

- [ ] **Step 4: Commit only the source-controlled acceptance record**

```bash
git add docs/superpowers/specs/2026-07-19-wrapper-independent-graph-observation-design.md
git commit -m "docs: record graph observation evaluation"
```

Do not add the generated `artifacts/` directory or unrelated scratch files.
