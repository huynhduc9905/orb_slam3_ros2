# Wrapper Graph Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Capture a non-published initial ORB-SLAM3 graph snapshot so the first changed graph snapshot can emit a canonical `LOOP_CLOSED` event.

**Architecture:** The graph timer continues to be the sole consumer of `SlamBackend::mapChanged()`. Once its existing readiness guard passes, it captures one `SlamBackend::graphSnapshot()` into `previous_graph_` before invoking `mapChanged()`. Changed snapshots retain the existing `publishGraph()` flow and are classified against that retained baseline.

**Tech Stack:** ROS 2 Kilted, C++17, rclcpp, GoogleTest, CMake, colcon, Nix dev shell.

## Global Constraints

- Modify only `orb_slam3_wrapper` tracked source and test files for the behavioral change.
- Do not edit the vendored ORB-SLAM3 submodule.
- Do not change evaluator criteria or tune camera or ORB parameters.
- Preserve the existing graph timer as the sole consumer of `SlamBackend::mapChanged()`.
- The baseline snapshot must not publish `/orb_slam3/graph_snapshot`, publish semantic events, update `last_graph_revision_`, or trigger a downstream rebuild.
- A later changed snapshot with a newly introduced reciprocal loop edge must publish one canonical `LOOP_CLOSED` event.
- Follow TDD: demonstrate the focused regression fails before production code is written, then pass it and run focused wrapper coverage.
- Do not stage, modify, or delete existing unrelated untracked scratch files or generated artifacts.

---

### Task 1: Capture and consume the initial graph baseline

**Files:**
- Modify: `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`
- Modify: `orb_slam3_wrapper/src/wrapper_node.cpp`
- Modify: `orb_slam3_wrapper/test/wrapper_component_test.cpp`

**Interfaces:**
- Consumes: `SlamBackend::graphSnapshot() -> ORB_SLAM3::GraphSnapshot` and `SlamBackend::mapChanged() -> bool`.
- Produces: `previous_graph_` contains a non-published initial baseline before the first changed graph is classified by `publishGraph(const ORB_SLAM3::GraphSnapshot&, const std_msgs::msg::Header&)`.

- [ ] **Step 1: Add the focused failing component regression**

Add this test next to `GraphLoopEdgesAreCanonicalizedAndDeduplicated` in `orb_slam3_wrapper/test/wrapper_component_test.cpp`:

```cpp
TEST_F(WrapperComponentTest, FirstChangedGraphWithLoopEdgeEmitsLoopClosed) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);

  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical";
  image.height = image.width = 2;
  image.encoding = "mono8";
  image.step = 2;
  image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);

  ASSERT_TRUE(spinUntil(node, [&] { return backend_ptr->track_calls == 1; }));
  EXPECT_EQ(node->graphPublishCountForTest(), 0u);
  EXPECT_EQ(node->eventPublishCountForTest(), 1u);

  ORB_SLAM3::KeyframeSnapshot first;
  first.id = 20;
  first.map_id = 17;
  first.loop_edge_ids = {10};
  ORB_SLAM3::KeyframeSnapshot second;
  second.id = 10;
  second.map_id = 17;
  second.loop_edge_ids = {20};
  backend_ptr->graph.revision = 1;
  backend_ptr->graph.active_map_id = 17;
  backend_ptr->graph.keyframes = {first, second};
  backend_ptr->changed = true;

  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));
  EXPECT_EQ(node->lastTrackingEventForTest().type,
            orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
  EXPECT_EQ(node->eventPublishCountForTest(), 2u);
}
```

The initial `INITIALIZED` event makes the event count one before the graph change and two after the `LOOP_CLOSED` event. The test's timer spin while `changed` is false proves that the baseline is non-published.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
nix develop --command colcon build --packages-select orb_slam3_wrapper
nix develop --command colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R wrapper_component_test --output-on-failure
```

Expected: `FirstChangedGraphWithLoopEdgeEmitsLoopClosed` fails because no snapshot is read while `changed` is false, so `previous_graph_` is empty when revision 1 is classified and no `LOOP_CLOSED` event is emitted.

- [ ] **Step 3: Add the one-time baseline state**

In `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`, add the state beside `previous_graph_`:

```cpp
bool graph_baseline_captured_{false};
std::optional<ORB_SLAM3::GraphSnapshot> previous_graph_;
```

- [ ] **Step 4: Capture the baseline before the timer consumes a change**

In `WrapperNode::pollGraphChanges()` in `orb_slam3_wrapper/src/wrapper_node.cpp`, replace the current body with:

```cpp
void WrapperNode::pollGraphChanges() {
  if (!backend_ || !backend_configured_ || last_tracked_.header.frame_id.empty()) return;
  if (!graph_baseline_captured_) {
    previous_graph_ = backend_->graphSnapshot();
    graph_baseline_captured_ = true;
  }
  if (!backend_->mapChanged()) return;
  const auto graph = backend_->graphSnapshot();
  if (graph.revision != last_graph_revision_) publishGraph(graph, last_tracked_.header);
}
```

Do not modify `publishGraph()`, `classifyGraphDeltaEvidence()`, or vendor code. The baseline must be captured exactly once after the readiness guard, before the sole `mapChanged()` call, and must have no publication side effects.

- [ ] **Step 5: Run the focused regression and verify GREEN**

Run:

```bash
nix develop --command colcon build --packages-select orb_slam3_wrapper
nix develop --command colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R wrapper_component_test --output-on-failure
```

Expected: `wrapper_component_test` passes, including `FirstChangedGraphWithLoopEdgeEmitsLoopClosed`.

- [ ] **Step 6: Run wrapper graph semantics coverage**

Run:

```bash
nix develop --command colcon test --packages-select orb_slam3_wrapper \
  --ctest-args -R 'wrapper_component_test|graph_semantics_test' --output-on-failure
```

Expected: both test executables pass with no failures.

- [ ] **Step 7: Inspect and commit the task files**

Run:

```bash
git diff --check
git diff -- orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp \
  orb_slam3_wrapper/src/wrapper_node.cpp \
  orb_slam3_wrapper/test/wrapper_component_test.cpp
git add orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp \
  orb_slam3_wrapper/src/wrapper_node.cpp \
  orb_slam3_wrapper/test/wrapper_component_test.cpp \
  docs/superpowers/plans/2026-07-20-wrapper-graph-baseline.md
git commit -m "fix: establish graph baseline before loop changes"
```

Expected: the commit contains the baseline state, the timer behavior, the regression, and this plan. Existing unrelated untracked files and generated artifacts remain untouched.
