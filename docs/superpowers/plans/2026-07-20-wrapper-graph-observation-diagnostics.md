# Wrapper Graph Observation Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Log the graph snapshot evidence observed by the wrapper timer so circle replay can distinguish missing source loop edges from missing semantic extraction.

**Architecture:** Add a wrapper-local canonical loop-edge counter and use it only for structured `RCLCPP_INFO` diagnostics. The timer logs the non-published baseline snapshot, and `publishGraph()` logs each changed snapshot after graph-delta classification but before state updates. Existing ROS messages, evaluator logic, graph semantics, and rebuild behavior remain unchanged.

**Tech Stack:** ROS 2 Kilted, C++17, rclcpp, GoogleTest, CMake, colcon, Nix dev shell.

## Global Constraints

- Modify only `orb_slam3_wrapper` tracked source and test files for this diagnostic change.
- Do not edit the vendored ORB-SLAM3 submodule.
- Do not change evaluator criteria, metrics schemas, camera or ORB parameters, ROS messages, or replay tooling.
- Preserve the graph timer as the sole consumer of `SlamBackend::mapChanged()`.
- `raw_loop_edges` counts canonical unordered keyframe-ID pairs; reciprocal loop-edge entries count once.
- Baseline logging must not publish a graph/event, update graph revision state, or trigger a rebuild.
- Changed-snapshot logging must occur after `classifyGraphDeltaEvidence()` and before `previous_graph_` or `last_graph_revision_` is updated.
- Log records use the exact `graph_observation` prefix and fields specified below.
- Do not stage, modify, or delete existing unrelated untracked scratch files or generated artifacts.

---

### Task 1: Log canonical graph-observation evidence

**Files:**
- Modify: `orb_slam3_wrapper/src/wrapper_node.cpp`
- Modify: `orb_slam3_wrapper/test/wrapper_component_test.cpp`

**Interfaces:**
- Consumes: `ORB_SLAM3::GraphSnapshot::keyframes`, each `KeyframeSnapshot::id`, and `KeyframeSnapshot::loop_edge_ids`.
- Consumes: `classifyGraphDeltaEvidence(previous_graph_, graph) -> GraphDeltaEvidence`.
- Produces: wrapper log records:
  - `graph_observation stage=baseline revision=<n> raw_loop_edges=<n> previous_baseline=false`
  - `graph_observation stage=changed revision=<n> raw_loop_edges=<n> previous_baseline=true extracted_loop_edges=<n>`

- [ ] **Step 1: Add a failing unit-level regression for canonical raw edge count**

Expose a test-only static helper declaration in `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`:

```cpp
std::size_t canonicalLoopEdgeCountForTest(const ORB_SLAM3::GraphSnapshot& graph);
```

Add this test after `GraphLoopEdgesAreCanonicalizedAndDeduplicated` in `orb_slam3_wrapper/test/wrapper_component_test.cpp`:

```cpp
TEST_F(WrapperComponentTest, CanonicalLoopEdgeCountDeduplicatesReciprocalEdges) {
  ORB_SLAM3::KeyframeSnapshot first;
  first.id = 20;
  first.loop_edge_ids = {10, 30};
  ORB_SLAM3::KeyframeSnapshot second;
  second.id = 10;
  second.loop_edge_ids = {20};
  ORB_SLAM3::KeyframeSnapshot third;
  third.id = 30;
  third.loop_edge_ids = {20};
  ORB_SLAM3::GraphSnapshot graph;
  graph.keyframes = {first, second, third};

  EXPECT_EQ(orb_slam3_wrapper::canonicalLoopEdgeCountForTest(graph), 2u);
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam3_wrapper'
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam3_wrapper --ctest-args -R wrapper_component_test --output-on-failure'
```

Expected: compilation fails because `canonicalLoopEdgeCountForTest` is not declared or defined.

- [ ] **Step 3: Add the canonical loop-edge counter**

In the anonymous namespace of `orb_slam3_wrapper/src/wrapper_node.cpp`, add:

```cpp
std::size_t canonicalLoopEdgeCount(const ORB_SLAM3::GraphSnapshot& graph) {
  std::set<std::pair<std::uint64_t, std::uint64_t>> edges;
  for (const auto& keyframe : graph.keyframes) {
    for (const auto id : keyframe.loop_edge_ids) {
      edges.emplace(std::min(keyframe.id, id), std::max(keyframe.id, id));
    }
  }
  return edges.size();
}
```

Outside the anonymous namespace in `orb_slam3_wrapper`, define the test-only wrapper:

```cpp
std::size_t canonicalLoopEdgeCountForTest(const ORB_SLAM3::GraphSnapshot& graph) {
  return canonicalLoopEdgeCount(graph);
}
```

This counter does not mutate the graph or publish anything.

- [ ] **Step 4: Log the baseline and changed graph observations**

In `WrapperNode::pollGraphChanges()`, immediately after baseline assignment and before setting `graph_baseline_captured_`, add:

```cpp
RCLCPP_INFO(get_logger(),
            "graph_observation stage=baseline revision=%lu raw_loop_edges=%zu previous_baseline=false",
            static_cast<unsigned long>(previous_graph_->revision),
            canonicalLoopEdgeCount(*previous_graph_));
```

In `WrapperNode::publishGraph()`, immediately after:

```cpp
const auto evidence = classifyGraphDeltaEvidence(previous_graph_, graph);
```

and before either loop over `evidence`, add:

```cpp
RCLCPP_INFO(get_logger(),
            "graph_observation stage=changed revision=%lu raw_loop_edges=%zu previous_baseline=%s extracted_loop_edges=%zu",
            static_cast<unsigned long>(graph.revision), canonicalLoopEdgeCount(graph),
            previous_graph_ ? "true" : "false",
            evidence.loop_edges.size());
```

Do not change publication, event, state-update, or classification code.

- [ ] **Step 5: Run the focused regression and verify GREEN**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam3_wrapper'
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam3_wrapper --ctest-args -R wrapper_component_test --output-on-failure'
```

Expected: all component tests pass, including `CanonicalLoopEdgeCountDeduplicatesReciprocalEdges`.

- [ ] **Step 6: Run graph semantics coverage**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam3_wrapper --ctest-args -R "wrapper_component_test|graph_semantics_test" --output-on-failure'
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
  docs/superpowers/plans/2026-07-20-wrapper-graph-observation-diagnostics.md
git commit -m "feat: log graph observation evidence"
```

Expected: the commit contains only the diagnostic counter, structured logs,
unit regression, and this implementation plan. Existing unrelated untracked
files and generated artifacts remain untouched.
