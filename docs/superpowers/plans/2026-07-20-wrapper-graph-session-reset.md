# Wrapper Graph Session Reset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reset graph-observation state when changed CameraInfo causes an ORB-SLAM3 backend reconfiguration, preventing semantic graph comparisons from spanning backend instances.

**Architecture:** An effective CameraInfo change already invalidates wrapper calibration and makes the backend unconfigured. At that exact boundary, clear the baseline-capture flag, previous graph snapshot, and last published graph revision. Once configuration and tracking resume, the existing timer captures a new non-published baseline before processing new-session graph changes.

**Tech Stack:** ROS 2 Kilted, C++17, rclcpp, GoogleTest, CMake, colcon, Nix dev shell.

## Global Constraints

- Modify only `orb_slam3_wrapper` tracked source and test files for the behavioral change.
- Do not edit the vendored ORB-SLAM3 submodule.
- Do not change evaluator criteria or tune camera or ORB parameters.
- Preserve the existing graph timer as the sole consumer of `SlamBackend::mapChanged()`.
- An effective CameraInfo change resets exactly `graph_baseline_captured_`, `previous_graph_`, and `last_graph_revision_`.
- Identical CameraInfo must not reset graph-observation state or reconfigure the backend.
- Resetting graph-observation state must not publish a graph/event or trigger a downstream rebuild.
- The first changed graph after reconfiguration must be compared with a new non-published baseline and must not be skipped by a prior-session graph revision.
- Follow TDD: demonstrate the focused regression fails before production code is written, then pass it and run focused wrapper coverage.
- Do not stage, modify, or delete existing unrelated untracked scratch files or generated artifacts.

---

### Task 1: Reset graph observation at calibration invalidation

**Files:**
- Modify: `orb_slam3_wrapper/src/wrapper_node.cpp`
- Modify: `orb_slam3_wrapper/test/wrapper_component_test.cpp`

**Interfaces:**
- Consumes: effective CameraInfo changes through `infoCallback()` and `setCameraInfoForTest()`.
- Consumes: existing graph state `graph_baseline_captured_`, `previous_graph_`, and `last_graph_revision_`.
- Produces: the next ready timer pass captures a new baseline and publishes a changed graph whose revision overlaps the previous backend session.

- [ ] **Step 1: Add the failing reconfiguration regression**

Add this test next to `FirstChangedGraphWithLoopEdgeEmitsLoopClosed` in `orb_slam3_wrapper/test/wrapper_component_test.cpp`:

```cpp
TEST_F(WrapperComponentTest, ReconfigurationResetsGraphObservationSession) {
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

  backend_ptr->graph.revision = 9;
  backend_ptr->graph.active_map_id = 17;
  backend_ptr->changed = true;
  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));

  auto changed_left = info();
  changed_left.width = 847;
  auto changed_right = info(true);
  changed_right.width = 847;
  node->setCameraInfoForTest(changed_left, changed_right);
  backend_ptr->changed = false;
  backend_ptr->graph.revision = 0;
  backend_ptr->graph.active_map_id = 23;
  backend_ptr->graph.keyframes.clear();
  node->processStereoForTest(image, image);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  rclcpp::spin_some(node);
  EXPECT_EQ(node->graphPublishCountForTest(), 1u);

  ORB_SLAM3::KeyframeSnapshot first;
  first.id = 20;
  first.map_id = 23;
  first.loop_edge_ids = {10};
  ORB_SLAM3::KeyframeSnapshot second;
  second.id = 10;
  second.map_id = 23;
  second.loop_edge_ids = {20};
  backend_ptr->graph.revision = 9;
  backend_ptr->graph.keyframes = {first, second};
  backend_ptr->changed = true;

  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 2u; }));
  EXPECT_EQ(node->lastGraphSnapshotForTest().revision, 9u);
  EXPECT_EQ(node->lastTrackingEventForTest().type,
            orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
}
```

The first session publishes revision 9. The changed CameraInfo forces another successful fake configuration; the revision-0 snapshot is the non-published new-session baseline. The new-session revision 9 must still publish and emit the reciprocal loop edge, proving the old session revision cannot suppress it.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam3_wrapper'
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam3_wrapper --ctest-args -R wrapper_component_test --output-on-failure'
```

Expected: `ReconfigurationResetsGraphObservationSession` fails because the existing `last_graph_revision_` is still 9, so the final changed graph is not published. The test must not proceed to production code unless this focused failure occurs.

- [ ] **Step 3: Reset all graph-observation session state on changed CameraInfo**

In both changed-calibration branches of `orb_slam3_wrapper/src/wrapper_node.cpp`, immediately after `backend_configured_ = false;`, add:

```cpp
graph_baseline_captured_ = false;
previous_graph_.reset();
last_graph_revision_ = 0;
```

The two locations are the non-identical-message paths in `infoCallback()` and `setCameraInfoForTest()`. Do not add a helper, change the identical-message early return, change `pollGraphChanges()`, or alter semantic classification.

- [ ] **Step 4: Run the focused regression and verify GREEN**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam3_wrapper'
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam3_wrapper --ctest-args -R wrapper_component_test --output-on-failure'
```

Expected: all `wrapper_component_test` cases pass, including `ReconfigurationResetsGraphObservationSession`.

- [ ] **Step 5: Run graph semantics coverage**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam3_wrapper --ctest-args -R "wrapper_component_test|graph_semantics_test" --output-on-failure'
```

Expected: both test executables pass with no failures.

- [ ] **Step 6: Inspect and commit the task files**

Run:

```bash
git diff --check
git diff -- orb_slam3_wrapper/src/wrapper_node.cpp \
  orb_slam3_wrapper/test/wrapper_component_test.cpp
git add orb_slam3_wrapper/src/wrapper_node.cpp \
  orb_slam3_wrapper/test/wrapper_component_test.cpp \
  docs/superpowers/plans/2026-07-20-wrapper-graph-session-reset.md
git commit -m "fix: reset graph state on reconfiguration"
```

Expected: the commit contains only the session-state reset, regression test, and this implementation plan. Existing unrelated untracked files and generated artifacts remain untouched.
