# Wrapper Graph Session Reset Design

## Goal

Prevent graph semantic comparisons from spanning ORB-SLAM3 backend instances
when a changed CameraInfo calibration causes the backend to be reconfigured.

## Scope

The change is limited to wrapper graph-observation state. It does not modify
the ORB-SLAM3 vendor submodule, evaluator criteria, camera or ORB parameters,
or graph-semantic classification rules.

## Session Boundary

An effective CameraInfo change is the graph-observation session boundary. Both
`WrapperNode::infoCallback()` and `WrapperNode::setCameraInfoForTest()` already
invalidate the calibration and set `backend_configured_` false when calibration
input changes. That path can recreate `OrbSlam3Backend`'s `ORB_SLAM3::System`.

At the same boundary, the wrapper clears the graph-observation state:

```cpp
graph_baseline_captured_ = false;
previous_graph_.reset();
last_graph_revision_ = 0;
```

The reset is performed only for an effective calibration change, not for an
identical CameraInfo message.

## Behavior After Reconfiguration

Until calibration again succeeds and a tracked-frame header exists, the graph
timer does not access the backend. On the next ready timer pass it captures a
new non-published baseline for the recreated backend. The first later changed
snapshot is compared only with that new baseline and is not skipped because its
revision overlaps a revision from the previous backend instance.

The reset itself publishes no graph/event and triggers no downstream rebuild.

## Regression Coverage

The component regression establishes a published graph at a nonzero revision,
changes CameraInfo to force backend reconfiguration, and supplies a new-session
baseline with `changed` false. It then supplies a changed graph using a
revision that would have been filtered by the previous session's revision. The
test verifies that graph publishes and emits semantic evidence from the new
baseline, proving stale snapshots and revisions cannot cross the session
boundary.

## Verification

Build the wrapper and run the focused component test before and after the
change, then run `wrapper_component_test|graph_semantics_test`. Review the
diff for scope and whitespace before committing only the wrapper task files
and implementation plan.
