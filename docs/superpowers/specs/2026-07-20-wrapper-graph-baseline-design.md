# Wrapper Graph Baseline Design

## Goal

Retain an initial ORB-SLAM3 graph snapshot without publishing it so the first
subsequent graph change can be classified for newly introduced loop edges.

## Scope

The change is limited to `orb_slam3_wrapper`. It does not modify the ORB-SLAM3
vendor submodule, camera or ORB parameters, evaluator criteria, or map rebuild
behavior.

## Behavior

`WrapperNode::pollGraphChanges()` remains the only consumer of
`SlamBackend::mapChanged()`. Once a backend is configured and a tracked-frame
header is available, it captures exactly one `SlamBackend::graphSnapshot()` as
the previous graph before its first `mapChanged()` call.

The captured baseline is retained exclusively for delta classification. It
does not publish `/orb_slam3/graph_snapshot`, publish semantic events, update
`last_graph_revision_`, or trigger a downstream rebuild.

After the baseline is captured, the timer's existing changed-snapshot flow is
unchanged: when `mapChanged()` returns true and the graph revision differs from
`last_graph_revision_`, `publishGraph()` publishes the graph and classifies it
against the retained baseline. A reciprocal loop edge introduced in that first
changed snapshot therefore emits one canonical `LOOP_CLOSED` event.

## State

Add a boolean initialized to false to record whether the one-time baseline was
captured. The existing `previous_graph_` optional stores both that baseline and
each subsequently published graph, preserving the current delta chain after
the first publication.

## Regression Coverage

Add a component test that processes one successful stereo frame, spins the
timer while the fake backend reports no graph change, and verifies that no
graph or event was published. It then changes the fake graph to revision 1
with reciprocal loop edges, reports a graph change, and verifies exactly one
published graph and one `LOOP_CLOSED` event.

The test must fail before the baseline behavior exists and pass afterward.

## Verification

Build `orb_slam3_wrapper`, run the focused component regression, then run the
`wrapper_component_test|graph_semantics_test` CTest selection. After code
review, run the rate-one circle replay with boundary diagnostics before making
any end-to-end loop-closure acceptance claim.
