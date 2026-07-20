# Wrapper Graph Observation Diagnostics Design

## Goal

Expose the graph snapshot evidence seen by the wrapper timer during circle
replay without changing ROS messages, evaluator criteria, metrics schemas, or
graph-processing behavior.

## Scope

The change is limited to `orb_slam3_wrapper` structured logging and unit
coverage. Existing replay capture already writes wrapper stdout and stderr to
`orb_slam3_wrapper.log`, so no bringup or tooling change is required.

## Records

The wrapper emits stable, grep-friendly records with the `graph_observation`
prefix:

```text
graph_observation stage=baseline revision=<n> raw_loop_edges=<n> previous_baseline=false
graph_observation stage=changed revision=<n> raw_loop_edges=<n> previous_baseline=true extracted_loop_edges=<n>
```

The baseline record is emitted when the timer captures its non-published
snapshot. The changed record is emitted after graph-delta classification for
the changed snapshot and before the existing graph publication state is
updated.

## Counts

`raw_loop_edges` is the number of canonical unordered keyframe-ID pairs in the
snapshot's reciprocal loop-edge data. Reciprocal entries count once. The
wrapper uses the same canonical pair semantics as its existing graph publisher.

`extracted_loop_edges` is the count of loop-edge evidence returned by
`classifyGraphDeltaEvidence()` for that changed snapshot.

## Constraints

The diagnostics do not publish a ROS message, change `previous_graph_`,
change graph revisions, emit semantic events, or trigger downstream rebuilds.
The graph timer remains the only consumer of `SlamBackend::mapChanged()`.

## Verification

Add focused unit coverage for canonical raw loop-edge counting, including
reciprocal deduplication. Build and run the wrapper component and graph
semantics tests. After independent review, run the existing rate-one,
three-replay circle evaluation unchanged and inspect each wrapper log for the
records above before drawing a root-cause conclusion.
