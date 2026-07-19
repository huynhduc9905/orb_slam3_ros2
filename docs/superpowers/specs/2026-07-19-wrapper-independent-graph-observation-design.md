# Wrapper Independent Graph Observation Design

## Problem

The wrapper currently polls `SlamBackend::mapChanged()` only from
`WrapperNode::processStereo()`, after a stereo frame is tracked. Loop-closing
work is asynchronous, so a graph change can occur without a subsequent image
callback to observe and publish it. The circle-run evaluation must observe the
post-closure graph delta and emit a `LOOP_CLOSED` event with the corresponding
loop-edge evidence.

The notification-order correction in vendor commit `601bec2` is retained. Its
focused regression passes, but the circle-run gate at
`artifacts/circle-loop-evaluation-20260719_204100` remained at zero of three
passes. Each run observed revisions 1 and 2, no loop event, and no loop-edge
evidence. The wrapper must therefore decouple graph observation from camera
traffic so it can inspect backend changes after tracking callbacks stop.

## Scope

- Add an independent, modest-rate ROS timer in `WrapperNode`.
- Create the timer and stereo subscriptions in the node's default mutually
  exclusive callback group. This serializes access to the backend and shared
  graph/tracked-frame state under a multi-threaded executor.
- After backend configuration, the timer polls `mapChanged()` and retrieves a
  snapshot only when a change is available.
- Publish a snapshot only when its revision differs from
  `last_graph_revision_`; retain existing graph-delta classification and
  publication semantics.
- Remove graph polling from `processStereo()` so the timer is the sole
  consumer of `System::MapChanged()`.
- Timer-originated graph and event messages use the most recently tracked
  frame header. If no frame has yet been tracked, the timer does not consume a
  graph change.
- Add component tests for a graph change that arrives after the last stereo
  callback, including loop-edge event publication. Tests spin the node until
  the timer callback runs rather than expecting synchronous publication from
  `processStereoForTest()`.

## Non-Goals

- Do not change vendor loop correction, matching, GBA, map merge, or feature
  extraction behavior.
- Do not alter graph-delta classifications or the loop-evidence evaluator.
- Do not add a background backend thread or poll snapshots when no map change
  is reported.

## Data Flow

1. Stereo processing tracks frames and stores the latest valid output header.
2. The graph timer returns until the backend is configured and a tracked frame
   header is available.
3. It calls `mapChanged()`. If false, it returns without copying a graph.
4. If true, it obtains a graph snapshot. A revision different from the last
   published revision is sent through the existing `publishGraph()` path.
5. `publishGraph()` emits graph, marker, and semantic loop events, then
   updates the previous snapshot and revision state.

## Failure Handling

- A repeated or stale snapshot revision is ignored after consumption.
- Before the first tracked frame, no graph polling occurs, preserving the
  first available backend change for a later timer tick with a usable header.
- Timer polling does not invoke tracking and cannot affect image
  synchronization or calibration behavior.

## Verification

- A new wrapper component regression proves an asynchronous changed graph is
  published without another call to `processStereoForTest()`. It spins the
  node until the timer runs, then verifies a canonical loop edge and
  `LOOP_CLOSED` event.
- Existing wrapper component and graph-semantic tests continue to pass.
- The focused bringup loop-evidence tests pass.
- Run the three-trial circle evaluator at rate one. Success requires at least
  two `observed_and_rebuilt` trials.
