# Tracking throughput: decoupling monitors from the ORB tracking thread

Status: investigation + plan (no code changes yet)
Branch: `perf/tracking-throughput-investigation`
Date: 2026-07-21

## Problem

On a 32-thread workstation, ORB-SLAM3 stereo tracking does not hold 30 Hz
during full-stack dashboard runs. It drops, sometimes into single digits,
while total CPU usage is only ~10-15% and a single core sits at ~99%.

The core compute (ORB wrapper + 2D lidar mapper + optimizer) sustains well
above 30 Hz on its own; the regression appears only when the monitoring
nodes (dashboard, metrics recorder) are attached. This was previously known
but many features/bugfixes have landed since, so the current state was
re-profiled.

## Profiling evidence (current code, full-run bag)

Per-process CPU during a full-stack + dashboard run:

| process              | %CPU     | interpretation                         |
|----------------------|----------|----------------------------------------|
| orb_slam3_wrapper    | ~280%    | ~3 cores; healthy multi-threading      |
| metrics_recorder     | ~106%    | one full core, GIL-bound (Python)      |
| dashboard_server     | ~100%    | one full core, GIL-bound (Python)      |
| orb_lidar_mapper     | ~10%     | idle-ish                               |
| ros2 bag play        | ~10%     | idle-ish                               |

Total active ≈ 5-6 cores of 32 → ~15% system CPU, with individual Python
processes pegged at ~100% of one core. This matches the reported symptom
("10-15% total, one core at 99%"): the "99% core" is a GIL-bound Python
monitor, not the wrapper.

Tracking rate vs dashboard load (full stack + dashboard), `/state` `hz`:

| sample | track Hz | dashboard_server %CPU |
|--------|----------|-----------------------|
| steady | 28-30    | 5-58%                 |
| burst  | 22.7     | 102%                  |
| burst  | 10.3     | 96%                   |
| burst  | 9.3      | 102%                  |
| burst  | 8.3      | 101%                  |

The tracking crater is tightly correlated with dashboard_server saturating
its core (100%).

Isolation run — full stack with `--no-dashboard` (metrics_recorder still at
~106%), tracking rate measured directly via `ros2 topic hz
/orb_slam3/tracked_frame`:

```
average rate: 30.000   min 0.027s  max 0.040s  std dev 0.00178s  window 389
average rate: 29.998   min 0.027s  max 0.040s  std dev 0.00177s  window 419
```

Rock-solid 30 Hz. Conclusion: **metrics_recorder is NOT the cause;
dashboard_server is.** metrics_recorder wastes a core but does not stall
tracking.

## Root cause

Two single-threaded executors plus reliable QoS create a transitive
backpressure chain from the dashboard back into the wrapper's tracking loop.

- `dashboard_server.py` runs `rclpy.spin(node)` (single-threaded executor).
  All subscription callbacks and the `_render_map` timer share one GIL-bound
  thread. It subscribes RELIABLE to the map, map_revision, corrected_path
  (RevisionedPath = all poses) and events. During loop-closure bursts the
  mapper emits many large OccupancyGrid + full-path messages; draining and
  converting them (plus periodic PNG encoding on the same thread) saturates
  the single thread to 100%.

- `orb_slam3_wrapper` runs `rclcpp::spin` (single-threaded executor) in
  `main.cpp`. The SAME thread runs `TrackStereo` (the stereo image callback)
  AND the 50 ms `graph_timer` that publishes `graph_snapshot` (RELIABLE,
  transient_local), `path`, `keyframes`, `loops`.

Transitive chain when the dashboard saturates:
1. dashboard_server stops draining the mapper's RELIABLE `/orb_lidar/map`
   and `/orb_lidar/corrected_path_revisioned`.
2. The mapper's reliable writers backpressure, so the mapper spends time
   retrying/blocking and stops promptly draining the wrapper's RELIABLE
   `/orb_slam3/graph_snapshot`.
3. The wrapper's `graph_snapshot` `publish()` (executed on the single spin
   thread inside the graph timer) blocks up to the reliable
   `max_blocking_time` (~100 ms default).
4. Because tracking (`TrackStereo`) runs on that same spin thread, it is
   starved. ~100 ms stalls → ~10 Hz, matching the observed single-digit
   craters.

So the wrapper is not CPU-starved (32 cores, only ~6 busy). It is
*serialization-starved*: a downstream reliable-reader stall propagates,
through reliable QoS, into the one thread that also does tracking.

Note: this investigation raised `nFeatures` to 2000 earlier, which increases
per-frame tracking cost and shrinks the 33 ms budget — a secondary contributor,
not the cause. Consider reverting to 1400 if the budget is tight.

## Confirmation still owed (one experiment)

The transitive chain (step 1-4) is the best-supported mechanism but the exact
backpressure edge was inferred, not instrumented. Before/while implementing,
confirm by one of:
- Temporarily set the wrapper's `graph_snapshot`/viz publishers to BEST_EFFORT
  and re-run with dashboard; if tracking stops cratering, the reliable-publish
  stall is confirmed.
- Or add timing around `graph_pub_->publish(...)` and log stalls > 5 ms.

## Optimization plan (phased, ranked by impact / low risk first)

### Phase 1 — Decouple the wrapper tracking thread (highest impact)
Goal: a slow/blocked publish can never stall `TrackStereo`.
- Run the wrapper on a `MultiThreadedExecutor` with the stereo image
  callback in its own `MutuallyExclusiveCallbackGroup`, separate from the
  `graph_timer` and info callbacks. Tracking then progresses even if a
  publish blocks on another thread.
- And/or move `graph_snapshot`/`path`/`keyframes`/`loops` publishing to a
  dedicated worker thread (like the existing `LatestImageWorker` pattern for
  the tracking image), so serialization + publish never touch the tracking
  thread.
- Keep `graph_snapshot` RELIABLE (the mapper needs it), but bound blocking:
  it must not run on the tracking thread.

### Phase 2 — Keep dashboard_server from saturating
Goal: the read-only viewer drains promptly and drops stale data.
- Replace `rclpy.spin` with a `MultiThreadedExecutor`; put heavy map/path
  callbacks in a separate callback group from lightweight ones and the HTTP
  handlers.
- Switch map / corrected_path / events subscriptions to BEST_EFFORT,
  KEEP_LAST depth 1 (a live viewer only needs the latest; dropping stale is
  correct and removes any backpressure it can exert upstream).
- Move PNG encoding off the executor thread entirely (dedicated worker), and
  keep decimating (already 1 Hz).
- Only process the latest map/path (coalesce), skip intermediate revisions.

### Phase 3 — Trim metrics_recorder steady load (lower priority)
Does not crater tracking, but burns a full core.
- Decimate/lighten `_on_tracked_frame` (runs at 30 Hz); ensure all artifact
  I/O (PNG, JSONL) is off the subscription threads.
- Confirm best-effort on high-rate inputs; keep reliable only where a metric
  needs every sample (events).

### Phase 4 — Optional config
- Revert `ORBextractor.nFeatures` 2000 -> 1400 if the per-frame budget is
  tight after decoupling (2000 was a mapping-quality experiment, not required
  for the crash fixes).

## Verification
- Re-run full stack + dashboard on the full-run bag.
- Measure `ros2 topic hz /orb_slam3/tracked_frame` and dashboard `/state` `hz`
  through the loop-closure bursts; target sustained >= 28-30 Hz with no
  single-digit craters.
- Sample per-process CPU; dashboard_server should no longer coincide with
  tracking drops even if it briefly saturates.
- Regression guard: keep the `--no-dashboard` 30 Hz baseline as the reference.

## Files in scope
- `orb_slam3_wrapper/src/main.cpp` (executor)
- `orb_slam3_wrapper/src/wrapper_node.cpp` (callback groups, publish threading, QoS)
- `orb_slam_bringup/orb_slam_bringup/dashboard_server.py` (executor, QoS, PNG worker)
- `orb_slam_bringup/orb_slam_bringup/metrics_recorder.py` (phase 3, optional)
