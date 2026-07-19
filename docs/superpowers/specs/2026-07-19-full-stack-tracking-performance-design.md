# Full-Stack Tracking Performance Benchmark Design

**Status:** Draft for user review; implementation not started

## Goal

Measure the tracking-throughput cost of the full stack at the point where
ORB-SLAM3 itself first cannot keep up with accelerated bag playback. The
full-stack run passes when it preserves at least 80% of the ORB-wrapper-only
tracking rate at that same replay speed.

This work is scoped to the `fix/full-stack-track-rate-perf` branch. It includes
the existing metrics-recorder performance changes and the benchmark tooling
needed to validate them. It excludes ORB-SLAM3 loop closure, mapping behavior,
mapper algorithm changes, dashboard changes, and SLAM-quality tuning.

## Background

The existing `bag_replay.launch.py` always starts the ORB wrapper, lidar mapper,
and metrics recorder. The metrics recorder stores ROS header timestamps, which
represent bag/simulation time and therefore cannot measure wall-clock throughput
under saturation. A dedicated wall-clock observer is required for a fair
comparison that does not make the metrics recorder part of the ORB-only
baseline.

The current uncommitted recorder changes remove known callback-path costs:

- Compact NumPy occupancy-grid snapshots and vectorized map operations.
- Background, rate-limited map-revision PNG writing.
- One persistent JSONL event-file handle.
- A four-thread executor with separated tracking, map, and miscellaneous
  callback groups.

## Benchmark Topology

Both modes use the same immutable bag, playback rate, ROS domain isolation,
replay TF support, ORB wrapper parameters, and tracking-rate probe.

### ORB-only baseline

```text
ros2 bag play
required static/replay TF support
orb_slam3_wrapper
tracking-rate probe
```

The baseline omits the lidar mapper, metrics recorder, and dashboard.

### Full-stack comparison

```text
ros2 bag play
required static/replay TF support
orb_slam3_wrapper
orb_lidar_mapper
metrics_recorder
tracking-rate probe
```

The dashboard remains disabled in both benchmark modes. This isolates the
combined cost of mapping and recording from dashboard HTTP/rendering work; the
performance changes under validation are in the recorder.

## Tracking-Rate Probe

Add a small read-only ROS node that subscribes only to
`/orb_slam3/tracked_frame`.

The probe uses `time.monotonic()` for timing, never message header timestamps.
It begins measurement at the first `TrackedFrame` with `tracking_state == OK`.
After initialization it counts every received `TrackedFrame`, regardless of
later tracking state. It additionally counts OK frames so tracking health is
reported separately from throughput.

At orderly replay shutdown, the probe writes `tracking_benchmark.json`:

```json
{
  "mode": "orb_only",
  "playback_rate": 3.0,
  "wall_duration_s": 201.4,
  "received_frames": 16112,
  "tracking_fps": 80.0,
  "ok_frames": 16095,
  "ok_ratio": 0.9989,
  "initialized": true
}
```

`tracking_fps` is `received_frames / wall_duration_s`. The observer must avoid
logging per frame, image processing, trajectory collection, disk writes before
shutdown, and subscriptions other than `TrackedFrame`.

An individual run is invalid rather than passing when any of these conditions
occur:

- ORB never initializes.
- No frames are received after initialization.
- The eligible wall-clock measurement duration is below the configured minimum.
- The result file cannot be written.

## Rate Sweep And Acceptance Gate

The source camera rate is a configurable benchmark input; its initial value is
30 Hz. The sweep begins at 2x playback and advances in whole 1x steps.

For each ORB-only trial at playback rate `r`:

```text
expected_fps = source_camera_hz * r
```

The benchmark selects the first trial where:

```text
orb_only_tracking_fps < expected_fps
```

The selected rate is the stress point. The runner then executes exactly one
full-stack trial at that same rate. It passes when:

```text
full_stack_tracking_fps >= 0.80 * orb_only_tracking_fps
```

Example:

```text
source rate:             30 Hz
selected replay rate:    3x
expected source rate:    90 FPS
ORB-only measured rate:  80 FPS
full-stack pass gate:    64 FPS
```

The result reports both expected FPS and measured ORB-only FPS. The expected
rate explains where ORB first saturates; it is not itself the full-stack gate.

The runner must fail clearly if no saturation point is found before its explicit
maximum replay-rate argument. It must also stop if an ORB-only run is invalid.

## Benchmark Artifacts

Each individual output directory contains:

```text
tracking_benchmark.json
```

The orchestrating script writes a comparison result that includes:

- Source camera rate and maximum allowed playback rate.
- All ORB-only sweep results and invalid-run reasons.
- Selected stress-point rate and expected FPS.
- ORB-only tracking FPS.
- Full-stack tracking FPS.
- Calculated 80% threshold.
- Boolean pass/fail result.

The full-stack output retains its normal metrics-recorder artifacts, allowing
artifact completeness and tracking health to be reviewed alongside throughput.

## Implementation Boundaries

The benchmark launch path must share creation of the replay TF support and ORB
wrapper with the existing bag replay launch. It must add mode selection rather
than duplicate the wrapper configuration or bag-player command.

The probe should remain independent of `metrics_recorder.py`; it has a narrow
input, an explicit JSON output, and no map or image dependencies. The runner
is responsible for sequential execution, fresh ROS domains or non-overlapping
replays, collecting result files, and calculating the sweep/gate decision.

## Test Plan

Add focused automated coverage for:

- Probe start at the first OK frame and wall-clock FPS arithmetic with injected
  monotonic times.
- Counting all frames after initialization while separately reporting OK ratio.
- Invalid result cases: no initialization, no eligible frames, insufficient
  duration, and write failure.
- Sweep selection chooses the first ORB-only result below `source_rate * rate`.
- The full-stack gate passes at exactly 80% and fails below it.
- Launch composition: ORB-only excludes mapper and recorder; full-stack
  includes both; dashboard is disabled in benchmark runs.
- Existing recorder performance tests continue to pass.

Verification after implementation requires the focused unit tests plus an
actual sequential bag sweep. The final report must include the selected rate,
the two measured FPS values, and the computed threshold. No loop-closure or map
acceptance result is part of this performance gate.
