# Full-Stack Tracking Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reproducible ORB-wrapper-only versus full-stack tracking-throughput benchmark, and validate the current metrics-recorder optimizations against the first ORB saturation point.

**Architecture:** A tiny read-only `tracking_benchmark_probe` writes a monotonic-clock result after replay shutdown. `bag_replay.launch.py` gains explicit benchmark modes so both topologies share replay TF and wrapper configuration. A pure-Python benchmark module selects the first ORB-only under-target rate and evaluates the 80% full-stack gate; a shell runner invokes those modes sequentially and writes a comparison JSON.

**Tech Stack:** ROS 2 launch/rclpy, Python 3, pytest, rosbag2 playback, Bash, JSON.

## Global Constraints

- Work only on `fix/full-stack-track-rate-perf`.
- Retain the existing uncommitted metrics-recorder optimization changes; do not revert or overwrite them.
- Do not change ORB-SLAM3 loop closure, mapping behavior, mapper algorithms, or dashboard behavior.
- Benchmark timing must use `time.monotonic()`, never ROS or bag header timestamps.
- The probe subscribes only to `/orb_slam3/tracked_frame`, does no per-frame disk I/O, and starts counting at the first `TRACK_OK` frame.
- Count every received frame after initialization; report `ok_ratio` separately.
- Baseline starts at `2x`, advances by whole `1x` rates, and selects the first ORB-only rate where `tracking_fps < source_camera_hz * playback_rate`.
- Full-stack passes only when `full_stack_tracking_fps >= 0.80 * orb_only_tracking_fps` at the selected rate.
- Benchmark runs must disable the dashboard in both modes.
- Invalid runs are failures: no initialization, zero eligible frames, eligible duration below the configured minimum, or unable to write a result.

---

## File Structure

| File | Responsibility |
|---|---|
| `orb_slam_bringup/orb_slam_bringup/tracking_benchmark.py` | Pure result model, monotonic frame counter, JSON serialization, sweep selection, and gate calculation. |
| `orb_slam_bringup/orb_slam_bringup/tracking_benchmark_probe.py` | Thin ROS node that forwards `TrackedFrame` callbacks to the pure counter and writes one result at shutdown. |
| `orb_slam_bringup/launch/bag_replay.launch.py` | Add `benchmark_mode` and probe configuration; compose ORB-only and full-stack graphs from the existing shared replay setup. |
| `orb_slam_bringup/setup.py` | Install the probe console entry point. |
| `tools/run_tracking_performance_benchmark.sh` | Sequential ORB-only sweep, selected-rate full-stack execution, and comparison artifact writer. |
| `orb_slam_bringup/test/test_tracking_benchmark.py` | Unit tests for counter behavior, invalid results, sweep selection, and 80% gate. |
| `orb_slam_bringup/test/test_bag_profile.py` | Launch-source regression assertions for benchmark arguments and mode composition. |
| `README.md` | Document prerequisites, benchmark command, artifacts, selection rule, and 80% gate. |

## Task 1: Add Pure Benchmark Measurement And Decision Logic

**Files:**
- Create: `orb_slam_bringup/orb_slam_bringup/tracking_benchmark.py`
- Create: `orb_slam_bringup/test/test_tracking_benchmark.py`

**Interfaces:**
- Produces: `TrackingRateCounter`, `BenchmarkResult`, `load_result(path: Path) -> BenchmarkResult`, `select_stress_point(results, source_camera_hz) -> BenchmarkResult`, and `compare_full_stack(baseline, full_stack) -> dict[str, object]`.
- Consumed by: `tracking_benchmark_probe.py` and `run_tracking_performance_benchmark.sh` through `python3 -m orb_slam_bringup.tracking_benchmark`.

- [ ] **Step 1: Write failing measurement-counter tests**

```python
from orb_slam_bringup.tracking_benchmark import TrackingRateCounter


def test_counter_starts_at_first_ok_and_counts_every_later_frame():
    counter = TrackingRateCounter(mode="orb_only", playback_rate=3.0, min_duration_s=1.0)

    counter.observe(tracking_ok=False, now_s=10.0)
    counter.observe(tracking_ok=True, now_s=11.0)
    counter.observe(tracking_ok=False, now_s=11.25)
    counter.observe(tracking_ok=True, now_s=11.50)

    result = counter.finish(now_s=12.0)

    assert result.initialized is True
    assert result.received_frames == 3
    assert result.ok_frames == 2
    assert result.wall_duration_s == 1.0
    assert result.tracking_fps == 3.0
    assert result.ok_ratio == 2 / 3
    assert result.invalid_reason is None
```

- [ ] **Step 2: Write failing invalid-result and serialization tests**

```python
from pathlib import Path

from orb_slam_bringup.tracking_benchmark import TrackingRateCounter, load_result


def test_counter_reports_invalid_when_orb_never_initializes():
    counter = TrackingRateCounter(mode="orb_only", playback_rate=2.0, min_duration_s=1.0)
    counter.observe(tracking_ok=False, now_s=4.0)

    result = counter.finish(now_s=6.0)

    assert result.invalid_reason == "orb_never_initialized"


def test_result_round_trips_as_json(tmp_path: Path):
    counter = TrackingRateCounter(mode="full_stack", playback_rate=3.0, min_duration_s=1.0)
    counter.observe(tracking_ok=True, now_s=1.0)
    counter.observe(tracking_ok=True, now_s=2.0)
    path = tmp_path / "tracking_benchmark.json"

    counter.finish(now_s=3.0).write(path)

    assert load_result(path).tracking_fps == 1.0
    assert load_result(path).mode == "full_stack"
```

- [ ] **Step 3: Write failing sweep and gate tests**

```python
from orb_slam_bringup.tracking_benchmark import (
    BenchmarkResult,
    compare_full_stack,
    select_stress_point,
)


def result(rate: float, fps: float) -> BenchmarkResult:
    return BenchmarkResult("orb_only", rate, 10.0, int(fps * 10), fps, int(fps * 10), 1.0, True, None)


def test_selects_first_orb_only_rate_below_source_target():
    selected = select_stress_point([result(2.0, 60.0), result(3.0, 80.0)], source_camera_hz=30.0)

    assert selected.playback_rate == 3.0
    assert selected.tracking_fps == 80.0


def test_full_stack_passes_at_exactly_eighty_percent():
    baseline = result(3.0, 80.0)
    full_stack = BenchmarkResult("full_stack", 3.0, 10.0, 640, 64.0, 640, 1.0, True, None)

    comparison = compare_full_stack(baseline, full_stack)

    assert comparison["threshold_fps"] == 64.0
    assert comparison["passed"] is True
```

- [ ] **Step 4: Run tests to verify failure**

Run: `pytest test/test_tracking_benchmark.py -v`

Expected: collection fails because `orb_slam_bringup.tracking_benchmark` does not exist.

- [ ] **Step 5: Implement the immutable result type and counter**

Create `tracking_benchmark.py` with these exact public types and behaviors:

`BenchmarkResult` is a frozen dataclass with these fields, in order:
`mode`, `playback_rate`, `wall_duration_s`, `received_frames`, `tracking_fps`,
`ok_frames`, `ok_ratio`, `initialized`, and `invalid_reason`. It exposes
`as_dict()` and `write(path: Path)`. `TrackingRateCounter` exposes
`__init__(mode: str, playback_rate: float, min_duration_s: float)`,
`observe(tracking_ok: bool, now_s: float)`, and `finish(now_s: float)`.

`observe()` ignores frames until the first `tracking_ok=True`; that frame is the first eligible frame and sets the start time. `finish()` computes duration using `now_s - start_s`, uses `received_frames / duration` only for valid duration, and returns these invalid reasons exactly: `orb_never_initialized`, `no_eligible_frames`, and `measurement_duration_below_minimum`.

Write JSON atomically using the existing repository `atomic_write_bytes` pattern only if importing it does not add metrics-recorder dependencies; otherwise write a temporary sibling path and replace it with `Path.replace()`.

- [ ] **Step 6: Implement decision functions and CLI**

Implement:

```python
def select_stress_point(
    results: Sequence[BenchmarkResult], source_camera_hz: float
) -> BenchmarkResult:
    for result in sorted(results, key=lambda item: item.playback_rate):
        if result.invalid_reason is not None:
            raise ValueError(f"invalid ORB-only result at {result.playback_rate}x: {result.invalid_reason}")
        if result.tracking_fps < source_camera_hz * result.playback_rate:
            return result
    raise ValueError("ORB-only saturation point was not found")

def compare_full_stack(
    baseline: BenchmarkResult, full_stack: BenchmarkResult
) -> dict[str, object]:
    if baseline.invalid_reason or full_stack.invalid_reason:
        raise ValueError("cannot compare invalid benchmark results")
    if baseline.playback_rate != full_stack.playback_rate:
        raise ValueError("baseline and full-stack playback rates differ")
    threshold = 0.80 * baseline.tracking_fps
    return {
        "playback_rate": baseline.playback_rate,
        "orb_only_tracking_fps": baseline.tracking_fps,
        "full_stack_tracking_fps": full_stack.tracking_fps,
        "threshold_fps": threshold,
        "passed": full_stack.tracking_fps >= threshold,
    }
```

Provide a `main(args: Sequence[str] | None = None) -> int` with two subcommands:

```text
python3 -m orb_slam_bringup.tracking_benchmark select \
  --source-camera-hz 30 \
  --result orb-2x/tracking_benchmark.json \
  --result orb-3x/tracking_benchmark.json

python3 -m orb_slam_bringup.tracking_benchmark compare \
  --baseline orb-3x/tracking_benchmark.json \
  --full-stack full-3x/tracking_benchmark.json \
  --output comparison.json
```

`select` prints one JSON object for the selected result and exits nonzero on no saturation or invalid input. `compare` writes JSON that includes the required gate fields and exits 0 for pass, 1 for a valid comparison that fails the 80% gate, and 2 for invalid input.

- [ ] **Step 7: Run focused tests to verify pass**

Run: `pytest test/test_tracking_benchmark.py -v`

Expected: all tests pass.

- [ ] **Step 8: Add boundary tests and rerun**

Add explicit tests for zero eligible frames, duration below minimum, no saturation found, comparison with differing rates, and a full-stack value just below 80%.

Run: `pytest test/test_tracking_benchmark.py -v`

Expected: all tests pass.

- [ ] **Step 9: Commit**

```bash
git add orb_slam_bringup/orb_slam_bringup/tracking_benchmark.py orb_slam_bringup/test/test_tracking_benchmark.py
git commit -m "feat: add tracking benchmark core"
```

## Task 2: Add The Read-Only ROS Tracking Probe

**Files:**
- Create: `orb_slam_bringup/orb_slam_bringup/tracking_benchmark_probe.py`
- Modify: `orb_slam_bringup/setup.py:46-55`
- Modify: `orb_slam_bringup/test/test_tracking_benchmark.py`

**Interfaces:**
- Consumes: `TrackingRateCounter` and `BenchmarkResult.write()` from Task 1.
- Produces: console script `tracking_benchmark_probe`, writing `<artifact_dir>/tracking_benchmark.json`.
- Consumed by: benchmark launch composition in Task 3.

- [ ] **Step 1: Write a failing probe shutdown test**

Use fake node/message objects rather than a live ROS graph:

```python
from types import SimpleNamespace

from orb_slam_bringup.tracking_benchmark_probe import TrackingBenchmarkProbe


def test_probe_writes_single_result_after_tracking_initializes(tmp_path, monkeypatch):
    now = iter([10.0, 10.5, 11.0])
    monkeypatch.setattr("orb_slam_bringup.tracking_benchmark_probe.time.monotonic", lambda: next(now))
    probe = TrackingBenchmarkProbe.for_test(
        artifact_dir=tmp_path, mode="orb_only", playback_rate=3.0, min_duration_s=0.5
    )

    probe.on_tracked_frame(SimpleNamespace(tracking_state=0))
    probe.on_tracked_frame(SimpleNamespace(tracking_state=2))
    probe.on_tracked_frame(SimpleNamespace(tracking_state=2))
    probe.flush()

    data = json.loads((tmp_path / "tracking_benchmark.json").read_text())
    assert data["received_frames"] == 2
    assert data["initialized"] is True
```

Use the actual `TRACK_OK` numeric constant imported from `orb_slam3_msgs.msg.TrackedFrame` in production code; the test must use that same value or expose it as a module constant.

- [ ] **Step 2: Run the probe test to verify failure**

Run: `pytest test/test_tracking_benchmark.py::test_probe_writes_single_result_after_tracking_initializes -v`

Expected: import failure because the probe module does not exist.

- [ ] **Step 3: Implement the probe**

Create a small `rclpy.node.Node` implementation with these parameters:

```text
artifact_dir: string
mode: string, one of orb_only or full_stack
playback_rate: double > 0
min_duration_s: double >= 0
tracked_frame_topic: string, default /orb_slam3/tracked_frame
```

The node must:

- Construct `TrackingRateCounter(mode, playback_rate, min_duration_s)`.
- Subscribe with `qos_profile_sensor_data` to the configured tracked-frame topic.
- Call `counter.observe(msg.tracking_state == TrackedFrame.OK, time.monotonic())` from the callback.
- Register a shutdown callback and make `flush()` idempotent with a boolean guard.
- On flush, call `counter.finish(time.monotonic()).write(artifact_dir / "tracking_benchmark.json")`.
- Log one summary line at flush, never per frame.
- Let a result-write failure propagate to process failure after logging the exception; it must not silently report a benchmark pass.

Provide `for_test()` as a classmethod that bypasses ROS node construction and builds the same counter/paths, so tests do not require `rclpy.init()`.

`main()` uses `rclpy.spin(node)`, flushes in `finally`, destroys the node, and calls `rclpy.shutdown()`.

- [ ] **Step 4: Register the console script**

In `setup.py`, add exactly:

```python
"tracking_benchmark_probe = orb_slam_bringup.tracking_benchmark_probe:main",
```

after the existing `metrics_recorder` entry point.

- [ ] **Step 5: Run probe and core tests**

Run: `pytest test/test_tracking_benchmark.py -v`

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add orb_slam_bringup/orb_slam_bringup/tracking_benchmark_probe.py orb_slam_bringup/orb_slam_bringup/tracking_benchmark.py orb_slam_bringup/setup.py orb_slam_bringup/test/test_tracking_benchmark.py
git commit -m "feat: add tracking rate benchmark probe"
```

## Task 3: Add Shared Benchmark Launch Modes

**Files:**
- Modify: `orb_slam_bringup/launch/bag_replay.launch.py:137-443`
- Modify: `orb_slam_bringup/test/test_bag_profile.py`

**Interfaces:**
- Consumes: `tracking_benchmark_probe` from Task 2.
- Produces: `benchmark_mode:=off|orb_only|full_stack`, `benchmark_min_duration_s`, and shared mode-specific node composition.
- Consumed by: the shell runner in Task 4.

- [ ] **Step 1: Write failing static launch-composition tests**

Add tests that inspect the launch source, following existing launch-source tests:

```python
def test_bag_replay_declares_explicit_tracking_benchmark_modes():
    text = BAG_REPLAY_LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"benchmark_mode"' in text
    assert '"benchmark_min_duration_s"' in text
    assert 'tracking_benchmark_probe' in text
    assert '"orb_only"' in text
    assert '"full_stack"' in text


def test_benchmark_launch_keeps_dashboard_disabled_by_mode():
    text = BAG_REPLAY_LAUNCH_PATH.read_text(encoding="utf-8")
    assert "benchmark modes do not start the dashboard" in text
```

- [ ] **Step 2: Run launch tests to verify failure**

Run: `pytest test/test_bag_profile.py -v`

Expected: the new benchmark assertions fail.

- [ ] **Step 3: Add launch arguments and validate mode**

In `_setup`, read these configurations:

```python
benchmark_mode = LaunchConfiguration("benchmark_mode").perform(context).strip().lower()
benchmark_min_duration_s = float(
    LaunchConfiguration("benchmark_min_duration_s").perform(context)
)
if benchmark_mode not in ("off", "orb_only", "full_stack"):
    raise RuntimeError("benchmark_mode must be one of: off, orb_only, full_stack")
if benchmark_min_duration_s < 0.0:
    raise RuntimeError("benchmark_min_duration_s must be nonnegative")
```

Declare the arguments with defaults:

```python
DeclareLaunchArgument("benchmark_mode", default_value="off"),
DeclareLaunchArgument("benchmark_min_duration_s", default_value="10.0"),
```

The standard existing behavior must be unchanged when `benchmark_mode:=off`.

- [ ] **Step 4: Compose the two benchmark graphs without duplicating wrapper/replay setup**

Keep the existing static TF, replay-only odom adapter, TF audit, wrapper, bag player, and shutdown handler shared.

Apply these rules around existing node additions:

```text
benchmark_mode == off:
  retain existing mapper + metrics recorder behavior
  honor start_dashboard as today

benchmark_mode == orb_only:
  omit mapper and metrics recorder
  omit dashboard regardless of start_dashboard
  add tracking_benchmark_probe(mode=orb_only)

benchmark_mode == full_stack:
  add mapper and metrics recorder
  omit dashboard regardless of start_dashboard
  add tracking_benchmark_probe(mode=full_stack)
```

The probe receives `artifact_dir`, `mode`, `playback_rate=float(rate)`,
`min_duration_s=benchmark_min_duration_s`, and `use_sim_time=True`. Add a
single `LogInfo` line stating that benchmark modes do not start the dashboard.

Do not add the metrics-recorder report-check gate to ORB-only mode. Existing
full-stack artifacts remain available in `full_stack` mode.

- [ ] **Step 5: Run launch-source tests**

Run: `pytest test/test_bag_profile.py -v`

Expected: all tests pass.

- [ ] **Step 6: Verify launch argument parsing without replaying the bag**

Run: `ros2 launch orb_slam_bringup bag_replay.launch.py --show-args`

Expected: output lists `benchmark_mode` and `benchmark_min_duration_s` with their documented defaults.

- [ ] **Step 7: Commit**

```bash
git add orb_slam_bringup/launch/bag_replay.launch.py orb_slam_bringup/test/test_bag_profile.py
git commit -m "feat: add ORB-only benchmark launch mode"
```

## Task 4: Add The Sequential Sweep Runner

**Files:**
- Create: `tools/run_tracking_performance_benchmark.sh`
- Modify: `orb_slam_bringup/test/test_tracking_benchmark.py`

**Interfaces:**
- Consumes: launch arguments from Task 3 and command-line interface from Task 1.
- Produces: `comparison.json`, individual `orb-only-<rate>x/tracking_benchmark.json`, and `full-stack-<rate>x/tracking_benchmark.json` under an output root.
- Consumed by: manual validation and README documentation in Task 5.

- [ ] **Step 1: Write failing runner contract tests**

Add source-level tests so the runner contract is preserved without launching ROS:

```python
RUNNER_PATH = Path(__file__).resolve().parents[2] / "tools" / "run_tracking_performance_benchmark.sh"


def test_runner_uses_orb_only_sweep_and_full_stack_comparison():
    text = RUNNER_PATH.read_text(encoding="utf-8")
    assert "benchmark_mode:=orb_only" in text
    assert "benchmark_mode:=full_stack" in text
    assert "tracking_benchmark select" in text
    assert "tracking_benchmark compare" in text
    assert "--source-camera-hz" in text
    assert "--max-rate" in text
```

- [ ] **Step 2: Run runner contract test to verify failure**

Run: `pytest test/test_tracking_benchmark.py::test_runner_uses_orb_only_sweep_and_full_stack_comparison -v`

Expected: failure because the runner does not exist.

- [ ] **Step 3: Implement the runner with explicit arguments**

Create a Bash script with `set -euo pipefail`, repository-root discovery from the script location, and these options:

```text
--bag PATH                 default /home/duc/robot/20260713_152907
--output DIR               default artifacts/tracking-performance-<timestamp>
--domain BASE_INTEGER      default 95
--source-camera-hz FLOAT   default 30
--start-rate INTEGER       default 2
--max-rate INTEGER         default 8
--min-duration-s FLOAT     default 10
--ros-setup PATH           optional ROS distro setup.bash override
```

Reject nonpositive source rate/minimum duration, start rate below 1, and maximum rate below start rate. Verify the bag, `install/setup.bash`, `ros2`, and `python3` exist before starting.

For each integer rate from `start_rate` through `max_rate`, run this sequentially with a unique ROS domain `base_domain + rate`:

```bash
ros2 launch orb_slam_bringup bag_replay.launch.py \
  "bag_path:=${BAG_PATH}" \
  "artifact_dir:=${RUN_DIR}" \
  "rate:=${RATE}" \
  "ros_domain_id:=${DOMAIN}" \
  "start_dashboard:=false" \
  "benchmark_mode:=orb_only" \
  "benchmark_min_duration_s:=${MIN_DURATION_S}"
```

Require `RUN_DIR/tracking_benchmark.json` after every launch. Collect result paths. Invoke:

```bash
python3 -m orb_slam_bringup.tracking_benchmark select \
  --source-camera-hz "${SOURCE_CAMERA_HZ}" \
  --result "${RESULT_PATHS[@]}"
```

after every trial. If it reports no saturation, continue; if it returns a selected JSON result, capture its `playback_rate` with Python JSON parsing and break. If the loop ends without selection, exit nonzero and state that `--max-rate` was reached.

Then execute the full-stack launch once using selected rate and the next unused domain:

```bash
ros2 launch orb_slam_bringup bag_replay.launch.py \
  "bag_path:=${BAG_PATH}" \
  "artifact_dir:=${FULL_STACK_DIR}" \
  "rate:=${SELECTED_RATE}" \
  "ros_domain_id:=$((BASE_DOMAIN + MAX_RATE + 1))" \
  "start_dashboard:=false" \
  "benchmark_mode:=full_stack" \
  "benchmark_min_duration_s:=${MIN_DURATION_S}"
```

Call the comparison command with `--output "${OUTPUT_DIR}/comparison.json"`. Preserve its exit code so a valid below-80% result exits 1. Print exact artifact paths and measured rates before exiting.

- [ ] **Step 4: Mark runner executable and run contract tests**

Run: `chmod +x tools/run_tracking_performance_benchmark.sh && pytest test/test_tracking_benchmark.py -v`

Expected: all tests pass.

- [ ] **Step 5: Validate argument error handling without ROS replay**

Run: `tools/run_tracking_performance_benchmark.sh --start-rate 4 --max-rate 3`

Expected: exits nonzero with a clear maximum-rate validation message before sourcing or launching ROS.

- [ ] **Step 6: Commit**

```bash
git add tools/run_tracking_performance_benchmark.sh orb_slam_bringup/test/test_tracking_benchmark.py
git commit -m "feat: add tracking performance sweep runner"
```

## Task 5: Finalize The Existing Metrics Recorder Optimization

**Files:**
- Modify: `orb_slam_bringup/orb_slam_bringup/metrics_recorder.py`
- Modify: `orb_slam_bringup/test/test_metrics.py`
- Modify: `orb_slam_bringup/test/test_metrics_shutdown.py`

**Interfaces:**
- Consumes: the existing uncommitted recorder changes already present on this branch.
- Produces: a committed recorder optimization implementation used by the full-stack benchmark mode.

- [ ] **Step 1: Review the exact existing optimization diff**

Run: `git diff -- orb_slam_bringup/orb_slam_bringup/metrics_recorder.py orb_slam_bringup/test/test_metrics.py orb_slam_bringup/test/test_metrics_shutdown.py`

Expected: the diff contains only the intended compact NumPy map snapshots,
vectorized map/PNG/PGM operations, persistent JSONL writer, gated background
PNG queue, callback groups, multi-threaded executor, and their tests.

- [ ] **Step 2: Run the recorder-focused regression suite**

Run: `pytest test/test_metrics.py test/test_metrics_shutdown.py -v`

Expected: all recorder tests pass.

- [ ] **Step 3: Run the complete bringup suite before benchmarking**

Run: `pytest test -v`

Expected: all `orb_slam_bringup` tests pass.

- [ ] **Step 4: Commit the recorder changes independently**

```bash
git add orb_slam_bringup/orb_slam_bringup/metrics_recorder.py orb_slam_bringup/test/test_metrics.py orb_slam_bringup/test/test_metrics_shutdown.py
git commit -m "perf: reduce metrics recorder callback overhead"
```

## Task 6: Document And Validate The Benchmark Path

**Files:**
- Modify: `README.md:112-155`
- Modify: `orb_slam_bringup/test/test_bag_profile.py`

**Interfaces:**
- Consumes: runner artifact structure from Task 4.
- Produces: operator instructions that match the actual script and acceptance gate.

- [ ] **Step 1: Write a failing README contract test**

```python
def test_readme_documents_tracking_performance_benchmark():
    readme = Path(__file__).resolve().parents[2] / "README.md"
    text = readme.read_text(encoding="utf-8")
    assert "run_tracking_performance_benchmark.sh" in text
    assert "80%" in text
    assert "first ORB-only rate" in text
    assert "tracking_benchmark.json" in text
```

- [ ] **Step 2: Run README test to verify failure**

Run: `pytest test/test_bag_profile.py::test_readme_documents_tracking_performance_benchmark -v`

Expected: failure because the benchmark is not documented.

- [ ] **Step 3: Document benchmark execution and interpretation**

Add a `## Tracking performance benchmark` section after the full-bag replay instructions. Include this exact starter command:

```bash
source install/setup.bash
tools/run_tracking_performance_benchmark.sh \
  --bag /home/duc/robot/20260713_152907 \
  --source-camera-hz 30 \
  --start-rate 2 \
  --max-rate 8
```

Document that the runner:

- Starts with ORB-only at 2x and advances in 1x increments.
- Selects the first ORB-only speed below `source_camera_hz * playback_rate`.
- Runs full stack at exactly that selected rate, with dashboard disabled in both modes.
- Writes per-run `tracking_benchmark.json` plus top-level `comparison.json`.
- Passes only at 80% or more of baseline FPS.
- Fails for invalid probe results or if no saturation point is found through `--max-rate`.

Use the 3x/80 FPS/64 FPS example from the design spec.

- [ ] **Step 4: Run all focused Python tests**

Run: `pytest test/test_tracking_benchmark.py test/test_bag_profile.py test/test_metrics.py test/test_metrics_shutdown.py -v`

Expected: all tests pass.

- [ ] **Step 5: Build the affected ROS package**

Run: `COLCON_DEFAULTS_FILE=/dev/null colcon build --packages-select orb_slam_bringup --symlink-install`

Expected: build exits 0.

- [ ] **Step 6: Verify installed launch and probe interfaces**

Run: `source install/setup.bash && ros2 launch orb_slam_bringup bag_replay.launch.py --show-args && ros2 run orb_slam_bringup tracking_benchmark_probe --help`

Expected: launch lists both benchmark arguments; probe starts argument handling without an import or entry-point error.

- [ ] **Step 7: Run the actual sequential benchmark sweep**

Run: `source install/setup.bash && tools/run_tracking_performance_benchmark.sh --bag /home/duc/robot/20260713_152907 --source-camera-hz 30 --start-rate 2 --max-rate 8 --domain 95`

Expected: the script writes ORB-only results, selects the first under-target rate, writes the full-stack result and `comparison.json`, and exits 0 only if the full-stack FPS reaches the calculated 80% threshold. Record the selected rate, baseline FPS, full-stack FPS, threshold, and exit status in the task summary.

- [ ] **Step 8: Commit documentation**

```bash
git add README.md orb_slam_bringup/test/test_bag_profile.py
git commit -m "docs: describe tracking performance benchmark"
```

## Final Verification

- [ ] **Step 1: Inspect the working tree and intended diff**

Run: `git status --short && git diff --check && git log --oneline -10`

Expected: no unintended changes are staged or committed; pre-existing recorder changes are preserved and reviewed separately.

- [ ] **Step 2: Run package tests after all commits**

Run: `pytest orb_slam_bringup/test -v`

Expected: all `orb_slam_bringup` tests pass.

- [ ] **Step 3: Record benchmark evidence**

Inspect the generated `comparison.json` and both selected-run `tracking_benchmark.json` artifacts. Report only measured values, the computed 80% threshold, and whether the process exit code satisfies the acceptance gate.
