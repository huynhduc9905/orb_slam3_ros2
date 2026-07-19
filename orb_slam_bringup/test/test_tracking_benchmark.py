from pathlib import Path
import json
from types import SimpleNamespace
import pytest

from orb_slam_bringup.tracking_benchmark import (
    BenchmarkResult,
    TrackingRateCounter,
    compare_full_stack,
    load_result,
    main,
    select_stress_point,
)
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


def test_counter_reports_invalid_when_orb_never_initializes():
    counter = TrackingRateCounter(mode="orb_only", playback_rate=2.0, min_duration_s=1.0)
    counter.observe(tracking_ok=False, now_s=4.0)

    result = counter.finish(now_s=6.0)

    assert result.invalid_reason == "orb_never_initialized"


def test_counter_reports_invalid_when_zero_eligible_frames():
    counter = TrackingRateCounter(mode="orb_only", playback_rate=2.0, min_duration_s=1.0)
    counter.initialized = True
    counter.start_s = 1.0
    counter.received_frames = 0

    result = counter.finish(now_s=3.0)

    assert result.invalid_reason == "no_eligible_frames"


def test_counter_reports_invalid_when_duration_below_minimum():
    counter = TrackingRateCounter(mode="orb_only", playback_rate=2.0, min_duration_s=5.0)
    counter.observe(tracking_ok=True, now_s=1.0)
    counter.observe(tracking_ok=True, now_s=2.0)

    result = counter.finish(now_s=3.0)  # duration = 2.0 < 5.0

    assert result.invalid_reason == "measurement_duration_below_minimum"


def test_result_round_trips_as_json(tmp_path: Path):
    counter = TrackingRateCounter(mode="full_stack", playback_rate=3.0, min_duration_s=1.0)
    counter.observe(tracking_ok=True, now_s=1.0)
    counter.observe(tracking_ok=True, now_s=2.0)
    path = tmp_path / "tracking_benchmark.json"

    counter.finish(now_s=3.0).write(path)

    assert load_result(path).tracking_fps == 1.0
    assert load_result(path).mode == "full_stack"


def result(rate: float, fps: float, invalid_reason: str | None = None) -> BenchmarkResult:
    return BenchmarkResult("orb_only", rate, 10.0, int(fps * 10), fps if invalid_reason is None else 0.0, int(fps * 10), 1.0, True, invalid_reason)


def test_selects_first_orb_only_rate_below_source_target():
    selected = select_stress_point([result(2.0, 60.0), result(3.0, 80.0)], source_camera_hz=30.0)

    assert selected.playback_rate == 3.0
    assert selected.tracking_fps == 80.0


def test_select_stress_point_raises_when_no_saturation_found():
    with pytest.raises(ValueError, match="ORB-only saturation point was not found"):
        select_stress_point([result(1.0, 30.0), result(2.0, 60.0)], source_camera_hz=30.0)


def test_select_stress_point_raises_when_result_invalid():
    with pytest.raises(ValueError, match="invalid ORB-only result at 2.0x: orb_never_initialized"):
        select_stress_point([result(2.0, 60.0, invalid_reason="orb_never_initialized")], source_camera_hz=30.0)


def test_compare_full_stack_differing_rates_raises():
    baseline = result(2.0, 60.0)
    full_stack = BenchmarkResult("full_stack", 3.0, 10.0, 600, 60.0, 600, 1.0, True, None)
    with pytest.raises(ValueError, match="baseline and full-stack playback rates differ"):
        compare_full_stack(baseline, full_stack)


def test_compare_full_stack_invalid_result_raises():
    baseline = result(2.0, 60.0, invalid_reason="measurement_duration_below_minimum")
    full_stack = BenchmarkResult("full_stack", 2.0, 10.0, 600, 60.0, 600, 1.0, True, None)
    with pytest.raises(ValueError, match="cannot compare invalid benchmark results"):
        compare_full_stack(baseline, full_stack)


def test_full_stack_passes_at_exactly_eighty_percent():
    baseline = result(3.0, 80.0)
    full_stack = BenchmarkResult("full_stack", 3.0, 10.0, 640, 64.0, 640, 1.0, True, None)

    comparison = compare_full_stack(baseline, full_stack)

    assert comparison["threshold_fps"] == 64.0
    assert comparison["passed"] is True


def test_full_stack_fails_below_eighty_percent():
    baseline = result(3.0, 80.0)
    full_stack = BenchmarkResult("full_stack", 3.0, 10.0, 639, 63.9, 639, 1.0, True, None)

    comparison = compare_full_stack(baseline, full_stack)

    assert comparison["threshold_fps"] == 64.0
    assert comparison["passed"] is False


def test_cli_select_and_compare(tmp_path: Path, capsys):
    res_2x = tmp_path / "orb-2x.json"
    res_3x = tmp_path / "orb-3x.json"
    full_3x = tmp_path / "full-3x.json"
    comp_out = tmp_path / "comparison.json"

    result(2.0, 60.0).write(res_2x)
    result(3.0, 80.0).write(res_3x)

    ret = main(["select", "--source-camera-hz", "30", "--result", str(res_2x), "--result", str(res_3x)])
    assert ret == 0
    captured = capsys.readouterr()
    selected_data = json.loads(captured.out)
    assert selected_data["playback_rate"] == 3.0

    # Compare passing
    BenchmarkResult("full_stack", 3.0, 10.0, 640, 64.0, 640, 1.0, True, None).write(full_3x)
    ret_comp = main(["compare", "--baseline", str(res_3x), "--full-stack", str(full_3x), "--output", str(comp_out)])
    assert ret_comp == 0
    comp_data = json.loads(comp_out.read_text())
    assert comp_data["passed"] is True

    # Compare failing gate
    BenchmarkResult("full_stack", 3.0, 10.0, 639, 63.9, 639, 1.0, True, None).write(full_3x)
    ret_comp_fail = main(["compare", "--baseline", str(res_3x), "--full-stack", str(full_3x), "--output", str(comp_out)])
    assert ret_comp_fail == 1
    comp_data_fail = json.loads(comp_out.read_text())
    assert comp_data_fail["passed"] is False

    # Compare invalid input (e.g. rate mismatch)
    BenchmarkResult("full_stack", 2.0, 10.0, 600, 60.0, 600, 1.0, True, None).write(full_3x)
    ret_comp_err = main(["compare", "--baseline", str(res_3x), "--full-stack", str(full_3x), "--output", str(comp_out)])
    assert ret_comp_err == 2

