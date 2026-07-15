"""Unit tests for static report, acceptance gates, and run comparison."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from orb_slam_bringup.report import (
    ACCEPTANCE_THRESHOLDS,
    check_acceptance,
    compare_runs,
    generate_report_html,
    render_trajectory_overlay_png,
)


def _passing_metrics() -> dict:
    return {
        "bag": {
            "path": "/bags/demo",
            "duration_s": 221.146644609,
        },
        "git": {"commit": "abc123", "dirty": False},
        "configuration_sha256": "deadbeef" * 8,
        "stereo": {
            "expected_pairs": 6633,
            "paired_count": 6600,  # 99.5% >= 99%
            "paired_ratio": 6600 / 6633,
            "width": 848,
            "height": 480,
            "baseline_m": 0.0501881428,
            "camera_validated": True,
        },
        "tracking": {
            "initialized": True,
            "init_time_s": 2.5,
            "post_init_duration_s": 200.0,
            "ok_duration_s": 160.0,
            "ok_ratio_after_init": 0.80,
            "loop_count": 2,
            "loss_intervals": [{"start_s": 50.0, "end_s": 52.0, "duration_s": 2.0}],
            "mean_rebuild_ms": 120.0,
            "max_rebuild_ms": 200.0,
            "invalid_poses": 0,
            "deadlock": False,
        },
        "fallback": {
            "unresolved_scan_count": 0,
            "invalid_tf_committed": 0,
            "wheel_only_before_recovery": 0,
            "planarity_rejections": 0,
            "measured": True,
        },
        "loops": [
            {"t": 80.0, "graph_revision": 5, "map_id": 1, "detail": "loop-a"},
            {"t": 150.0, "graph_revision": 8, "map_id": 1, "detail": "loop-b"},
        ],
        "map_revisions": [
            {
                "t": 80.5,
                "state": "PUBLISHED",
                "graph_revision": 5,
                "map_revision": 1,
                "duration_ms": 100.0,
                "input_scan_count": 50,
                "committed_scan_count": 50,
            },
            {
                "t": 150.5,
                "state": "PUBLISHED",
                "graph_revision": 8,
                "map_revision": 2,
                "duration_ms": 140.0,
                "input_scan_count": 80,
                "committed_scan_count": 80,
            },
        ],
        "final_map": {
            "width": 100,
            "height": 80,
            "resolution": 0.05,
            "origin": [-2.0, -3.0, 0.0],
            "free_cells": 4000,
            "occupied_cells": 500,
            "unknown_cells": 3500,
            "pgm_yaml_match": True,
        },
        "diagnostics": [],
        "event_types_ordered": [
            "INITIALIZED",
            "LOST",
            "RELOCALIZED",
            "LOOP_CLOSED",
            "LOOP_CLOSED",
        ],
        "trajectories": {
            "orb": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"t": 1.0, "x": 0.1, "y": 0.0, "yaw": 0.01},
            ],
            "wheel": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"t": 1.0, "x": 0.11, "y": 0.0, "yaw": 0.01},
            ],
            "corrected": [
                {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"t": 1.0, "x": 0.1, "y": 0.0, "yaw": 0.01},
            ],
        },
    }


def _failing_metrics() -> dict:
    m = _passing_metrics()
    m = json.loads(json.dumps(m))  # deep copy
    m["tracking"]["ok_ratio_after_init"] = 0.5  # below 70%
    m["tracking"]["loop_count"] = 0
    m["loops"] = []
    m["fallback"]["invalid_tf_committed"] = 3
    m["final_map"]["free_cells"] = 0
    return m


def test_acceptance_thresholds_constants():
    assert ACCEPTANCE_THRESHOLDS["stereo_paired_min_ratio"] == pytest.approx(0.99)
    assert ACCEPTANCE_THRESHOLDS["expected_stereo_pairs"] == 6633
    assert ACCEPTANCE_THRESHOLDS["camera_width"] == 848
    assert ACCEPTANCE_THRESHOLDS["camera_height"] == 480
    assert ACCEPTANCE_THRESHOLDS["baseline_m"] == pytest.approx(0.0501881428)
    assert ACCEPTANCE_THRESHOLDS["ok_ratio_min"] == pytest.approx(0.70)
    assert ACCEPTANCE_THRESHOLDS["min_loop_closures"] == 1
    assert ACCEPTANCE_THRESHOLDS["traj_pos_tol_m"] == pytest.approx(0.02)
    assert ACCEPTANCE_THRESHOLDS["traj_yaw_tol_rad"] == pytest.approx(math.radians(1.0))
    assert ACCEPTANCE_THRESHOLDS["cell_count_rel_tol"] == pytest.approx(0.01)


def test_check_acceptance_passes_on_good_metrics():
    result = check_acceptance(_passing_metrics())
    assert result["pass"] is True
    assert all(g["pass"] for g in result["gates"])
    assert result["failed"] == []


def test_check_acceptance_fails_on_bad_metrics():
    result = check_acceptance(_failing_metrics())
    assert result["pass"] is False
    assert len(result["failed"]) >= 3
    names = {g["name"] for g in result["failed"]}
    assert "tracking_ok_ratio" in names
    assert "loop_closures" in names
    assert "invalid_tf_committed" in names
    assert "final_free_cells" in names


def test_check_main_exit_codes(tmp_path: Path, monkeypatch):
    from orb_slam_bringup import report as report_mod

    good = tmp_path / "good.json"
    bad = tmp_path / "bad.json"
    good.write_text(json.dumps(_passing_metrics()), encoding="utf-8")
    bad.write_text(json.dumps(_failing_metrics()), encoding="utf-8")

    assert report_mod.check_main(["check", str(good)]) == 0
    assert report_mod.check_main(["check", str(bad)]) != 0
    assert report_mod.check_main(["check", str(tmp_path / "missing.json")]) != 0


def test_compare_runs_identical():
    a = _passing_metrics()
    b = json.loads(json.dumps(a))
    result = compare_runs(a, b)
    assert result["pass"] is True
    assert result["mismatches"] == []


def test_compare_runs_detects_map_and_cell_mismatch():
    a = _passing_metrics()
    b = json.loads(json.dumps(a))
    b["final_map"]["width"] = 99
    b["final_map"]["free_cells"] = 100  # far from 4000
    result = compare_runs(a, b)
    assert result["pass"] is False
    reasons = " ".join(result["mismatches"])
    assert "dimension" in reasons.lower() or "width" in reasons.lower()
    assert "free" in reasons.lower() or "cell" in reasons.lower()


def test_compare_runs_trajectory_tolerance():
    a = _passing_metrics()
    b = json.loads(json.dumps(a))
    # Within 2 cm / 1 deg → pass
    b["trajectories"]["orb"][1]["x"] = 0.1 + 0.015
    b["trajectories"]["orb"][1]["yaw"] = 0.01 + math.radians(0.5)
    assert compare_runs(a, b)["pass"] is True

    # Beyond tolerance → fail
    b["trajectories"]["orb"][1]["x"] = 0.1 + 0.05
    result = compare_runs(a, b)
    assert result["pass"] is False
    assert any("trajectory" in m.lower() for m in result["mismatches"])


def test_compare_main_exit_codes(tmp_path: Path):
    from orb_slam_bringup import report as report_mod

    a = tmp_path / "a.json"
    b = tmp_path / "b.json"
    c = tmp_path / "c.json"
    a.write_text(json.dumps(_passing_metrics()), encoding="utf-8")
    b.write_text(json.dumps(_passing_metrics()), encoding="utf-8")
    bad = _passing_metrics()
    bad["final_map"]["height"] = 1
    c.write_text(json.dumps(bad), encoding="utf-8")

    assert report_mod.compare_main(["compare", str(a), str(b)]) == 0
    assert report_mod.compare_main(["compare", str(a), str(c)]) != 0


def test_generate_report_html_is_self_contained(tmp_path: Path):
    metrics = _passing_metrics()
    # Create dummy artifact images referenced by report
    (tmp_path / "map-revision-1.png").write_bytes(b"\x89PNG\r\n\x1a\n")
    (tmp_path / "final-map.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    html = generate_report_html(metrics, artifact_dir=tmp_path)
    out = tmp_path / "report.html"
    out.write_text(html, encoding="utf-8")

    assert "<html" in html.lower()
    assert "pass" in html.lower() or "fail" in html.lower()
    # No external network resources
    assert "http://" not in html
    assert "https://" not in html
    assert "cdn." not in html.lower()
    # Local CSS/JS embedded
    assert "<style" in html.lower()
    # Sections
    assert "tracking" in html.lower()
    assert "loop" in html.lower()
    assert "map" in html.lower()
    assert "diagnostic" in html.lower()
    # Links to raw artifacts (relative)
    assert "metrics.json" in html
    # Embedded base64 image for trajectory overlay
    assert "data:image/png;base64," in html


def test_render_trajectory_overlay_png():
    png = render_trajectory_overlay_png(
        orb=[{"t": 0, "x": 0.0, "y": 0.0}, {"t": 1, "x": 1.0, "y": 0.5}],
        wheel=[{"t": 0, "x": 0.0, "y": 0.0}, {"t": 1, "x": 1.1, "y": 0.4}],
        corrected=[{"t": 0, "x": 0.0, "y": 0.0}, {"t": 1, "x": 1.0, "y": 0.5}],
        width=320,
        height=240,
    )
    assert isinstance(png, (bytes, bytearray))
    assert png[:8] == b"\x89PNG\r\n\x1a\n"


def test_every_loop_requires_completed_rebuild_gate():
    m = _passing_metrics()
    # Remove rebuild for second loop graph_revision=8
    m["map_revisions"] = [m["map_revisions"][0]]
    result = check_acceptance(m)
    assert result["pass"] is False
    assert any(g["name"] == "loop_rebuild_complete" for g in result["failed"])


def _unmeasured_metrics() -> dict:
    """Live-recorder defaults: stereo/camera/fallback never instrumented."""
    m = json.loads(json.dumps(_passing_metrics()))
    m["stereo"]["paired_count"] = 0
    m["stereo"]["paired_ratio"] = 0.0
    m["stereo"]["camera_validated"] = False
    # Unmeasured fallback counters must not be confident zeros
    m["fallback"]["invalid_tf_committed"] = None
    m["fallback"]["wheel_only_before_recovery"] = None
    m["fallback"]["unresolved_scan_count"] = None
    return m


def test_unmeasured_run_fails_acceptance_gates():
    """Fail-closed: uninstrumented stereo/camera/fallback must not green-light."""
    result = check_acceptance(_unmeasured_metrics())
    assert result["pass"] is False
    names = {g["name"] for g in result["failed"]}
    assert "stereo_paired_ratio" in names
    assert "camera_validated" in names
    assert "invalid_tf_committed" in names
    assert "wheel_only_before_recovery" in names


def test_check_main_nonzero_on_unmeasured_metrics(tmp_path: Path):
    from orb_slam_bringup import report as report_mod

    path = tmp_path / "unmeasured.json"
    path.write_text(json.dumps(_unmeasured_metrics()), encoding="utf-8")
    assert report_mod.check_main(["check", str(path)]) != 0


def test_check_main_zero_on_measured_passing_metrics(tmp_path: Path):
    from orb_slam_bringup import report as report_mod

    path = tmp_path / "measured_good.json"
    path.write_text(json.dumps(_passing_metrics()), encoding="utf-8")
    assert report_mod.check_main(["check", str(path)]) == 0


def test_fallback_unmeasured_sentinel_fails_even_when_other_gates_pass():
    m = _passing_metrics()
    m["fallback"]["invalid_tf_committed"] = None
    result = check_acceptance(m)
    assert result["pass"] is False
    assert any(g["name"] == "invalid_tf_committed" for g in result["failed"])


def test_compare_runs_non_overlapping_timestamps_fails():
    """Different time bases: all samples skipped → must FAIL, not silent pass."""
    a = _passing_metrics()
    b = json.loads(json.dumps(a))
    # Shift B trajectories far outside the 0.05s match window
    for name in ("orb", "wheel", "corrected"):
        for pt in b["trajectories"][name]:
            pt["t"] = float(pt["t"]) + 1000.0
    result = compare_runs(a, b)
    assert result["pass"] is False
    assert any("trajectory" in m.lower() for m in result["mismatches"])
