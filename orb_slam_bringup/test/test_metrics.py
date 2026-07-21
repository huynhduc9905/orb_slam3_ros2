"""Unit tests for pure metric aggregation and Nav2 map export (no live ROS)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from orb_slam_bringup.metrics_recorder import (
    EVENT_INITIALIZED,
    EVENT_LOOP_CLOSED,
    EVENT_LOST,
    EVENT_RELOCALIZED,
    MAP_BUILDING,
    MAP_PUBLISHED,
    PNG_FREE,
    PNG_OCCUPIED,
    PNG_UNKNOWN,
    TRACK_LOST,
    TRACK_OK,
    TRACK_RECENTLY_LOST,
    EventJsonlWriter,
    FrameSample,
    MapGridSample,
    MapRevisionPngQueue,
    MapRevisionSample,
    MetricsAggregator,
    TrackingEventSample,
    count_occupancy_cells,
    occupancy_grid_to_png_array,
    write_nav2_map,
)


def _agg_from_timeline() -> dict:
    """Synthetic timeline: init → OK → loss → recovery → loop → rebuild published."""
    # Times in seconds (sim time).
    frames = [
        FrameSample(t=0.0, tracking_state=1, pose_valid=False),  # NOT_INITIALIZED
        FrameSample(t=1.0, tracking_state=TRACK_OK, pose_valid=True),
        FrameSample(t=2.0, tracking_state=TRACK_OK, pose_valid=True),
        FrameSample(t=3.0, tracking_state=TRACK_OK, pose_valid=True),
        FrameSample(t=4.0, tracking_state=TRACK_RECENTLY_LOST, pose_valid=False),
        FrameSample(t=5.0, tracking_state=TRACK_LOST, pose_valid=False),
        FrameSample(t=6.0, tracking_state=TRACK_OK, pose_valid=True),
        FrameSample(t=7.0, tracking_state=TRACK_OK, pose_valid=True),
        FrameSample(t=8.0, tracking_state=TRACK_OK, pose_valid=True),
        FrameSample(t=9.0, tracking_state=TRACK_OK, pose_valid=True),
        FrameSample(t=10.0, tracking_state=TRACK_OK, pose_valid=True),
    ]
    events = [
        TrackingEventSample(t=1.0, type=EVENT_INITIALIZED, graph_revision=1, map_id=1),
        TrackingEventSample(t=4.5, type=EVENT_LOST, graph_revision=1, map_id=1),
        TrackingEventSample(t=6.0, type=EVENT_RELOCALIZED, graph_revision=1, map_id=1),
        TrackingEventSample(
            t=8.0, type=EVENT_LOOP_CLOSED, graph_revision=2, map_id=1, detail="loop"
        ),
    ]
    revisions = [
        MapRevisionSample(
            t=8.1,
            state=MAP_BUILDING,
            graph_revision=2,
            map_revision=1,
            input_scan_count=100,
            committed_scan_count=0,
            duration_ms=0.0,
        ),
        MapRevisionSample(
            t=8.5,
            state=MAP_PUBLISHED,
            graph_revision=2,
            map_revision=1,
            input_scan_count=100,
            committed_scan_count=95,
            duration_ms=400.0,
        ),
        MapRevisionSample(
            t=9.5,
            state=MAP_BUILDING,
            graph_revision=2,
            map_revision=2,
            input_scan_count=120,
            committed_scan_count=0,
            duration_ms=0.0,
        ),
        MapRevisionSample(
            t=10.0,
            state=MAP_PUBLISHED,
            graph_revision=2,
            map_revision=2,
            input_scan_count=120,
            committed_scan_count=118,
            duration_ms=500.0,
        ),
    ]
    # 4x3 grid: free=0, occupied=100, unknown=-1
    data = [
        0, 0, 100,  # row 0
        -1, 0, 100,  # row 1
        -1, -1, 0,  # row 2
        100, 0, -1,  # row 3
    ]
    grid = MapGridSample(
        width=3,
        height=4,
        resolution=0.05,
        origin_x=-1.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        data=data,
    )
    agg = MetricsAggregator(
        bag_duration_s=10.0,
        frames=frames,
        events=events,
        revisions=revisions,
        final_map=grid,
        unresolved_scan_count=5,
        invalid_tf_committed=0,
        wheel_only_before_recovery=0,
    )
    return agg.compute()


def test_tracked_duration_ratio_after_init():
    m = _agg_from_timeline()
    tracking = m["tracking"]
    # After init at t=1.0, window is 9.0s (1→10).
    # OK frames (or intervals): from frame samples, post-init OK at
    # t=1,2,3 then lost 4,5 then OK 6,7,8,9,10.
    # Using interval-based duration between consecutive samples:
    # OK intervals: 1→2,2→3,3→4 (OK until 4 starts RECENTLY_LOST),
    # then 6→7,7→8,8→9,9→10.
    # Duration OK: 1+1+1 + 1+1+1+1 = 7.0 over post-init 9.0 → 7/9
    assert tracking["ok_ratio_after_init"] == pytest.approx(7.0 / 9.0)
    assert tracking["initialized"] is True
    assert tracking["init_time_s"] == pytest.approx(1.0)
    assert tracking["post_init_duration_s"] == pytest.approx(9.0)
    assert tracking["ok_duration_s"] == pytest.approx(7.0)


def test_loss_intervals():
    m = _agg_from_timeline()
    intervals = m["tracking"]["loss_intervals"]
    # LOST event at 4.5, RELOCALIZED at 6.0 → one interval [4.5, 6.0]
    assert len(intervals) == 1
    assert intervals[0]["start_s"] == pytest.approx(4.5)
    assert intervals[0]["end_s"] == pytest.approx(6.0)
    assert intervals[0]["duration_s"] == pytest.approx(1.5)


def test_loop_count_and_revision_sequence():
    m = _agg_from_timeline()
    assert m["tracking"]["loop_count"] == 1
    assert len(m["loops"]) == 1
    assert m["loops"][0]["graph_revision"] == 2
    assert m["loops"][0]["t"] == pytest.approx(8.0)

    revs = m["map_revisions"]
    published = [r for r in revs if r["state"] == "PUBLISHED"]
    assert [r["map_revision"] for r in published] == [1, 2]
    assert [r["graph_revision"] for r in published] == [2, 2]


def test_mean_max_rebuild_time():
    m = _agg_from_timeline()
    published = [r for r in m["map_revisions"] if r["state"] == "PUBLISHED"]
    times = [r["duration_ms"] for r in published]
    assert times == [400.0, 500.0]
    assert m["tracking"]["mean_rebuild_ms"] == pytest.approx(450.0)
    assert m["tracking"]["max_rebuild_ms"] == pytest.approx(500.0)


def test_unresolved_scan_count():
    m = _agg_from_timeline()
    assert m["fallback"]["unresolved_scan_count"] == 5
    assert m["fallback"]["invalid_tf_committed"] == 0
    assert m["fallback"]["wheel_only_before_recovery"] == 0


def test_map_dimensions_and_cell_counts():
    m = _agg_from_timeline()
    fm = m["final_map"]
    assert fm["width"] == 3
    assert fm["height"] == 4
    assert fm["resolution"] == pytest.approx(0.05)
    assert fm["origin"] == [-1.0, -2.0, 0.0]
    # free=0: indices 0,1,4,8,10 → 5
    # occupied=100: 2,5,9 → 3
    # unknown=-1: 3,6,7,11 → 4
    assert fm["free_cells"] == 5
    assert fm["occupied_cells"] == 3
    assert fm["unknown_cells"] == 4
    assert fm["free_cells"] + fm["occupied_cells"] + fm["unknown_cells"] == 12


def test_count_occupancy_cells_helper():
    data = [0, 100, -1, 50, 0]
    free, occ, unk = count_occupancy_cells(data)
    # free: <= free_thresh style? Nav2: free is 0, occupied is 100, unknown -1.
    # Use strict: free==0, occupied==100, else if <0 unknown, else occupied-ish.
    # Spec: free cells (0), occupied (100), unknown (-1). Other values count as occupied.
    assert free == 2
    assert occ == 2  # 100 and 50
    assert unk == 1


def test_write_nav2_map_roundtrip(tmp_path: Path):
    """Write PGM/YAML from OccupancyGrid-like sample; reload and match dims/origin."""
    data = [
        0, 0, 100,
        -1, 0, 100,
        -1, -1, 0,
        100, 0, -1,
    ]
    grid = MapGridSample(
        width=3,
        height=4,
        resolution=0.05,
        origin_x=-1.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        data=data,
    )
    stem = tmp_path / "final-map"
    write_nav2_map(grid, stem)

    yaml_path = Path(str(stem) + ".yaml")
    pgm_path = Path(str(stem) + ".pgm")
    assert yaml_path.is_file()
    assert pgm_path.is_file()

    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    assert meta["resolution"] == pytest.approx(0.05)
    assert meta["origin"] == pytest.approx([-1.0, -2.0, 0.0])
    assert meta["negate"] == 0
    assert "occupied_thresh" in meta
    assert "free_thresh" in meta
    assert meta["image"] in ("final-map.pgm", str(pgm_path.name))

    # P5 PGM: header + binary pixels (occupied=0, free=254, unknown=205)
    raw = pgm_path.read_bytes()
    assert raw.startswith(b"P5\n")
    # Parse header
    lines = []
    idx = 0
    # skip magic
    idx = raw.index(b"\n") + 1
    while True:
        end = raw.index(b"\n", idx)
        line = raw[idx:end].decode("ascii")
        idx = end + 1
        if line.startswith("#"):
            continue
        lines.append(line)
        if len(lines) >= 2:  # width height, then maxval
            if len(lines) == 2:
                # still need maxval
                continue
            break
        if len(lines) == 1:
            # could be "W H" or need next
            parts = lines[0].split()
            if len(parts) >= 2:
                # next line is maxval
                end = raw.index(b"\n", idx)
                maxval_line = raw[idx:end].decode("ascii")
                idx = end + 1
                lines.append(maxval_line)
                break

    wh = lines[0].split()
    width, height = int(wh[0]), int(wh[1])
    maxval = int(lines[1])
    assert width == 3
    assert height == 4
    assert maxval == 255
    pixels = raw[idx:]
    assert len(pixels) == width * height

    # Row-major, row 0 is first in OccupancyGrid (ROS OccupancyGrid is row-major
    # with origin at bottom-left in world; PGM writes with origin bottom-left so
    # first row in file is top of image = last row of grid data).
    # write_nav2_map should flip vertically for standard map_server convention.
    # After reload via our reader helper:
    free, occ, unk = count_occupancy_cells(data)
    # Verify pixel encoding for known cells
    # data[0]=0 free → 254, data[2]=100 occ → 0, data[3]=-1 unk → 205
    # Depending on flip: if we flip, file row 0 = grid row height-1 = data[9,10,11]
    # We'll check total counts via reverse mapping.
    rev = []
    for p in pixels:
        if p == 0:
            rev.append(100)
        elif p == 254:
            rev.append(0)
        else:
            rev.append(-1)
    # Regardless of flip, cell counts must match
    assert rev.count(0) == free
    assert rev.count(100) == occ
    assert rev.count(-1) == unk


def test_occupancy_grid_to_png_array_values():
    data = [0, 100, -1]
    arr = occupancy_grid_to_png_array(3, 1, data)
    assert arr.shape == (1, 3)
    # occupied dark, free light, unknown mid
    assert arr[0, 0] > arr[0, 2] > arr[0, 1]
    assert int(arr[0, 0]) == PNG_FREE
    assert int(arr[0, 1]) == PNG_OCCUPIED
    assert int(arr[0, 2]) == PNG_UNKNOWN


def test_occupancy_grid_to_png_array_multirow_and_nonzero_free_only_zero():
    """Semantics: free is exactly 0; any positive is occupied; negative unknown."""
    data = [
        0, 50, -1,
        100, 0, 1,
    ]
    arr = occupancy_grid_to_png_array(3, 2, data)
    assert arr.shape == (2, 3)
    assert int(arr[0, 0]) == PNG_FREE
    assert int(arr[0, 1]) == PNG_OCCUPIED  # 50 is occupied (not free band)
    assert int(arr[0, 2]) == PNG_UNKNOWN
    assert int(arr[1, 0]) == PNG_OCCUPIED
    assert int(arr[1, 1]) == PNG_FREE
    assert int(arr[1, 2]) == PNG_OCCUPIED


def test_map_revision_png_queue_writes_on_join(tmp_path: Path):
    """PNG encode/write must not be required to finish before enqueue returns."""
    data = [0, 100, -1, 0]
    grid = MapGridSample(
        width=2,
        height=2,
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=data,
    )
    path = tmp_path / "map-revision-7.png"
    q = MapRevisionPngQueue()
    q.enqueue(grid, path)
    # Caller may mutate grid after enqueue; queue owns a snapshot.
    grid.data[0] = 100
    q.join()
    assert path.is_file()
    # Original free cell at (0,0) must still be free in the PNG (snapshot).
    from PIL import Image

    img = np.array(Image.open(path))
    # flipud on write → image row 0 is grid row 1
    assert img.shape == (2, 2)
    # grid[0]=0 free → after flip appears at bottom row of image
    assert int(img[1, 0]) == PNG_FREE


def test_map_revision_png_gate_rate_limits():
    """Mid-run PNGs must be rate-limited so GIL/disk cannot starve spin."""
    from orb_slam_bringup.metrics_recorder import MapRevisionPngGate

    gate = MapRevisionPngGate(min_interval_s=1.0)
    assert gate.should_write(t=0.0, map_revision=1) is True
    assert gate.should_write(t=0.1, map_revision=2) is False
    assert gate.should_write(t=0.9, map_revision=3) is False
    assert gate.should_write(t=1.0, map_revision=4) is True
    assert gate.should_write(t=1.5, map_revision=5) is False
    assert gate.should_write(t=2.1, map_revision=6) is True
    # min_interval_s <= 0 disables limiting (write every revision)
    gate2 = MapRevisionPngGate(min_interval_s=0.0)
    assert gate2.should_write(t=0.0, map_revision=1) is True
    assert gate2.should_write(t=0.01, map_revision=2) is True


def test_event_jsonl_writer_keeps_handle_and_appends(tmp_path: Path):
    path = tmp_path / "events.jsonl"
    w = EventJsonlWriter(path)
    w.append({"kind": "a", "t": 1.0})
    w.append({"kind": "b", "t": 2.0})
    w.close()
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["kind"] == "a"
    assert json.loads(lines[1])["kind"] == "b"


def test_metrics_top_level_keys():
    m = _agg_from_timeline()
    for key in (
        "bag",
        "git",
        "configuration_sha256",
        "stereo",
        "tracking",
        "fallback",
        "loops",
        "map_revisions",
        "final_map",
        "diagnostics",
    ):
        assert key in m


def test_atomic_write_json(tmp_path: Path):
    from orb_slam_bringup.metrics_recorder import atomic_write_text

    path = tmp_path / "metrics.json"
    atomic_write_text(path, json.dumps({"ok": True}))
    assert json.loads(path.read_text(encoding="utf-8")) == {"ok": True}
    # No leftover temp files
    temps = list(tmp_path.glob(".*")) + list(tmp_path.glob("*.tmp"))
    assert temps == []


def test_live_stereo_section_fail_closed_when_unmeasured():
    """Unobserved stereo must report 0 pairs / 0.0 ratio / camera_validated False."""
    from orb_slam_bringup.metrics_recorder import build_live_stereo_section

    stereo = build_live_stereo_section(expected_pairs=6633, paired_count=0)
    assert stereo["paired_count"] == 0
    assert stereo["paired_ratio"] == 0.0
    assert stereo["camera_validated"] is False
    # Must NOT invent expected_pairs as paired_count
    assert stereo["paired_count"] != stereo["expected_pairs"]


def test_live_stereo_section_measured_good():
    from orb_slam_bringup.metrics_recorder import build_live_stereo_section

    stereo = build_live_stereo_section(
        expected_pairs=6633,
        paired_count=6600,
        camera_validated=True,
    )
    assert stereo["paired_count"] == 6600
    assert stereo["paired_ratio"] == pytest.approx(6600 / 6633)
    assert stereo["camera_validated"] is True


def test_fallback_from_diagnostics_unmeasured_is_none():
    from orb_slam_bringup.metrics_recorder import fallback_from_diagnostics

    fb = fallback_from_diagnostics(None)
    assert fb["invalid_tf_committed"] is None
    assert fb["wheel_only_before_recovery"] is None
    assert fb["measured"] is False


def test_fallback_from_diagnostics_maps_mapper_kvs():
    from orb_slam_bringup.metrics_recorder import fallback_from_diagnostics

    fb = fallback_from_diagnostics(
        {
            "tf_lookup_failures": 0,
            "wheel_interp_failures": 0,
            "planarity_rejections": 2,
        }
    )
    assert fb["measured"] is True
    # Commit invariants are 0 by mapper contract (not proxies of rejections).
    assert fb["invalid_tf_committed"] == 0
    assert fb["wheel_only_before_recovery"] == 0
    assert fb["planarity_rejections"] == 2
    # Raw rejection counters still exposed for observability.
    assert fb["tf_lookup_failures"] == 0
    assert fb["wheel_interp_failures"] == 0


def test_commit_gates_not_inflated_by_rejection_counters():
    """tf_lookup_failures / wheel_interp_failures are rejections, not commits.

    Mapper never commits invalid-TF or wheel-only scans; those counters count
    REJECTED / provisional scans. Commit gates must stay 0/measured.
    """
    from orb_slam_bringup.metrics_recorder import fallback_from_diagnostics

    fb = fallback_from_diagnostics(
        {
            "tf_lookup_failures": 1,
            "wheel_interp_failures": 1173,
            "planarity_rejections": 4,
        }
    )
    assert fb["measured"] is True
    assert fb["invalid_tf_committed"] == 0
    assert fb["wheel_only_before_recovery"] == 0
    assert fb["tf_lookup_failures"] == 1
    assert fb["wheel_interp_failures"] == 1173
    assert fb["planarity_rejections"] == 4


def test_explicit_violation_counters_override_mapper_contract():
    """If mapper ever publishes explicit commit-violation KVs, use them."""
    from orb_slam_bringup.metrics_recorder import fallback_from_diagnostics

    fb = fallback_from_diagnostics(
        {
            "tf_lookup_failures": 5,
            "wheel_interp_failures": 9,
            "invalid_tf_committed": 2,
            "wheel_only_before_recovery": 3,
        }
    )
    assert fb["invalid_tf_committed"] == 2
    assert fb["wheel_only_before_recovery"] == 3
    assert fb["measured"] is True


def test_aggregator_default_fallback_unmeasured():
    """MetricsAggregator with no fallback inputs must not invent confident zeros."""
    agg = MetricsAggregator(bag_duration_s=1.0)
    m = agg.compute()
    # Fail-closed: unmeasured counters are None (not 0)
    assert m["fallback"]["invalid_tf_committed"] is None
    assert m["fallback"]["wheel_only_before_recovery"] is None


def test_stereo_pair_counter_matched_timestamps():
    """100 left + 100 right within tolerance → 100 pairs."""
    from orb_slam_bringup.metrics_recorder import StereoPairCounter

    c = StereoPairCounter(tolerance_s=0.01)
    for i in range(100):
        t = i * 0.033
        c.on_left(t)
        c.on_right(t + 0.002)  # 2 ms offset, within 10 ms
    assert c.paired_count == 100
    assert c.left_count == 100
    assert c.right_count == 100
    assert c.streams_seen is True


def test_stereo_pair_counter_mismatched_timestamps():
    """Stamps outside tolerance do not form pairs."""
    from orb_slam_bringup.metrics_recorder import StereoPairCounter

    c = StereoPairCounter(tolerance_s=0.01)
    for i in range(50):
        c.on_left(i * 0.033)
        # 50 ms offset — outside 10 ms tolerance
        c.on_right(i * 0.033 + 0.050)
    assert c.paired_count == 0
    assert c.left_count == 50
    assert c.right_count == 50
    assert c.streams_seen is True


def test_stereo_pair_counter_partial_match():
    """Only frames with a partner within tolerance count as pairs."""
    from orb_slam_bringup.metrics_recorder import StereoPairCounter

    c = StereoPairCounter(tolerance_s=0.01)
    # 10 matched
    for i in range(10):
        t = float(i)
        c.on_left(t)
        c.on_right(t + 0.001)
    # 5 left-only (no right)
    for i in range(10, 15):
        c.on_left(float(i))
    assert c.paired_count == 10
    assert c.left_count == 15
    assert c.right_count == 10


def test_match_stereo_stamps_full_match():
    """100 matched stamps within 10 ms → 100 pairs (bag offline algorithm)."""
    from orb_slam_bringup.metrics_recorder import match_stereo_stamps

    left = [i * 0.033 for i in range(100)]
    right = [t + 0.002 for t in left]  # 2 ms offset
    assert match_stereo_stamps(left, right, tolerance_s=0.01) == 100


def test_match_stereo_stamps_offset_by_one_second():
    """All right stamps shifted past the left span → 0 pairs."""
    from orb_slam_bringup.metrics_recorder import match_stereo_stamps

    # Compact burst of left stamps; rights all start 1 s after the last left
    # so no left/right pair is within 10 ms.
    left = [i * 0.033 for i in range(50)]  # ~0 .. 1.617 s
    right = [t + 2.0 for t in left]  # ~2.0 .. 3.617 s — no overlap
    assert match_stereo_stamps(left, right, tolerance_s=0.01) == 0


def test_match_stereo_stamps_partial_overlap():
    """Partial temporal overlap → only overlapping stamps pair; each right used once."""
    from orb_slam_bringup.metrics_recorder import match_stereo_stamps

    # left: 0..9, right: 5..14 (5 s overlap of 5 frames at integer stamps)
    left = [float(i) for i in range(10)]
    right = [float(i) for i in range(5, 15)]
    assert match_stereo_stamps(left, right, tolerance_s=0.01) == 5


def test_match_stereo_stamps_each_right_used_once():
    """Greedy nearest-match must not double-claim a right stamp."""
    from orb_slam_bringup.metrics_recorder import match_stereo_stamps

    left = [0.0, 0.001, 0.002]
    right = [0.0]  # only one right within tol of all three lefts
    assert match_stereo_stamps(left, right, tolerance_s=0.01) == 1


def test_validate_camera_info_matching_profile():
    """Live CameraInfo matching profile → validated True."""
    from orb_slam_bringup.metrics_recorder import validate_camera_info

    # K row-major 3x3; fx at [0]
    fx = 426.9840393066406
    k = [fx, 0.0, 430.8, 0.0, fx, 238.9, 0.0, 0.0, 1.0]
    # Right-camera P: Tx = -fx * baseline → P[3] = -fx * baseline
    baseline = 0.05018814280629158
    p = [fx, 0.0, 430.8, -fx * baseline, 0.0, fx, 238.9, 0.0, 0.0, 0.0, 1.0, 0.0]
    ok, details = validate_camera_info(
        width=848,
        height=480,
        k=k,
        p=p,
        expected_width=848,
        expected_height=480,
        expected_fx=426.984,
        expected_baseline_m=0.0501881428,
    )
    assert ok is True
    assert details["width"] == 848
    assert details["height"] == 480
    assert details["baseline_m"] == pytest.approx(baseline, abs=1e-6)


def test_validate_camera_info_wrong_width():
    from orb_slam_bringup.metrics_recorder import validate_camera_info

    fx = 426.984
    k = [fx, 0, 0, 0, fx, 0, 0, 0, 1]
    p = [fx, 0, 0, -fx * 0.05, 0, fx, 0, 0, 0, 0, 1, 0]
    ok, _ = validate_camera_info(
        width=640,
        height=480,
        k=k,
        p=p,
        expected_width=848,
        expected_height=480,
        expected_fx=426.984,
        expected_baseline_m=0.0501881428,
    )
    assert ok is False


def test_validate_camera_info_wrong_baseline():
    from orb_slam_bringup.metrics_recorder import validate_camera_info

    fx = 426.984
    k = [fx, 0, 0, 0, fx, 0, 0, 0, 1]
    # baseline 0.10 m — far from expected ~0.05
    p = [fx, 0, 0, -fx * 0.10, 0, fx, 0, 0, 0, 0, 1, 0]
    ok, details = validate_camera_info(
        width=848,
        height=480,
        k=k,
        p=p,
        expected_width=848,
        expected_height=480,
        expected_fx=426.984,
        expected_baseline_m=0.0501881428,
    )
    assert ok is False
    assert abs(details["baseline_m"] - 0.0501881428) > 1e-3
