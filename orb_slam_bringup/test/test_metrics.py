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
    TRACK_LOST,
    TRACK_OK,
    TRACK_RECENTLY_LOST,
    FrameSample,
    MapGridSample,
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
