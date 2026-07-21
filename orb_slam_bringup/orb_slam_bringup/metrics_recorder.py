"""Metrics recorder: pure aggregation + ROS node that writes run evidence artifacts.

Read-only w.r.t. ROS: only subscribes and writes files. No publishers/services.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image

# ---------------------------------------------------------------------------
# Tracking / event constants (mirror orb_slam3_msgs)
# ---------------------------------------------------------------------------

TRACK_NO_IMAGES = 0
TRACK_NOT_INITIALIZED = 1
TRACK_OK = 2
TRACK_RECENTLY_LOST = 3
TRACK_LOST = 4

EVENT_INITIALIZED = 0
EVENT_LOST = 1
EVENT_RELOCALIZED = 2
EVENT_LOOP_CLOSED = 3
EVENT_MAP_CREATED = 4
EVENT_MAP_MERGED = 5
EVENT_MAP_RESET = 6

EVENT_TYPE_NAMES = {
    EVENT_INITIALIZED: "INITIALIZED",
    EVENT_LOST: "LOST",
    EVENT_RELOCALIZED: "RELOCALIZED",
    EVENT_LOOP_CLOSED: "LOOP_CLOSED",
    EVENT_MAP_CREATED: "MAP_CREATED",
    EVENT_MAP_MERGED: "MAP_MERGED",
    EVENT_MAP_RESET: "MAP_RESET",
}

MAP_IDLE = 0
MAP_BUILDING = 1
MAP_PUBLISHED = 2
MAP_FAILED = 3

MAP_STATE_NAMES = {
    MAP_IDLE: "IDLE",
    MAP_BUILDING: "BUILDING",
    MAP_PUBLISHED: "PUBLISHED",
    MAP_FAILED: "FAILED",
}

# Nav2 PGM pixel values
PGM_OCCUPIED = 0
PGM_FREE = 254
PGM_UNKNOWN = 205

# PNG preview grayscale (occupied dark, free light, unknown mid)
PNG_OCCUPIED = 30
PNG_FREE = 220
PNG_UNKNOWN = 128


# ---------------------------------------------------------------------------
# Pure data samples (no ROS types) for aggregation / tests
# ---------------------------------------------------------------------------


@dataclass
class FrameSample:
    t: float
    tracking_state: int
    pose_valid: bool
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    graph_revision: int = 0


@dataclass
class TrackingEventSample:
    t: float
    type: int
    graph_revision: int = 0
    map_id: int = 0
    keyframe_id: int = 0
    detail: str = ""


@dataclass
class MapRevisionSample:
    t: float
    state: int
    graph_revision: int = 0
    map_revision: int = 0
    input_scan_count: int = 0
    committed_scan_count: int = 0
    duration_ms: float = 0.0
    detail: str = ""


@dataclass
class MapGridSample:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    # Row-major occupancy; 0 free, 100 occ, -1 unk.
    # Prefer np.ndarray (int16) on the live path to avoid multi-MB list copies.
    data: Sequence[int]


@dataclass
class TrajectoryPoint:
    t: float
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class MetricsAggregator:
    """Pure metric aggregation from timestamped samples (no ROS graph)."""

    bag_duration_s: float = 0.0
    frames: List[FrameSample] = field(default_factory=list)
    events: List[TrackingEventSample] = field(default_factory=list)
    revisions: List[MapRevisionSample] = field(default_factory=list)
    final_map: Optional[MapGridSample] = None
    # None = unmeasured (fail-closed at acceptance gates). Explicit 0 is measured-good.
    unresolved_scan_count: Optional[int] = None
    invalid_tf_committed: Optional[int] = None
    wheel_only_before_recovery: Optional[int] = None
    planarity_rejections: Optional[int] = None
    # Raw mapper rejection counters (observability only — NOT commit gates).
    tf_lookup_failures: Optional[int] = None
    wheel_interp_failures: Optional[int] = None
    bag: Optional[Dict[str, Any]] = None
    git: Optional[Dict[str, Any]] = None
    configuration_sha256: str = ""
    stereo: Optional[Dict[str, Any]] = None
    diagnostics: Optional[List[Any]] = None
    trajectories: Optional[Dict[str, List[Dict[str, float]]]] = None
    invalid_poses: int = 0
    deadlock: bool = False
    pgm_yaml_match: bool = True

    def compute(self) -> Dict[str, Any]:
        init_t = _first_init_time(self.events, self.frames)
        end_t = _end_time(self.frames, self.events, self.bag_duration_s)
        ok_duration, post_init = _ok_duration(self.frames, init_t, end_t)
        ok_ratio = (ok_duration / post_init) if post_init > 0 else 0.0
        loss_intervals = _loss_intervals(self.events)
        loops = [
            {
                "t": e.t,
                "graph_revision": e.graph_revision,
                "map_id": e.map_id,
                "detail": e.detail,
            }
            for e in self.events
            if e.type == EVENT_LOOP_CLOSED
        ]
        map_revisions = [
            {
                "t": r.t,
                "state": MAP_STATE_NAMES.get(r.state, str(r.state)),
                "graph_revision": r.graph_revision,
                "map_revision": r.map_revision,
                "input_scan_count": r.input_scan_count,
                "committed_scan_count": r.committed_scan_count,
                "duration_ms": r.duration_ms,
                "detail": r.detail,
            }
            for r in self.revisions
        ]
        published = [r for r in self.revisions if r.state == MAP_PUBLISHED]
        rebuild_times = [r.duration_ms for r in published]
        mean_rebuild = (
            sum(rebuild_times) / len(rebuild_times) if rebuild_times else 0.0
        )
        max_rebuild = max(rebuild_times) if rebuild_times else 0.0

        final_map: Dict[str, Any] = {}
        if self.final_map is not None:
            free, occ, unk = count_occupancy_cells(self.final_map.data)
            final_map = {
                "width": self.final_map.width,
                "height": self.final_map.height,
                "resolution": self.final_map.resolution,
                "origin": [
                    self.final_map.origin_x,
                    self.final_map.origin_y,
                    self.final_map.origin_yaw,
                ],
                "free_cells": free,
                "occupied_cells": occ,
                "unknown_cells": unk,
                "pgm_yaml_match": self.pgm_yaml_match,
            }

        event_types_ordered = [
            EVENT_TYPE_NAMES.get(e.type, str(e.type)) for e in self.events
        ]

        return {
            "bag": self.bag if self.bag is not None else {"duration_s": self.bag_duration_s},
            "git": self.git if self.git is not None else {},
            "configuration_sha256": self.configuration_sha256,
            "stereo": self.stereo if self.stereo is not None else {},
            "tracking": {
                "initialized": init_t is not None,
                "init_time_s": init_t if init_t is not None else None,
                "post_init_duration_s": post_init,
                "ok_duration_s": ok_duration,
                "ok_ratio_after_init": ok_ratio,
                "loop_count": len(loops),
                "loss_intervals": loss_intervals,
                "mean_rebuild_ms": mean_rebuild,
                "max_rebuild_ms": max_rebuild,
                "invalid_poses": self.invalid_poses,
                "deadlock": self.deadlock,
            },
            "fallback": {
                "unresolved_scan_count": self.unresolved_scan_count,
                "invalid_tf_committed": self.invalid_tf_committed,
                "wheel_only_before_recovery": self.wheel_only_before_recovery,
                "planarity_rejections": self.planarity_rejections,
                # Rejection counters (safe skips) — do not confuse with commits.
                "tf_lookup_failures": self.tf_lookup_failures,
                "wheel_interp_failures": self.wheel_interp_failures,
                "measured": (
                    self.invalid_tf_committed is not None
                    and self.wheel_only_before_recovery is not None
                ),
            },
            "loops": loops,
            "map_revisions": map_revisions,
            "final_map": final_map,
            "diagnostics": self.diagnostics if self.diagnostics is not None else [],
            "event_types_ordered": event_types_ordered,
            "trajectories": self.trajectories if self.trajectories is not None else {},
        }


def _first_init_time(
    events: Sequence[TrackingEventSample], frames: Sequence[FrameSample]
) -> Optional[float]:
    for e in events:
        if e.type == EVENT_INITIALIZED:
            return e.t
    for f in frames:
        if f.tracking_state == TRACK_OK:
            return f.t
    return None


def _end_time(
    frames: Sequence[FrameSample],
    events: Sequence[TrackingEventSample],
    bag_duration_s: float,
) -> float:
    candidates = [bag_duration_s]
    if frames:
        candidates.append(frames[-1].t)
    if events:
        candidates.append(events[-1].t)
    return max(candidates) if candidates else 0.0


def _ok_duration(
    frames: Sequence[FrameSample],
    init_t: Optional[float],
    end_t: float,
) -> Tuple[float, float]:
    """Interval-based OK duration after init using consecutive frame samples."""
    if init_t is None or not frames:
        return 0.0, 0.0
    post_init = max(0.0, end_t - init_t)
    # Sort by time
    ordered = sorted(frames, key=lambda f: f.t)
    ok = 0.0
    for i in range(len(ordered) - 1):
        a, b = ordered[i], ordered[i + 1]
        # Interval [a.t, b.t) counts as OK if a is OK and a.t >= init_t
        # (state holds until next sample)
        if a.t < init_t:
            # partial interval after init
            if b.t <= init_t:
                continue
            if a.tracking_state == TRACK_OK:
                ok += b.t - init_t
            continue
        if a.tracking_state == TRACK_OK:
            ok += b.t - a.t
    # If last frame is after init and we have end_t > last, do not extend
    # (tests use last frame == end_t)
    return ok, post_init


def _loss_intervals(events: Sequence[TrackingEventSample]) -> List[Dict[str, float]]:
    intervals: List[Dict[str, float]] = []
    open_start: Optional[float] = None
    for e in sorted(events, key=lambda x: x.t):
        if e.type == EVENT_LOST and open_start is None:
            open_start = e.t
        elif e.type == EVENT_RELOCALIZED and open_start is not None:
            intervals.append(
                {
                    "start_s": open_start,
                    "end_s": e.t,
                    "duration_s": e.t - open_start,
                }
            )
            open_start = None
    return intervals


def count_occupancy_cells(data: Sequence[int]) -> Tuple[int, int, int]:
    """Count free (0), occupied (100 or other positive), unknown (<0)."""
    arr = np.asarray(data, dtype=np.int16)
    free = int(np.count_nonzero(arr == 0))
    unk = int(np.count_nonzero(arr < 0))
    occ = int(arr.size - free - unk)
    return free, occ, unk


def occupancy_grid_to_png_array(
    width: int, height: int, data: Sequence[int]
) -> np.ndarray:
    """Grayscale uint8 array (H, W): free light, occupied dark, unknown mid.

    Semantics match count_occupancy_cells: free is exactly 0, any positive is
    occupied, negative is unknown. Vectorized for mid-run map previews.
    """
    if width <= 0 or height <= 0:
        return np.zeros((0, 0), dtype=np.uint8)
    arr = np.asarray(data, dtype=np.int16).reshape((height, width))
    gray = np.full((height, width), PNG_UNKNOWN, dtype=np.uint8)
    gray[arr == 0] = PNG_FREE
    gray[arr > 0] = PNG_OCCUPIED
    return gray


def save_map_revision_png(
    grid: MapGridSample, path: Path
) -> None:
    arr = occupancy_grid_to_png_array(grid.width, grid.height, grid.data)
    # Flip vertically so world +y is up in image (match PGM convention)
    arr = np.flipud(arr)
    Image.fromarray(arr, mode="L").save(path)


class EventJsonlWriter:
    """Append-only JSONL writer that keeps a single open handle.

    Avoids open/close per event on the hot tracked_frame path. Thread-safe.
    """

    def __init__(self, path: Path) -> None:
        import threading

        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        # Truncate at start so each run is self-contained.
        self._fh = self._path.open("w", encoding="utf-8")
        self._lock = threading.Lock()

    def append(self, obj: Dict[str, Any]) -> None:
        with self._lock:
            if self._fh is not None:
                self._fh.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.flush()
                    self._fh.close()
                finally:
                    self._fh = None  # type: ignore[assignment]


class MapRevisionPngGate:
    """Rate-limit mid-run map-revision PNGs.

    Even with a background writer, unbounded PNG encode/write contends for the
    GIL and disk and can starve the single-threaded rclpy executor. Default
    one PNG per second is enough for dashboard/report previews; final-map is
    always written on flush regardless of this gate.
    """

    def __init__(self, min_interval_s: float = 1.0) -> None:
        self._min_interval_s = float(min_interval_s)
        self._last_write_t: Optional[float] = None

    def should_write(self, t: float, map_revision: int = 0) -> bool:
        del map_revision  # reserved for future "always write revision N" policy
        if self._min_interval_s <= 0.0:
            return True
        if self._last_write_t is None or (t - self._last_write_t) >= self._min_interval_s:
            self._last_write_t = float(t)
            return True
        return False


class MapRevisionPngQueue:
    """Background worker for map-revision PNG encode/write.

    Callers enqueue a snapshot of the grid; join() drains pending work so
    artifacts exist before process exit. ROS callbacks must not block on I/O.
    """

    def __init__(self) -> None:
        import queue
        import threading

        self._queue: "queue.Queue[Optional[Tuple[MapGridSample, Path]]]" = queue.Queue()
        self._thread = threading.Thread(
            target=self._run, name="map-revision-png-writer", daemon=True
        )
        self._thread.start()

    def enqueue(self, grid: MapGridSample, path: Path) -> None:
        # Snapshot grid data so callers can mutate the live map safely.
        if isinstance(grid.data, np.ndarray):
            data: Sequence[int] = grid.data.copy()
        else:
            data = np.asarray(grid.data, dtype=np.int16).copy()
        snap = MapGridSample(
            width=grid.width,
            height=grid.height,
            resolution=grid.resolution,
            origin_x=grid.origin_x,
            origin_y=grid.origin_y,
            origin_yaw=grid.origin_yaw,
            data=data,
        )
        self._queue.put((snap, Path(path)))

    def join(self) -> None:
        """Block until all enqueued jobs finish, then stop the worker."""
        self._queue.put(None)
        self._thread.join(timeout=120.0)

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                grid, path = item
                try:
                    save_map_revision_png(grid, path)
                except Exception:  # noqa: BLE001
                    # Best-effort artifact; never kill the worker.
                    pass
            finally:
                self._queue.task_done()


def write_nav2_map(grid: MapGridSample, path_stem: Path | str) -> None:
    """Write Nav2-compatible final-map.pgm + final-map.yaml from grid sample.

    PGM (P5): occupied=0, free=254, unknown=205. Image is flipped so map
    origin is bottom-left (map_server convention).
    """
    stem = Path(path_stem)
    pgm_path = Path(str(stem) + ".pgm")
    yaml_path = Path(str(stem) + ".yaml")

    w, h = grid.width, grid.height
    # OccupancyGrid: row 0 is y=origin (bottom). PGM row 0 is top → flip.
    arr = np.asarray(grid.data, dtype=np.int16).reshape((h, w))
    arr = np.flipud(arr)
    pixels = np.full(arr.shape, PGM_UNKNOWN, dtype=np.uint8)
    pixels[arr == 0] = PGM_FREE
    pixels[arr > 0] = PGM_OCCUPIED

    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    atomic_write_bytes(pgm_path, header + pixels.tobytes())

    meta = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(grid.resolution),
        "origin": [
            float(grid.origin_x),
            float(grid.origin_y),
            float(grid.origin_yaw),
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    # YAML dump without external pyyaml requirement for write path: hand-format
    # for determinism (tests use yaml.safe_load which needs PyYAML — available).
    lines = [
        f"image: {meta['image']}",
        f"mode: {meta['mode']}",
        f"resolution: {meta['resolution']}",
        f"origin: [{meta['origin'][0]}, {meta['origin'][1]}, {meta['origin'][2]}]",
        f"negate: {meta['negate']}",
        f"occupied_thresh: {meta['occupied_thresh']}",
        f"free_thresh: {meta['free_thresh']}",
        "",
    ]
    atomic_write_text(yaml_path, "\n".join(lines))


def atomic_write_text(path: Path | str, text: str, encoding: str = "utf-8") -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        tmp.write_text(text, encoding=encoding)
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            try:
                tmp.unlink()
            except OSError:
                pass


def atomic_write_bytes(path: Path | str, data: bytes) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        tmp.write_bytes(data)
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            try:
                tmp.unlink()
            except OSError:
                pass


def atomic_write_json(path: Path | str, obj: Any) -> None:
    atomic_write_text(path, json.dumps(obj, indent=2, sort_keys=False) + "\n")


def configuration_sha256(paths: Sequence[Path | str]) -> str:
    h = hashlib.sha256()
    for p in paths:
        path = Path(p)
        h.update(str(path.resolve()).encode("utf-8"))
        h.update(b"\0")
        if path.is_file():
            h.update(path.read_bytes())
        h.update(b"\0")
    return h.hexdigest()


def capture_git_info(repo_dir: Path | str | None = None) -> Dict[str, Any]:
    """Capture git commit + dirty state of the independent repo."""
    cwd = str(repo_dir) if repo_dir is not None else None
    info: Dict[str, Any] = {"commit": "", "dirty": False, "describe": ""}
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        info["commit"] = commit
        status = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        info["dirty"] = bool(status.strip())
        try:
            info["describe"] = subprocess.check_output(
                ["git", "describe", "--always", "--dirty"],
                cwd=cwd,
                stderr=subprocess.DEVNULL,
                text=True,
            ).strip()
        except subprocess.CalledProcessError:
            info["describe"] = commit[:8]
    except (subprocess.CalledProcessError, FileNotFoundError, OSError):
        pass
    return info


def write_trajectory_csv(path: Path, points: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        with tmp.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["t", "x", "y", "yaw"])
            w.writeheader()
            for p in points:
                w.writerow(
                    {
                        "t": p.get("t", 0.0),
                        "x": p.get("x", 0.0),
                        "y": p.get("y", 0.0),
                        "yaw": p.get("yaw", 0.0),
                    }
                )
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            try:
                tmp.unlink()
            except OSError:
                pass


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def build_live_stereo_section(
    *,
    expected_pairs: int = 6633,
    paired_count: int = 0,
    width: int = 848,
    height: int = 480,
    baseline_m: float = 0.0501881428,
    camera_validated: bool = False,
    measured: Optional[bool] = None,
) -> Dict[str, Any]:
    """Stereo metrics for live flush — fail-closed defaults.

    Does NOT invent paired_count from expected_pairs. camera_validated stays
    False unless a live CameraInfo validation path sets it True.
    Loading YAML profile values alone is NOT validation.
    """
    expected = int(expected_pairs) if expected_pairs > 0 else 6633
    paired = max(0, int(paired_count))
    ratio = (paired / float(expected)) if expected > 0 else 0.0
    if measured is None:
        measured_flag = paired > 0 or bool(camera_validated)
    else:
        measured_flag = bool(measured)
    return {
        "expected_pairs": expected,
        "paired_count": paired,
        "paired_ratio": ratio,
        "width": int(width),
        "height": int(height),
        "baseline_m": float(baseline_m),
        "camera_validated": bool(camera_validated),
        "measured": measured_flag,
    }


# Default stereo pairing tolerance: |t_left - t_right| <= 10 ms
STEREO_PAIR_TOLERANCE_S = 0.01


def match_stereo_stamps(
    left_stamps: Sequence[float],
    right_stamps: Sequence[float],
    tolerance_s: float = STEREO_PAIR_TOLERANCE_S,
) -> int:
    """Greedy nearest-match stereo pairing; each right stamp used at most once.

    Mirrors the controller-verified offline algorithm (bisect + first unused
    right within [lt-tol, lt+tol]). Deterministic and independent of live QoS.
    """
    import bisect

    if not left_stamps or not right_stamps:
        return 0
    R = sorted(float(t) for t in right_stamps)
    used = [False] * len(R)
    pairs = 0
    tol = float(tolerance_s)
    for lt in left_stamps:
        lt = float(lt)
        i = bisect.bisect_left(R, lt - tol)
        while i < len(R) and R[i] <= lt + tol:
            if not used[i]:
                used[i] = True
                pairs += 1
                break
            i += 1
    return pairs


def count_stereo_pairs_from_bag(
    bag_path: str | Path,
    left_info_topic: str,
    right_info_topic: str,
    tolerance_s: float = STEREO_PAIR_TOLERANCE_S,
    expected: int = 6633,
) -> Tuple[int, int, int]:
    """Count stereo CameraInfo pairs from an immutable bag (offline, deterministic).

    Reads only the two CameraInfo topics' header stamps via rosbag2_py, then
    runs :func:`match_stereo_stamps`. Lazy-imports rosbag2_py so unit tests that
    never touch bags do not require it.

    Returns (paired_count, left_count, right_count).
    ``expected`` is unused for counting (kept for API symmetry / callers); the
    gate still divides paired_count by its own expected_pairs elsewhere.
    """
    del expected  # not used for counting — gate uses expected_pairs separately
    # Lazy import: pure unit tests must not require rosbag2_py.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f"bag path not found: {bag_path}")

    storage_id = "mcap"
    # Infer storage from metadata or file extension when possible.
    meta = bag_path / "metadata.yaml"
    if meta.is_file():
        try:
            import yaml

            md = yaml.safe_load(meta.read_text(encoding="utf-8")) or {}
            info = md.get("rosbag2_bagfile_information") or md
            sid = info.get("storage_identifier") or info.get("storage_id")
            if sid:
                storage_id = str(sid)
        except Exception:  # noqa: BLE001
            pass
    elif bag_path.suffix == ".db3":
        storage_id = "sqlite3"
    elif bag_path.suffix == ".mcap":
        storage_id = "mcap"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if left_info_topic not in topic_types or right_info_topic not in topic_types:
        # Topics absent → zero counts (genuine missing stereo must fail the gate).
        return 0, 0, 0

    left_type = get_message(topic_types[left_info_topic])
    right_type = get_message(topic_types[right_info_topic])
    # Filter to the two CameraInfo topics only.
    try:
        from rosbag2_py import StorageFilter

        reader.set_filter(
            StorageFilter(topics=[left_info_topic, right_info_topic])
        )
    except Exception:  # noqa: BLE001
        pass

    left_stamps: List[float] = []
    right_stamps: List[float] = []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == left_info_topic:
            msg = deserialize_message(data, left_type)
            stamp = msg.header.stamp
            left_stamps.append(float(stamp.sec) + float(stamp.nanosec) * 1e-9)
        elif topic == right_info_topic:
            msg = deserialize_message(data, right_type)
            stamp = msg.header.stamp
            right_stamps.append(float(stamp.sec) + float(stamp.nanosec) * 1e-9)

    paired = match_stereo_stamps(left_stamps, right_stamps, tolerance_s=tolerance_s)
    return paired, len(left_stamps), len(right_stamps)


@dataclass
class StereoPairCounter:
    """Count stereo pairs by matching left/right CameraInfo (or image) stamps.

    Only stamps are retained — never pixel data. Buffers are bounded so a
    long bag does not grow unbounded when one side drops frames.
    """

    tolerance_s: float = STEREO_PAIR_TOLERANCE_S
    max_buffer: int = 64
    _left: List[float] = field(default_factory=list)
    _right: List[float] = field(default_factory=list)
    paired_count: int = 0
    left_count: int = 0
    right_count: int = 0

    @property
    def streams_seen(self) -> bool:
        return self.left_count > 0 or self.right_count > 0

    def on_left(self, t: float) -> None:
        self.left_count += 1
        self._left.append(float(t))
        self._trim(self._left)
        self._try_match_from_left()

    def on_right(self, t: float) -> None:
        self.right_count += 1
        self._right.append(float(t))
        self._trim(self._right)
        self._try_match_from_right()

    def _trim(self, buf: List[float]) -> None:
        if len(buf) > self.max_buffer:
            del buf[: len(buf) - self.max_buffer]

    def _try_match_from_left(self) -> None:
        """Newest left tries to claim nearest unmatched right within tolerance."""
        if not self._left or not self._right:
            return
        t = self._left[-1]
        best_i = -1
        best_dt = self.tolerance_s + 1.0
        for i, tr in enumerate(self._right):
            dt = abs(tr - t)
            if dt <= self.tolerance_s and dt < best_dt:
                best_dt = dt
                best_i = i
        if best_i >= 0:
            self._right.pop(best_i)
            self._left.pop()
            self.paired_count += 1

    def _try_match_from_right(self) -> None:
        if not self._left or not self._right:
            return
        t = self._right[-1]
        best_i = -1
        best_dt = self.tolerance_s + 1.0
        for i, tl in enumerate(self._left):
            dt = abs(tl - t)
            if dt <= self.tolerance_s and dt < best_dt:
                best_dt = dt
                best_i = i
        if best_i >= 0:
            self._left.pop(best_i)
            self._right.pop()
            self.paired_count += 1


def validate_camera_info(
    *,
    width: int,
    height: int,
    k: Sequence[float],
    p: Sequence[float],
    expected_width: int = 848,
    expected_height: int = 480,
    expected_fx: float = 426.984,
    expected_baseline_m: float = 0.0501881428,
    fx_tol: float = 1.0,
    baseline_tol: float = 1e-4,
) -> Tuple[bool, Dict[str, Any]]:
    """Validate live CameraInfo against the bag profile.

    Checks width/height, K[0] (fx), and stereo baseline from P[3]/(-fx)
    (right-camera projection convention: Tx = -fx * baseline).
    """
    fx = float(k[0]) if len(k) >= 1 else 0.0
    # Baseline from right P: P[3] = -fx * baseline_m  →  baseline = -P[3]/fx
    baseline_m = 0.0
    if len(p) >= 4 and abs(fx) > 1e-9:
        baseline_m = abs(float(p[3]) / fx)

    details: Dict[str, Any] = {
        "width": int(width),
        "height": int(height),
        "fx": fx,
        "baseline_m": baseline_m,
    }
    ok = (
        int(width) == int(expected_width)
        and int(height) == int(expected_height)
        and abs(fx - float(expected_fx)) <= fx_tol
        and abs(baseline_m - float(expected_baseline_m)) <= baseline_tol
    )
    details["ok"] = ok
    return ok, details


def fallback_from_diagnostics(
    kv: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Map mapper /diagnostics KeyValues to fallback counters.

    Commit-invariant gates (invalid_tf_committed, wheel_only_before_recovery)
    mean "scans WRONGLY COMMITTED to the map". By mapper design those are
    always 0: tf_lookup_failures count REJECTED scans, wheel_interp_failures
    count skipped anchors / provisional wheel-only display during loss, and
    planarity_rejections count excluded non-planar poses — none are commits
    of bad data. Mapping rejections onto commit gates was a category error.

    When any mapper diagnostic KVs are observed we therefore report the
    commit invariants as measured 0 (mapper contract), while still exposing
    the raw rejection counters for observability. Explicit violation KVs
    (invalid_tf_committed / wheel_only_before_recovery) override if present.
    Until diagnostics are seen at all, counters stay None (fail-closed).
    """
    if not kv:
        return {
            "unresolved_scan_count": None,
            "invalid_tf_committed": None,
            "wheel_only_before_recovery": None,
            "planarity_rejections": None,
            "tf_lookup_failures": None,
            "wheel_interp_failures": None,
            "measured": False,
        }

    def _as_int(key: str) -> Optional[int]:
        if key not in kv:
            return None
        try:
            return int(kv[key])
        except (TypeError, ValueError):
            return None

    tf_fail = _as_int("tf_lookup_failures")
    wheel_fail = _as_int("wheel_interp_failures")
    planarity = _as_int("planarity_rejections")
    unresolved = _as_int("unresolved_scan_count")

    # Prefer explicit violation counters if the mapper ever publishes them.
    inv_tf = _as_int("invalid_tf_committed")
    wheel_only = _as_int("wheel_only_before_recovery")

    # Any observed mapper diagnostic makes commit invariants measurable-by-
    # design (0 unless an explicit violation counter says otherwise).
    any_mapper_kv = any(
        v is not None
        for v in (tf_fail, wheel_fail, planarity, unresolved, inv_tf, wheel_only)
    )
    if any_mapper_kv:
        if inv_tf is None:
            inv_tf = 0  # mapper never commits invalid-TF scans
        if wheel_only is None:
            wheel_only = 0  # mapper never commits wheel-only scans
    measured = inv_tf is not None and wheel_only is not None
    return {
        "unresolved_scan_count": unresolved,
        "invalid_tf_committed": inv_tf,
        "wheel_only_before_recovery": wheel_only,
        "planarity_rejections": planarity,
        "tf_lookup_failures": tf_fail,
        "wheel_interp_failures": wheel_fail,
        "measured": measured,
    }


def extract_diagnostic_kvs(diagnostics: Sequence[Any]) -> Dict[str, Any]:
    """Flatten latest KeyValue pairs from stored diagnostic snapshots.

    Accepts either:
    - list of status dicts with optional ``values`` list of {key,value}
    - list of already-flattened kv dicts
    Later entries overwrite earlier ones (latest wins).
    """
    out: Dict[str, Any] = {}
    if not diagnostics:
        return out
    for entry in diagnostics:
        if not isinstance(entry, dict):
            continue
        values = entry.get("values")
        if isinstance(values, list):
            for kv in values:
                if isinstance(kv, dict) and "key" in kv:
                    out[str(kv["key"])] = kv.get("value")
        else:
            # Already a flat map of counters
            for k, v in entry.items():
                if k in (
                    "name",
                    "level",
                    "message",
                    "hardware_id",
                    "values",
                ):
                    continue
                out[str(k)] = v
    return out


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------


class MetricsRecorderNode:
    """ROS metrics recorder (constructed inside main after rclpy.init)."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        # Build as Node via composition to keep module importable without ROS
        # for pure unit tests. We subclass dynamically.
        class _Node(Node):
            pass

        self._node = _Node("metrics_recorder")
        n = self._node

        n.declare_parameter("artifact_dir", "/tmp/orb_slam_artifacts")
        n.declare_parameter("bag_path", "")
        n.declare_parameter("bag_duration_s", 0.0)
        n.declare_parameter("config_path", "")
        n.declare_parameter("repo_dir", "")
        n.declare_parameter("expected_stereo_pairs", 6633)
        # Min sim-time between mid-run map-revision-*.png writes. 0 = every
        # published revision (old behavior; can starve tracked_frame capture).
        n.declare_parameter("map_revision_png_min_interval_s", 1.0)
        n.declare_parameter(
            "left_camera_info_topic", "/camera/camera/infra1/camera_info"
        )
        n.declare_parameter(
            "right_camera_info_topic", "/camera/camera/infra2/camera_info"
        )
        if not n.has_parameter("use_sim_time"):
            n.declare_parameter("use_sim_time", True)

        self._artifact_dir = Path(
            n.get_parameter("artifact_dir").get_parameter_value().string_value
        )
        self._bag_path = n.get_parameter("bag_path").get_parameter_value().string_value
        self._bag_duration_s = (
            n.get_parameter("bag_duration_s").get_parameter_value().double_value
        )
        self._config_path = n.get_parameter("config_path").get_parameter_value().string_value
        self._repo_dir = n.get_parameter("repo_dir").get_parameter_value().string_value
        try:
            self._expected_pairs = int(
                n.get_parameter("expected_stereo_pairs")
                .get_parameter_value()
                .integer_value
            )
        except Exception:  # noqa: BLE001
            self._expected_pairs = 6633
        if self._expected_pairs <= 0:
            self._expected_pairs = 6633

        self._left_info_topic = (
            n.get_parameter("left_camera_info_topic")
            .get_parameter_value()
            .string_value
            or "/camera/camera/infra1/camera_info"
        )
        self._right_info_topic = (
            n.get_parameter("right_camera_info_topic")
            .get_parameter_value()
            .string_value
            or "/camera/camera/infra2/camera_info"
        )
        left_info_topic = self._left_info_topic
        right_info_topic = self._right_info_topic

        self._artifact_dir.mkdir(parents=True, exist_ok=True)
        self._events_path = self._artifact_dir / "events.jsonl"
        # Keep a single open handle — open/close per tracked_frame was wasteful
        # and mid-run PNG I/O on the same spin thread starved frame capture.
        self._events_writer = EventJsonlWriter(self._events_path)
        self._png_queue = MapRevisionPngQueue()
        import threading
        self._lock = threading.Lock()
        try:
            png_interval = float(
                n.get_parameter("map_revision_png_min_interval_s")
                .get_parameter_value()
                .double_value
            )
        except Exception:  # noqa: BLE001
            png_interval = 1.0
        self._png_gate = MapRevisionPngGate(min_interval_s=png_interval)

        self._frames: List[FrameSample] = []
        self._events: List[TrackingEventSample] = []
        self._revisions: List[MapRevisionSample] = []
        self._orb_traj: List[Dict[str, float]] = []
        self._wheel_traj: List[Dict[str, float]] = []
        self._corrected_traj: List[Dict[str, float]] = []
        self._last_map: Optional[MapGridSample] = None
        self._diagnostics: List[Any] = []
        self._invalid_poses = 0
        self._flushed = False
        self._revision_png_count = 0
        # Live stereo: timestamp pairing on CameraInfo (tiny; same stamps as images).
        # Must NOT substitute expected_pairs when unobserved (fail closed).
        self._stereo_pairs = StereoPairCounter(tolerance_s=STEREO_PAIR_TOLERANCE_S)
        self._camera_validated = False  # only True after live CameraInfo check
        self._live_width = 848
        self._live_height = 480
        self._live_baseline_m = 0.0501881428
        self._expected_fx = 426.984
        self._expected_width = 848
        self._expected_height = 480
        self._expected_baseline_m = 0.0501881428
        # Latest mapper diagnostic KeyValues (tf_lookup_failures, …)
        self._diag_kv: Dict[str, Any] = {}

        # Prefill expected camera geometry from profile if available (display only;
        # camera_validated stays False until a live CameraInfo confirms).
        self._load_profile_camera_defaults()

        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        # Separate groups so MultiThreadedExecutor can service tracked_frame
        # while a map/PNG-related callback is busy.
        self._tracking_cb_group = MutuallyExclusiveCallbackGroup()
        self._map_cb_group = MutuallyExclusiveCallbackGroup()
        self._misc_cb_group = MutuallyExclusiveCallbackGroup()

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Only the latest map is useful for previews/final export; deep queues
        # of multi-MB OccupancyGrid messages starve the recorder.
        reliable_transient_latest = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,  # was 50; absorb brief map-side stalls
        )
        # Sensor-data QoS for bag camera streams (BEST_EFFORT KEEP_LAST)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        from nav_msgs.msg import OccupancyGrid
        from nav_msgs.msg import Path as NavPath
        from sensor_msgs.msg import CameraInfo
        from orb_slam3_msgs.msg import (
            MapRevision,
            RevisionedPath,
            TrackedFrame,
            TrackingEvent,
        )
        from diagnostic_msgs.msg import DiagnosticArray

        n.create_subscription(
            TrackedFrame,
            "/orb_slam3/tracked_frame",
            self._on_tracked_frame,
            best_effort,
            callback_group=self._tracking_cb_group,
        )
        n.create_subscription(
            TrackingEvent,
            "/orb_slam3/events",
            self._on_tracking_event,
            reliable,
            callback_group=self._tracking_cb_group,
        )
        n.create_subscription(
            MapRevision,
            "/orb_lidar/map_revision",
            self._on_map_revision,
            reliable,
            callback_group=self._map_cb_group,
        )
        n.create_subscription(
            OccupancyGrid,
            "/orb_lidar/map",
            self._on_map,
            reliable_transient_latest,
            callback_group=self._map_cb_group,
        )
        n.create_subscription(
            RevisionedPath,
            "/orb_lidar/corrected_path_revisioned",
            self._on_corrected_path,
            reliable,
            callback_group=self._misc_cb_group,
        )
        n.create_subscription(
            NavPath,
            "/orb_lidar/wheel_path",
            self._on_wheel_path,
            reliable,
            callback_group=self._misc_cb_group,
        )
        n.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._on_diagnostics,
            best_effort,
            callback_group=self._misc_cb_group,
        )
        # Stereo pair counting + camera validation (READ-ONLY CameraInfo stamps)
        n.create_subscription(
            CameraInfo,
            left_info_topic,
            self._on_left_camera_info,
            sensor_qos,
            callback_group=self._misc_cb_group,
        )
        n.create_subscription(
            CameraInfo,
            right_info_topic,
            self._on_right_camera_info,
            sensor_qos,
            callback_group=self._misc_cb_group,
        )

        # Flush on shutdown
        try:
            n.context.on_shutdown(self._flush)
        except Exception:  # noqa: BLE001
            pass

        n.get_logger().info(
            f"metrics_recorder writing artifacts to {self._artifact_dir}; "
            f"stereo CameraInfo: {left_info_topic}, {right_info_topic}"
        )

    @property
    def node(self):
        return self._node

    def _stamp_to_s(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _append_event_jsonl(self, obj: Dict[str, Any]) -> None:
        try:
            self._events_writer.append(obj)
        except Exception:  # noqa: BLE001
            # Never let artifact I/O drop tracking samples.
            pass

    def _load_profile_camera_defaults(self) -> None:
        """Load nominal camera geometry from profile YAML (not validation)."""
        candidates: List[Path] = []
        if self._config_path:
            candidates.append(Path(self._config_path))
        try:
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("orb_slam_bringup"))
            p = share / "config" / "tasterobot_bag.yaml"
            if p.is_file():
                candidates.append(p)
        except Exception:  # noqa: BLE001
            pass
        src = Path(__file__).resolve().parents[1] / "config" / "tasterobot_bag.yaml"
        if src.is_file():
            candidates.append(src)
        for p in candidates:
            try:
                import yaml

                prof = yaml.safe_load(p.read_text(encoding="utf-8"))
                cam = (prof or {}).get("camera", {}) or {}
                if "width" in cam:
                    self._expected_width = int(cam["width"])
                    self._live_width = self._expected_width
                if "height" in cam:
                    self._expected_height = int(cam["height"])
                    self._live_height = self._expected_height
                if "baseline_m" in cam:
                    self._expected_baseline_m = float(cam["baseline_m"])
                    self._live_baseline_m = self._expected_baseline_m
                if "fx" in cam:
                    self._expected_fx = float(cam["fx"])
                bag = (prof or {}).get("bag", {}) or {}
                if not self._bag_duration_s and "duration_s" in bag:
                    self._bag_duration_s = float(bag["duration_s"])
                if not self._bag_path and bag.get("path"):
                    self._bag_path = str(bag["path"])
                return
            except Exception:  # noqa: BLE001
                continue

    def _on_left_camera_info(self, msg) -> None:
        t = self._stamp_to_s(msg.header.stamp)
        self._stereo_pairs.on_left(t)
        # Prefer left CameraInfo for width/height/fx; baseline needs right P.
        self._live_width = int(msg.width)
        self._live_height = int(msg.height)

    def _on_right_camera_info(self, msg) -> None:
        t = self._stamp_to_s(msg.header.stamp)
        self._stereo_pairs.on_right(t)
        # Right CameraInfo carries stereo baseline in P[3] = -fx * baseline.
        try:
            k = list(msg.k)
            p = list(msg.p)
            ok, details = validate_camera_info(
                width=int(msg.width),
                height=int(msg.height),
                k=k,
                p=p,
                expected_width=self._expected_width,
                expected_height=self._expected_height,
                expected_fx=self._expected_fx,
                expected_baseline_m=self._expected_baseline_m,
            )
            if details.get("baseline_m"):
                self._live_baseline_m = float(details["baseline_m"])
            if ok:
                self._camera_validated = True
                self._live_width = int(details["width"])
                self._live_height = int(details["height"])
        except Exception:  # noqa: BLE001
            pass

    def _on_tracked_frame(self, msg) -> None:
        t = self._stamp_to_s(msg.header.stamp)
        pose_valid = bool(msg.pose_valid)
        if not pose_valid and msg.tracking_state == TRACK_OK:
            self._invalid_poses += 1
        x = y = yaw = 0.0
        if pose_valid:
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            q = msg.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self._orb_traj.append({"t": t, "x": x, "y": y, "yaw": yaw})
        sample = FrameSample(
            t=t,
            tracking_state=int(msg.tracking_state),
            pose_valid=pose_valid,
            x=x,
            y=y,
            yaw=yaw,
            graph_revision=int(msg.graph_revision),
        )
        self._frames.append(sample)
        self._append_event_jsonl(
            {
                "kind": "tracked_frame",
                "t": t,
                "tracking_state": int(msg.tracking_state),
                "pose_valid": pose_valid,
                "graph_revision": int(msg.graph_revision),
            }
        )

    def _on_tracking_event(self, msg) -> None:
        t = self._stamp_to_s(msg.header.stamp)
        sample = TrackingEventSample(
            t=t,
            type=int(msg.type),
            graph_revision=int(msg.graph_revision),
            map_id=int(msg.map_id),
            keyframe_id=int(msg.keyframe_id),
            detail=str(msg.detail),
        )
        self._events.append(sample)
        self._append_event_jsonl(
            {
                "kind": "tracking_event",
                "t": t,
                "type": EVENT_TYPE_NAMES.get(int(msg.type), str(msg.type)),
                "type_code": int(msg.type),
                "graph_revision": int(msg.graph_revision),
                "map_id": int(msg.map_id),
                "detail": str(msg.detail),
            }
        )

    def _on_map_revision(self, msg) -> None:
        t = self._stamp_to_s(msg.header.stamp)
        sample = MapRevisionSample(
            t=t,
            state=int(msg.state),
            graph_revision=int(msg.graph_revision),
            map_revision=int(msg.map_revision),
            input_scan_count=int(msg.input_scan_count),
            committed_scan_count=int(msg.committed_scan_count),
            duration_ms=float(msg.duration_ms),
            detail=str(msg.detail),
        )
        self._revisions.append(sample)
        self._append_event_jsonl(
            {
                "kind": "map_revision",
                "t": t,
                "state": MAP_STATE_NAMES.get(int(msg.state), str(msg.state)),
                "graph_revision": int(msg.graph_revision),
                "map_revision": int(msg.map_revision),
                "duration_ms": float(msg.duration_ms),
                "input_scan_count": int(msg.input_scan_count),
                "committed_scan_count": int(msg.committed_scan_count),
            }
        )
        # On PUBLISHED, rate-limit + queue PNG off the executor thread so
        # tracked_frame callbacks are not starved by encode/disk/GIL work.
        if int(msg.state) == MAP_PUBLISHED and self._last_map is not None:
            n = int(msg.map_revision) if msg.map_revision else (
                self._revision_png_count + 1
            )
            self._revision_png_count = max(self._revision_png_count, n)
            if not self._png_gate.should_write(t=t, map_revision=n):
                return
            png_path = self._artifact_dir / f"map-revision-{n}.png"
            try:
                self._png_queue.enqueue(self._last_map, png_path)
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warning(f"map revision PNG enqueue failed: {exc}")

    def _on_map(self, msg) -> None:
        info = msg.info
        # Keep a compact int16 snapshot (not a Python int list) so map
        # publishes do not dominate the single-threaded executor.
        try:
            data: Sequence[int] = np.asarray(msg.data, dtype=np.int16).copy()
        except Exception:  # noqa: BLE001
            data = np.asarray([int(v) for v in msg.data], dtype=np.int16)
        origin = info.origin
        yaw = yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        self._last_map = MapGridSample(
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=float(yaw),
            data=data,
        )

    def _on_corrected_path(self, msg) -> None:
        pts: List[Dict[str, float]] = []
        for ps in msg.poses:
            t = self._stamp_to_s(ps.header.stamp)
            q = ps.pose.orientation
            pts.append(
                {
                    "t": t,
                    "x": float(ps.pose.position.x),
                    "y": float(ps.pose.position.y),
                    "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
                }
            )
        if pts:
            self._corrected_traj = pts

    def _on_wheel_path(self, msg) -> None:
        pts: List[Dict[str, float]] = []
        for ps in msg.poses:
            t = self._stamp_to_s(ps.header.stamp)
            q = ps.pose.orientation
            pts.append(
                {
                    "t": t,
                    "x": float(ps.pose.position.x),
                    "y": float(ps.pose.position.y),
                    "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
                }
            )
        if pts:
            self._wheel_traj = pts

    def _on_diagnostics(self, msg) -> None:
        for s in msg.status:
            values = []
            try:
                for kv in s.values:
                    values.append({"key": str(kv.key), "value": str(kv.value)})
                    self._diag_kv[str(kv.key)] = kv.value
            except Exception:  # noqa: BLE001
                values = []
            level = s.level
            if isinstance(level, (bytes, bytearray)):
                level = int.from_bytes(level, "little") if level else 0
            else:
                level = int(level)
            self._diagnostics.append(
                {
                    "name": s.name,
                    "level": level,
                    "message": s.message,
                    "hardware_id": s.hardware_id,
                    "values": values,
                }
            )

    def _flush(self) -> None:
        if self._flushed:
            return
        self._flushed = True
        try:
            # Drain background PNG worker before final artifacts / process exit.
            try:
                self._png_queue.join()
            except Exception:  # noqa: BLE001
                pass
            try:
                self._events_writer.close()
            except Exception:  # noqa: BLE001
                pass
            self._do_flush()
        except Exception as exc:  # noqa: BLE001
            try:
                self._node.get_logger().error(f"metrics flush failed: {exc}")
            except Exception:  # noqa: BLE001
                print(f"metrics flush failed: {exc}", file=sys.stderr)

    def _do_flush(self) -> None:
        from orb_slam_bringup.report import generate_report_html

        # Config hash
        config_paths: List[Path] = []
        if self._config_path:
            config_paths.append(Path(self._config_path))
        else:
            # Try package share / source tree
            try:
                from ament_index_python.packages import get_package_share_directory

                share = Path(get_package_share_directory("orb_slam_bringup"))
                p = share / "config" / "tasterobot_bag.yaml"
                if p.is_file():
                    config_paths.append(p)
            except Exception:  # noqa: BLE001
                pass
            src = Path(__file__).resolve().parents[1] / "config" / "tasterobot_bag.yaml"
            if src.is_file() and src not in config_paths:
                config_paths.append(src)

        cfg_sha = configuration_sha256(config_paths) if config_paths else ""
        repo = self._repo_dir or str(Path(__file__).resolve().parents[2])
        git = capture_git_info(repo)

        # Stereo section — fail closed: report observed pairs only.
        # Pair COUNT source of truth is the immutable bag (offline, deterministic).
        # Live CameraInfo is kept only for camera_validated (geometry check).
        for p in config_paths:
            try:
                import yaml

                prof = yaml.safe_load(p.read_text(encoding="utf-8"))
                bag = (prof or {}).get("bag", {}) or {}
                if not self._bag_duration_s:
                    self._bag_duration_s = float(bag.get("duration_s", 0.0))
                if not self._bag_path:
                    self._bag_path = str(bag.get("path", ""))
                break
            except Exception:  # noqa: BLE001
                pass
        # Prefer live-validated geometry; fall back to expected profile values.
        width = self._live_width if self._camera_validated else self._expected_width
        height = self._live_height if self._camera_validated else self._expected_height
        baseline_m = (
            self._live_baseline_m
            if self._camera_validated
            else self._expected_baseline_m
        )
        if self._camera_validated:
            width = self._live_width
            height = self._live_height
            baseline_m = self._live_baseline_m

        # Bag-offline pair count replaces lossy live subscription as gate input.
        paired_count = self._stereo_pairs.paired_count
        stereo_measured = self._stereo_pairs.streams_seen or self._camera_validated
        bag_for_stereo = Path(self._bag_path) if self._bag_path else None
        if bag_for_stereo is not None and bag_for_stereo.exists():
            try:
                bag_paired, bag_left, bag_right = count_stereo_pairs_from_bag(
                    bag_for_stereo,
                    self._left_info_topic,
                    self._right_info_topic,
                    tolerance_s=STEREO_PAIR_TOLERANCE_S,
                    expected=self._expected_pairs,
                )
                paired_count = bag_paired
                # Bag read succeeded → pair count is measured (even if 0 pairs).
                stereo_measured = True
                try:
                    self._node.get_logger().info(
                        f"stereo pairs from bag: {bag_paired}/"
                        f"{self._expected_pairs} "
                        f"(left={bag_left}, right={bag_right})"
                    )
                except Exception:  # noqa: BLE001
                    pass
            except Exception as exc:  # noqa: BLE001
                # Bag unreadable → keep live count / fail-closed (do not invent).
                try:
                    self._node.get_logger().warning(
                        f"bag stereo count failed, falling back to live: {exc}"
                    )
                except Exception:  # noqa: BLE001
                    pass

        stereo = build_live_stereo_section(
            expected_pairs=self._expected_pairs,
            paired_count=paired_count,
            width=width,
            height=height,
            baseline_m=baseline_m,
            camera_validated=self._camera_validated,
            measured=stereo_measured,
        )

        # Final map export
        pgm_match = True
        if self._last_map is not None:
            stem = self._artifact_dir / "final-map"
            write_nav2_map(self._last_map, stem)
            # Verify reload matches
            try:
                pgm_match = _verify_nav2_map(self._last_map, stem)
            except Exception:  # noqa: BLE001
                pgm_match = False

        trajectories = {
            "orb": list(self._orb_traj),
            "wheel": list(self._wheel_traj),
            "corrected": list(self._corrected_traj),
        }
        write_trajectory_csv(
            self._artifact_dir / "orb_trajectory.csv", trajectories["orb"]
        )
        write_trajectory_csv(
            self._artifact_dir / "wheel_trajectory.csv", trajectories["wheel"]
        )
        write_trajectory_csv(
            self._artifact_dir / "corrected_trajectory.csv", trajectories["corrected"]
        )

        # Fallback: commit invariants measured-by-design when diagnostics seen;
        # raw rejection counters kept for observability (not gate inputs).
        fb = fallback_from_diagnostics(self._diag_kv)
        agg = MetricsAggregator(
            bag_duration_s=self._bag_duration_s,
            frames=self._frames,
            events=self._events,
            revisions=self._revisions,
            final_map=self._last_map,
            unresolved_scan_count=fb["unresolved_scan_count"],
            invalid_tf_committed=fb["invalid_tf_committed"],
            wheel_only_before_recovery=fb["wheel_only_before_recovery"],
            planarity_rejections=fb["planarity_rejections"],
            tf_lookup_failures=fb.get("tf_lookup_failures"),
            wheel_interp_failures=fb.get("wheel_interp_failures"),
            bag={"path": self._bag_path, "duration_s": self._bag_duration_s},
            git=git,
            configuration_sha256=cfg_sha,
            stereo=stereo,
            diagnostics=self._diagnostics[-100:],
            trajectories=trajectories,
            invalid_poses=self._invalid_poses,
            pgm_yaml_match=pgm_match,
        )
        metrics = agg.compute()
        atomic_write_json(self._artifact_dir / "metrics.json", metrics)

        html = generate_report_html(metrics, artifact_dir=self._artifact_dir)
        atomic_write_text(self._artifact_dir / "report.html", html)

        try:
            self._node.get_logger().info(
                f"metrics flushed to {self._artifact_dir}"
            )
        except Exception:  # noqa: BLE001
            pass


def _verify_nav2_map(grid: MapGridSample, stem: Path) -> bool:
    import yaml

    yaml_path = Path(str(stem) + ".yaml")
    pgm_path = Path(str(stem) + ".pgm")
    if not yaml_path.is_file() or not pgm_path.is_file():
        return False
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if abs(float(meta["resolution"]) - grid.resolution) > 1e-9:
        return False
    origin = meta["origin"]
    if abs(origin[0] - grid.origin_x) > 1e-9 or abs(origin[1] - grid.origin_y) > 1e-9:
        return False
    # Include origin yaw so yaw drift fails pgm_yaml_match (fail-closed).
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
    if abs(origin_yaw - grid.origin_yaw) > 1e-6:
        return False
    raw = pgm_path.read_bytes()
    if not raw.startswith(b"P5"):
        return False
    # Parse dimensions
    idx = raw.index(b"\n") + 1
    while raw[idx : idx + 1] == b"#":
        idx = raw.index(b"\n", idx) + 1
    end = raw.index(b"\n", idx)
    wh = raw[idx:end].decode("ascii").split()
    width, height = int(wh[0]), int(wh[1])
    if width != grid.width or height != grid.height:
        return False
    return True


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    recorder = MetricsRecorderNode()
    node = recorder.node
    # Multi-threaded so map/PNG-related work cannot starve tracked_frame.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        recorder._flush()
        try:
            executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
