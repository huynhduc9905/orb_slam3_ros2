"""Thread-safe, ROS-free state model used by the dashboard adapters."""

from __future__ import annotations

import math
import threading
from collections import OrderedDict, deque
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral
from typing import Any

import numpy as np


MAX_UNMATCHED = 16
# Full-resolution trajectory history retained internally so the ENTIRE path
# (start -> current) stays available for display. Points are kept at their
# native density and are NEVER destructively re-strided in place, so the
# rendered path cannot progressively facet into a polygon as it grows. Oldest
# points are dropped only past this generous safety cap, which bounds memory on
# pathologically long live sessions; a full bag replay (~13k odom samples) stays
# well within it, so its whole path is retained.
MAX_TRAIL_HISTORY = 20000
MAX_PROVISIONAL_POINTS = 1500
MAX_KEYFRAMES = 4000
MAX_LOOP_EDGES = 4000
# Uniform decimation applied ONCE, at snapshot/display time, across the FULL
# retained path. Because decimation happens once over the complete path (not
# iteratively on the stored buffer), spacing is uniform end to end, so the whole
# trajectory renders smoothly (no progressive faceting into a polygon) at this
# point count. Kept moderate so the /state payload stays small enough to poll at
# ~15 Hz over bandwidth-limited (e.g. relayed Tailscale) links without the
# client falling behind.
DISPLAY_TRAIL_POINTS = 1000
# Graph node cloud + loop segments are decimated for the payload too: the loop
# graph panel cannot resolve thousands of distinct nodes, and serializing all of
# them (up to MAX_KEYFRAMES) dominated the /state payload (~84%) and json.dumps
# cost. Loop segments are resolved against the FULL keyframe set first, then the
# node cloud is decimated for display.
DISPLAY_GRAPH_NODES = 600
DISPLAY_LOOP_EDGES = 500
STALE_AFTER_S = 2.0
REVISION_STATE_PRIORITY = {"IDLE": 0, "BUILDING": 1, "PUBLISHED": 2, "FAILED": 3}
assert MAX_UNMATCHED == 16
assert MAX_TRAIL_HISTORY > 0 and MAX_PROVISIONAL_POINTS > 0 and DISPLAY_TRAIL_POINTS > 0


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MapEnvelope:
    stamp: tuple[int, int]
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    cells: Sequence[int]


@dataclass(frozen=True)
class RevisionEnvelope:
    stamp: tuple[int, int]
    state: str
    graph_revision: int
    map_revision: int
    committed_scan_count: int


def normalize_yaw(yaw: float) -> float:
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
                  normalize_yaw(a.yaw + b.yaw))


def inverse(a: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(-c * a.x - s * a.y, s * a.x - c * a.y, normalize_yaw(-a.yaw))


def bounded_points(points: Sequence[Any], limit: int) -> list[Any]:
    if limit <= 0 or len(points) <= limit:
        return list(points)
    if limit == 1:
        return [points[-1]]
    step = math.ceil((len(points) - 1) / (limit - 1))
    result = list(points[::step])
    if result[-1] != points[-1]:
        result.append(points[-1])
    return result[: limit - 1] + [points[-1]] if len(result) > limit else result


def _finite_pose(pose: Pose2D | None) -> None:
    if pose is not None and not all(math.isfinite(float(v)) for v in (pose.x, pose.y, pose.yaw)):
        raise ValueError("pose must contain only finite values")


def _finite(value: float, name: str) -> None:
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")


def _pose_json(pose: Pose2D | None) -> dict[str, float] | None:
    if pose is None:
        return None
    # Round to ~0.1 mm / ~0.1 mrad. Visually lossless for the map viewer but
    # roughly halves the /state payload: full-precision floats (~17 digits)
    # otherwise dominate JSON size, which matters when polling at ~15 Hz over
    # bandwidth-limited links.
    return {"x": round(float(pose.x), 4), "y": round(float(pose.y), 4),
            "yaw": round(float(pose.yaw), 4)}


def _copy_map(value: MapEnvelope) -> MapEnvelope:
    resolution = float(value.resolution)
    origin_x = float(value.origin_x)
    origin_y = float(value.origin_y)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("map resolution must be finite and positive")
    if not math.isfinite(origin_x) or not math.isfinite(origin_y):
        raise ValueError("map origin must be finite")
    if (not isinstance(value.width, Integral) or isinstance(value.width, bool) or
            not isinstance(value.height, Integral) or isinstance(value.height, bool)):
        raise ValueError("map dimensions must be integers")
    width, height = int(value.width), int(value.height)
    if width <= 0 or height <= 0:
        raise ValueError("map dimensions must be positive")
    expected = width * height

    # Vectorized validation + compaction. The previous per-cell Python loop cost
    # ~73 ms on a 448x448 map and ran on every incoming OccupancyGrid, pinning a
    # core; numpy validates all cells in one pass (<1 ms). `len()` alone is not
    # sufficient here: nested input can have the expected outer length but a
    # different number of actual cells after compact serialization.
    try:
        arr = np.asarray(value.cells)
    except (TypeError, ValueError) as exc:
        raise ValueError("map cells must be a one-dimensional array matching dimensions") from exc
    if arr.ndim != 1 or arr.size != expected:
        raise ValueError("map cells must be a one-dimensional array matching dimensions")
    if arr.dtype == np.bool_ or not np.issubdtype(arr.dtype, np.integer):
        raise ValueError("map cells must be integers in [-1, 100]")
    if arr.size and (int(arr.min()) < -1 or int(arr.max()) > 100):
        raise ValueError("map cells must be integers in [-1, 100]")
    # A read-only signed-byte view is compact, copied (tobytes() never aliases the
    # possibly-mutable source), and preserves -1 unknown cells.
    compact_cells = memoryview(arr.astype(np.int8).tobytes()).cast("b")
    return MapEnvelope(tuple(value.stamp), resolution, origin_x, origin_y,
                       width, height, compact_cells)


def _revision_order_key(value: RevisionEnvelope) -> tuple[Any, ...]:
    return (value.map_revision, REVISION_STATE_PRIORITY.get(value.state, -1),
            value.graph_revision, value.state,
            value.committed_scan_count, value.stamp)


class DashboardModel:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._maps: OrderedDict[tuple[int, int], MapEnvelope] = OrderedDict()
        self._revisions: OrderedDict[tuple[int, int], RevisionEnvelope] = OrderedDict()
        self._pending: tuple[MapEnvelope, RevisionEnvelope] | None = None
        self._visible_revision = 0
        self._visible_map: MapEnvelope | None = None
        self._visible_revision_envelope: RevisionEnvelope | None = None
        self._map_png_value = b""
        self._latest_revision: RevisionEnvelope | None = None
        self._state = "NO_IMAGES_YET"
        self._tracking_state = "NO_IMAGES_YET"
        self._tracked_pose: Pose2D | None = None
        self._tracked_keypoints = 0
        self._tracked_stamp: float | None = None
        self._wheel_pose: Pose2D | None = None
        self._wheel_stamp: float | None = None
        self._corrected: list[Pose2D] = []
        self._corrected_revision = 0
        self._fallback_trail: deque[Pose2D] = deque(maxlen=MAX_TRAIL_HISTORY)
        self._provisional: list[tuple[float, float]] = []
        self._loss_orb_anchor: Pose2D | None = None
        self._loss_wheel_anchor: Pose2D | None = None
        self._fallback_pose: Pose2D | None = None
        self._recovery_graph_revision: int | None = None
        self._loss_event_seen = False
        self._relocalized_event_seen = False
        # Separate real-time trajectory windows (independent of loss-recovery).
        self._orb_trail: deque[Pose2D] = deque(maxlen=MAX_TRAIL_HISTORY)
        self._odom_trail: deque[Pose2D] = deque(maxlen=MAX_TRAIL_HISTORY)
        # Loop-closure connectivity: keyframe node positions + loop edge pairs.
        self._graph_revision = 0
        self._graph_active_connected = False
        self._keyframes: list[tuple[int, float, float]] = []
        self._loop_edges: list[tuple[int, int]] = []

    def ingest_map(self, value: MapEnvelope) -> None:
        copied = _copy_map(value)
        with self._lock:
            if self._pending is not None and self._pending[0].stamp == copied.stamp:
                self._pending = None
            self._maps[copied.stamp] = copied
            self._maps.move_to_end(copied.stamp)
            while len(self._maps) > MAX_UNMATCHED:
                self._maps.popitem(last=False)
            self._finish_recovery_if_coherent_locked(copied.stamp)

    def ingest_revision(self, value: RevisionEnvelope) -> None:
        with self._lock:
            copied = RevisionEnvelope(tuple(value.stamp), str(value.state), int(value.graph_revision),
                                      int(value.map_revision), int(value.committed_scan_count))
            if self._pending is not None and self._pending[1].stamp == copied.stamp:
                self._pending = None
            # Retain the highest-priority revision per stamp. The mapper emits
            # PUBLISHED and then a trailing IDLE for the SAME stamp (the
            # rebuilder's quiescent-idle emit). An unconditional overwrite let
            # IDLE clobber PUBLISHED, so next_map_to_render (which requires
            # state == PUBLISHED) never found a render candidate and the map
            # never rendered — most visible in rebuild-only mode where map
            # publications are sparse. Order by _revision_order_key so IDLE(0)
            # cannot displace PUBLISHED(2) but FAILED(3)/newer revisions still do.
            existing = self._revisions.get(copied.stamp)
            if existing is None or _revision_order_key(copied) >= _revision_order_key(existing):
                self._revisions[copied.stamp] = copied
                self._revisions.move_to_end(copied.stamp)
            if (self._latest_revision is None or
                    _revision_order_key(copied) > _revision_order_key(self._latest_revision)):
                self._latest_revision = copied
            while len(self._revisions) > MAX_UNMATCHED:
                self._revisions.popitem(last=False)
            self._finish_recovery_if_coherent_locked(copied.stamp)

    def next_map_to_render(self) -> tuple[MapEnvelope, RevisionEnvelope] | None:
        with self._lock:
            candidates = [(self._maps[s], rev) for s, rev in self._revisions.items()
                          if s in self._maps and rev.state == "PUBLISHED"
                          and rev.map_revision > self._visible_revision]
            if not candidates:
                return None
            chosen = max(candidates, key=lambda pair: (pair[1].map_revision, pair[1].stamp))
            self._pending = chosen
            return _copy_map(chosen[0]), chosen[1]

    def publish_rendered_map(self, stamp: tuple[int, int], png: bytes) -> bool:
        with self._lock:
            if self._pending is None or self._pending[0].stamp != tuple(stamp):
                return False
            map_value, revision = self._pending
            current_map = self._maps.get(map_value.stamp)
            current_revision = self._revisions.get(revision.stamp)
            if (current_map is not map_value or current_revision is not revision or
                    revision.state != "PUBLISHED" or
                    revision.map_revision <= self._visible_revision or not isinstance(png, bytes)):
                self._pending = None
                return False
            self._visible_revision = revision.map_revision
            self._visible_map = _copy_map(map_value)
            self._visible_revision_envelope = RevisionEnvelope(
                revision.stamp, revision.state, revision.graph_revision,
                revision.map_revision, revision.committed_scan_count)
            self._map_png_value = bytes(png)
            self._pending = None
            return True

    def replace_corrected_path(self, graph_revision: int, points: list[Pose2D]) -> None:
        for point in points:
            _finite_pose(point)
        with self._lock:
            # Keep the corrected path at full resolution; it is decimated once,
            # uniformly, at display time so the whole corrected trajectory stays
            # smooth. Cap only past the generous safety history bound.
            self._corrected = list(points)[-MAX_TRAIL_HISTORY:]
            self._corrected_revision = int(graph_revision)

    def update_tracked(self, state: str, pose: Pose2D | None, keypoints: int, stamp_s: float) -> None:
        _finite_pose(pose)
        _finite(stamp_s, "stamp_s")
        with self._lock:
            self._tracking_state = str(state)
            if not self._loss_event_seen:
                self._state = self._tracking_state
            self._tracked_pose = pose
            self._tracked_keypoints = int(keypoints)
            self._tracked_stamp = float(stamp_s)
            if pose is not None:
                # deque(maxlen=...) drops only the oldest point past the safety
                # cap; survivors keep native spacing (no in-place re-striding).
                self._orb_trail.append(pose)

    def update_wheel(self, pose: Pose2D, stamp_s: float) -> None:
        _finite_pose(pose)
        _finite(stamp_s, "stamp_s")
        with self._lock:
            self._wheel_pose = pose
            self._wheel_stamp = float(stamp_s)
            # Always accumulate the raw odometry trajectory for the odom window.
            # deque(maxlen=...) keeps native spacing; no in-place re-striding.
            self._odom_trail.append(pose)
            if self._loss_event_seen and self._loss_orb_anchor is not None and self._loss_wheel_anchor is not None:
                self._fallback_pose = compose(self._loss_orb_anchor,
                                               compose(inverse(self._loss_wheel_anchor), pose))
                self._fallback_trail.append(self._fallback_pose)

    def update_graph(self, graph_revision: int, active_connected: bool,
                     keyframes: list[tuple[int, float, float]],
                     loop_edges: list[tuple[int, int]]) -> None:
        for _id, x, y in keyframes:
            _finite(x, "keyframe.x")
            _finite(y, "keyframe.y")
        with self._lock:
            self._graph_revision = int(graph_revision)
            self._graph_active_connected = bool(active_connected)
            self._keyframes = [(int(i), float(x), float(y))
                               for i, x, y in keyframes[:MAX_KEYFRAMES]]
            self._loop_edges = [(int(a), int(b)) for a, b in loop_edges[:MAX_LOOP_EDGES]]

    def start_loss(self, graph_revision: int, stamp_s: float) -> None:
        _finite(stamp_s, "stamp_s")
        with self._lock:
            if self._loss_event_seen:
                return
            self._loss_event_seen = True
            self._state = "LOST"
            self._loss_orb_anchor = self._tracked_pose
            self._loss_wheel_anchor = self._wheel_pose
            self._fallback_pose = self._tracked_pose
            self._fallback_trail = deque(
                [self._tracked_pose] if self._tracked_pose is not None else [],
                maxlen=MAX_TRAIL_HISTORY,
            )

    def mark_relocalized(self, graph_revision: int, stamp_s: float) -> None:
        _finite(stamp_s, "stamp_s")
        with self._lock:
            if not self._loss_event_seen or self._relocalized_event_seen:
                return
            self._relocalized_event_seen = True
            self._state = "recovery_pending"
            self._recovery_graph_revision = int(graph_revision)
            for stamp in tuple(self._revisions):
                self._finish_recovery_if_coherent_locked(stamp)
                if self._recovery_graph_revision is None:
                    break

    def _finish_recovery_if_coherent_locked(self, stamp: tuple[int, int]) -> None:
        revision = self._revisions.get(stamp)
        if (self._recovery_graph_revision is None or stamp not in self._maps or
                revision is None or revision.state != "PUBLISHED" or
                revision.graph_revision < self._recovery_graph_revision):
            return
        self._fallback_pose = None
        self._fallback_trail = deque(maxlen=MAX_TRAIL_HISTORY)
        self._loss_orb_anchor = None
        self._loss_wheel_anchor = None
        self._recovery_graph_revision = None
        self._loss_event_seen = False
        self._relocalized_event_seen = False
        self._state = "OK"

    def replace_provisional_scan(self, points: list[tuple[float, float]]) -> None:
        cleaned = []
        for x, y in points:
            _finite(x, "point.x")
            _finite(y, "point.y")
            cleaned.append((float(x), float(y)))
        with self._lock:
            self._provisional = bounded_points(cleaned, MAX_PROVISIONAL_POINTS)

    def snapshot(self, now_s: float) -> dict:
        _finite(now_s, "now_s")
        with self._lock:
            tracking_stale = self._tracked_stamp is None or now_s - self._tracked_stamp > STALE_AFTER_S
            wheel_stale = self._wheel_stamp is None or now_s - self._wheel_stamp > STALE_AFTER_S
            latest = self._latest_revision
            visible_map = self._visible_map
            visible_revision = self._visible_revision_envelope
            # Decimate uniformly ONCE, over the full retained path, at display
            # time. Because the stored trails were never re-strided in place,
            # spacing here is uniform end to end (no dense-recent/sparse-old
            # gradient), so the rendered path stays smooth over its whole length.
            fallback_trail = bounded_points(list(self._fallback_trail), DISPLAY_TRAIL_POINTS)
            fallback = {"active": self._fallback_pose is not None,
                        "pose": _pose_json(self._fallback_pose),
                        "trail": [_pose_json(p) for p in fallback_trail]}
            kf_pos = {i: (x, y) for i, x, y in self._keyframes}
            loop_segments = []
            for a, b in self._loop_edges:
                if a in kf_pos and b in kf_pos:
                    ax, ay = kf_pos[a]; bx, by = kf_pos[b]
                    loop_segments.append({"from": {"x": ax, "y": ay},
                                          "to": {"x": bx, "y": by}})
                    if len(loop_segments) >= DISPLAY_LOOP_EDGES:
                        break
            display_keyframes = bounded_points(self._keyframes, DISPLAY_GRAPH_NODES)
            return {
                "state": self._state,
                "tracking": {"state": self._state, "pose": _pose_json(self._tracked_pose),
                              "keypoints": self._tracked_keypoints, "stamp_s": self._tracked_stamp},
                "odom": {"pose": _pose_json(self._wheel_pose),
                         "trail": [_pose_json(p) for p in bounded_points(list(self._odom_trail), DISPLAY_TRAIL_POINTS)]},
                "orb": {"pose": _pose_json(self._tracked_pose),
                        "trail": [_pose_json(p) for p in bounded_points(list(self._orb_trail), DISPLAY_TRAIL_POINTS)]},
                "graph": {"revision": self._graph_revision,
                          "active_connected": self._graph_active_connected,
                          "keyframes": [{"id": i, "x": x, "y": y} for i, x, y in display_keyframes],
                          "loops": loop_segments},
                "health": {"tracking_stale": tracking_stale, "wheel_stale": wheel_stale,
                            "map_stale": latest is None},
                "map": ({"resolution": visible_map.resolution, "origin_x": visible_map.origin_x,
                         "origin_y": visible_map.origin_y, "width": visible_map.width,
                         "height": visible_map.height, "revision": self._visible_revision,
                         "state": visible_revision.state,
                         "graph_revision": visible_revision.graph_revision,
                         "committed_scan_count": visible_revision.committed_scan_count}
                        if visible_map is not None and visible_revision is not None
                        else {"revision": self._visible_revision}),
                "map_revision": ({"state": latest.state, "graph_revision": latest.graph_revision,
                                  "map_revision": latest.map_revision,
                                  "committed_scan_count": latest.committed_scan_count}
                                 if latest is not None else None),
                "paths": {"corrected": [_pose_json(p) for p in bounded_points(self._corrected, DISPLAY_TRAIL_POINTS)],
                          "wheel": [_pose_json(p) for p in fallback_trail],
                          "provisional": [[x, y] for x, y in self._provisional]},
                "fallback": fallback,
            }

    def map_png(self) -> bytes:
        with self._lock:
            return bytes(self._map_png_value)
