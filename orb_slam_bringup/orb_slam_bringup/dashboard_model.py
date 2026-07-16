"""Thread-safe, ROS-free state model used by the dashboard adapters."""

from __future__ import annotations

import math
import threading
from array import array
from collections import OrderedDict
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral
from typing import Any


MAX_UNMATCHED = 16
MAX_CORRECTED_POINTS = 1500
MAX_PROVISIONAL_POINTS = 1500
MAX_WHEEL_POINTS = 1500
STALE_AFTER_S = 2.0
REVISION_STATE_PRIORITY = {"IDLE": 0, "BUILDING": 1, "PUBLISHED": 2, "FAILED": 3}
assert MAX_UNMATCHED == 16
assert MAX_CORRECTED_POINTS > 0 and MAX_PROVISIONAL_POINTS > 0 and MAX_WHEEL_POINTS > 0


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
    return {"x": float(pose.x), "y": float(pose.y), "yaw": float(pose.yaw)}


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
    if len(value.cells) != expected:
        raise ValueError("map cell count does not match dimensions")

    compact = array("b")
    for index in range(expected):
        cell = value.cells[index]
        if (not isinstance(cell, Integral) or isinstance(cell, bool)
                or not -1 <= int(cell) <= 100):
            raise ValueError("map cells must be integers in [-1, 100]")
        compact.append(int(cell))
    # A read-only signed-byte view is compact, copied, and preserves -1 unknown cells.
    compact_cells = memoryview(bytes(compact)).cast("b")
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
        self._fallback_trail: list[Pose2D] = []
        self._provisional: list[tuple[float, float]] = []
        self._loss_orb_anchor: Pose2D | None = None
        self._loss_wheel_anchor: Pose2D | None = None
        self._fallback_pose: Pose2D | None = None
        self._recovery_graph_revision: int | None = None
        self._loss_event_seen = False
        self._relocalized_event_seen = False

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
            self._corrected = bounded_points(list(points), MAX_CORRECTED_POINTS)
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

    def update_wheel(self, pose: Pose2D, stamp_s: float) -> None:
        _finite_pose(pose)
        _finite(stamp_s, "stamp_s")
        with self._lock:
            self._wheel_pose = pose
            self._wheel_stamp = float(stamp_s)
            if self._loss_event_seen and self._loss_orb_anchor is not None and self._loss_wheel_anchor is not None:
                self._fallback_pose = compose(self._loss_orb_anchor,
                                               compose(inverse(self._loss_wheel_anchor), pose))
                self._fallback_trail.append(self._fallback_pose)
                self._fallback_trail = bounded_points(self._fallback_trail, MAX_WHEEL_POINTS)

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
            self._fallback_trail = ([self._tracked_pose] if self._tracked_pose is not None else [])

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
        self._fallback_trail = []
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
            fallback = {"active": self._fallback_pose is not None,
                        "pose": _pose_json(self._fallback_pose),
                        "trail": [_pose_json(p) for p in self._fallback_trail]}
            return {
                "state": self._state,
                "tracking": {"state": self._state, "pose": _pose_json(self._tracked_pose),
                              "keypoints": self._tracked_keypoints, "stamp_s": self._tracked_stamp},
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
                "paths": {"corrected": [_pose_json(p) for p in self._corrected],
                          "wheel": [_pose_json(p) for p in self._fallback_trail],
                          "provisional": [[x, y] for x, y in self._provisional]},
                "fallback": fallback,
            }

    def map_png(self) -> bytes:
        with self._lock:
            return bytes(self._map_png_value)
