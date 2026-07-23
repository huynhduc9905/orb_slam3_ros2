"""Read-only ROS-to-HTTP adapter for the direct dashboard."""

from __future__ import annotations

import io
import json
import math
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FsPath
from typing import Deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from visualization_msgs.msg import Marker

from orb_slam3_msgs.msg import (GraphSnapshot, MapRevision, RevisionedPath,
                                TrackedFrame, TrackingEvent)
from orb_slam_bringup.dashboard_model import (DashboardModel, MapEnvelope,
                                               Pose2D, RevisionEnvelope,
                                               bounded_points)

PNG_OCCUPIED = 30
PNG_FREE = 220
PNG_UNKNOWN = 128
MAX_EVENTS = 200

TRACKING_STATE_NAMES = {0: "NO_IMAGES_YET", 1: "NOT_INITIALIZED", 2: "OK",
                        3: "RECENTLY_LOST", 4: "LOST"}
EVENT_TYPE_NAMES = {0: "INITIALIZED", 1: "LOST", 2: "RELOCALIZED",
                    3: "LOOP_CLOSED", 4: "MAP_CREATED", 5: "MAP_MERGED",
                    6: "MAP_RESET"}
REVISION_STATE_NAMES = {0: "IDLE", 1: "BUILDING", 2: "PUBLISHED", 3: "FAILED"}


class RateTracker:
    def __init__(self, window_s: float = 3.0) -> None:
        if window_s <= 0.0:
            raise ValueError("window_s must be positive")
        self._window_s = float(window_s)
        self._stamps: Deque[float] = deque()

    def record(self, now_s: float) -> None:
        self._stamps.append(float(now_s))
        self._prune(now_s)

    def hz(self, now_s: float) -> float:
        self._prune(now_s)
        return len(self._stamps) / self._window_s

    def reset(self) -> None:
        self._stamps.clear()

    def _prune(self, now_s: float) -> None:
        while self._stamps and self._stamps[0] < now_s - self._window_s:
            self._stamps.popleft()


class SourceRateTracker:
    """Estimates the true source frequency from message header stamps rather
    than receipt wall-time. Immune to receive jitter/bursts and to a saturated
    dashboard: the header stamps carry the backend's own cadence. Uses the
    median of consecutive inter-stamp deltas over a bounded window, which is
    also robust to occasional dropped (best-effort) samples."""

    def __init__(self, max_samples: int = 120) -> None:
        self._max = max(3, int(max_samples))
        self._stamps: Deque[float] = deque(maxlen=self._max)

    def record(self, source_stamp_s: float) -> None:
        if not math.isfinite(source_stamp_s):
            return
        # Ignore duplicate/non-monotonic stamps so deltas stay meaningful.
        if self._stamps and source_stamp_s <= self._stamps[-1]:
            return
        self._stamps.append(float(source_stamp_s))

    def reset(self) -> None:
        self._stamps.clear()

    def hz(self, now_s: float | None = None, stale_after_s: float = 2.0) -> float:
        if len(self._stamps) < 2:
            return 0.0
        # If the newest sample is older than stale_after_s of source time, the
        # stream has stopped — report 0 rather than a phantom rate.
        if now_s is not None and math.isfinite(now_s) and now_s - self._stamps[-1] > stale_after_s:
            return 0.0
        deltas = sorted(self._stamps[i] - self._stamps[i - 1]
                        for i in range(1, len(self._stamps)))
        mid = len(deltas) // 2
        median = deltas[mid] if len(deltas) % 2 else 0.5 * (deltas[mid - 1] + deltas[mid])
        return 1.0 / median if median > 0.0 else 0.0


def stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose2d(pose) -> Pose2D:
    q = pose.orientation
    return Pose2D(float(pose.position.x), float(pose.position.y),
                  yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)))


def occupancy_to_gray(data, width: int, height: int) -> np.ndarray:
    if width <= 0 or height <= 0:
        return np.zeros((0, 0), dtype=np.uint8)
    arr = np.asarray(data, dtype=np.int16).reshape((height, width))
    gray = np.full((height, width), PNG_UNKNOWN, dtype=np.uint8)
    gray[arr >= 50] = PNG_OCCUPIED
    gray[(arr >= 0) & (arr < 50)] = PNG_FREE
    return np.flipud(gray)


def decimate_path(points, max_points):
    if max_points <= 0 or len(points) <= max_points:
        return list(points)
    step = math.ceil(len(points) / max_points)
    result = list(points[::step])
    if result[-1] != points[-1]:
        result.append(points[-1])
    return result


class DashboardServer(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_server")
        for name, value in {
            "host": "0.0.0.0", "port": 51871, "map_rate_hz": 1.0,
            "rate_window_s": 3.0, "max_path_points": 1500, "web_dir": "",
            "map_frame": "orb_map",
            "map_topic": "/orb_lidar/map", "map_revision_topic": "/orb_lidar/map_revision",
            "corrected_path_topic": "/orb_lidar/corrected_path_revisioned",
            "provisional_scan_topic": "/orb_lidar/provisional_scan",
            "odom_topic": "/odom_wheel", "tracked_frame_topic": "/orb_slam3/tracked_frame",
            "events_topic": "/orb_slam3/events",
            "graph_snapshot_topic": "/orb_slam3/graph_snapshot"}.items():
            self.declare_parameter(name, value)
        gp = self.get_parameter
        self._host, self._port = gp("host").value, int(gp("port").value)
        self._max_path_points = int(gp("max_path_points").value)
        self._map_frame = str(gp("map_frame").value)
        self._web_dir = FsPath(self._resolve_web_dir(gp("web_dir").value)).resolve()
        self._model = DashboardModel()
        self._events: Deque[dict] = deque(maxlen=MAX_EVENTS)
        # Backend-cadence rate from message header stamps (not receipt time), so
        # a busy dashboard never mis-reports the true tracking/odom frequency.
        self._tracking_rate = SourceRateTracker()
        self._odom_rate = SourceRateTracker()
        # Split heavy map/PNG work from lightweight state so a MultiThreadedExecutor
        # keeps draining pose/odom/graph while a map render is in flight.
        self._map_cbg = MutuallyExclusiveCallbackGroup()
        self._state_cbg = MutuallyExclusiveCallbackGroup()
        # Read-only viewer: BEST_EFFORT so the dashboard can never backpressure the
        # mapper/wrapper reliable writers (an external reliable reader was measured
        # to degrade tracking). Events stay reliable — they are low-rate and drive
        # loss/recovery state that must not be silently dropped.
        # Map grid + revision: BEST_EFFORT with generous depth. RELIABLE +
        # TRANSIENT_LOCAL was attempted but the large ~200 KB OccupancyGrid was
        # silently undelivered by FastDDS when the Python process couldn't
        # service the reliable handshake fast enough during high-rate callback
        # bursts. BEST_EFFORT depth=5 catches rebuild publications reliably in
        # practice since the mapper publishes on every graph snapshot (~1 Hz).
        best_effort1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        best_effort10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST)
        reliable100 = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST)
        map_sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, gp("map_topic").value, self._on_map,
                                 map_sub_qos, callback_group=self._state_cbg)
        self.create_subscription(MapRevision, gp("map_revision_topic").value, self._on_map_revision,
                                 map_sub_qos, callback_group=self._state_cbg)
        self.create_subscription(RevisionedPath, gp("corrected_path_topic").value, self._on_corrected_path,
                                 best_effort1, callback_group=self._state_cbg)
        self.create_subscription(Marker, gp("provisional_scan_topic").value, self._on_provisional_scan,
                                 best_effort1, callback_group=self._state_cbg)
        self.create_subscription(GraphSnapshot, gp("graph_snapshot_topic").value, self._on_graph,
                                 best_effort1, callback_group=self._state_cbg)
        self.create_subscription(Odometry, gp("odom_topic").value, self._on_odom,
                                 qos_profile_sensor_data, callback_group=self._state_cbg)
        self.create_subscription(TrackedFrame, gp("tracked_frame_topic").value, self._on_tracked_frame,
                                 qos_profile_sensor_data, callback_group=self._state_cbg)
        self.create_subscription(TrackingEvent, gp("events_topic").value, self._on_event,
                                 reliable100, callback_group=self._state_cbg)
        map_rate = float(gp("map_rate_hz").value)
        if map_rate > 0:
            self.create_timer(1.0 / map_rate, self._render_map, callback_group=self._map_cbg)
        self._httpd = ThreadingHTTPServer((self._host, self._port), self._make_handler())
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

    @staticmethod
    def _resolve_web_dir(param_value: str) -> str:
        if param_value:
            return param_value
        try:
            from ament_index_python.packages import get_package_share_directory
            return str(FsPath(get_package_share_directory("orb_slam_bringup")) / "web")
        except Exception:
            return str(FsPath(__file__).resolve().parents[1] / "web")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_map(self, msg: OccupancyGrid) -> None:
        info, origin = msg.info, msg.info.origin.position
        self._model.ingest_map(MapEnvelope(stamp_key(msg.header.stamp), float(info.resolution),
            float(origin.x), float(origin.y), int(info.width), int(info.height), msg.data))

    def _on_map_revision(self, msg: MapRevision) -> None:
        self._model.ingest_revision(RevisionEnvelope(stamp_key(msg.header.stamp),
            REVISION_STATE_NAMES.get(int(msg.state), "FAILED"), int(msg.graph_revision),
            int(msg.map_revision), int(msg.committed_scan_count)))

    def _on_corrected_path(self, msg: RevisionedPath) -> None:
        self._model.replace_corrected_path(int(msg.graph_revision),
                                           [pose2d(item.pose) for item in msg.poses])

    def _on_provisional_scan(self, msg: Marker) -> None:
        if msg.header.frame_id != self._map_frame:
            return
        action = int(msg.action)
        if action in (Marker.DELETE, Marker.DELETEALL):
            self._model.replace_provisional_scan([])
            return
        if action not in (Marker.ADD, Marker.MODIFY) or int(msg.type) != Marker.POINTS:
            return
        points = [(float(p.x), float(p.y)) for p in msg.points]
        self._model.replace_provisional_scan(bounded_points(points, self._max_path_points))

    def _on_odom(self, msg: Odometry) -> None:
        now = self._now_s()
        self._odom_rate.record(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self._model.update_wheel(pose2d(msg.pose.pose), now)

    def _on_tracked_frame(self, msg: TrackedFrame) -> None:
        now = self._now_s()
        state = TRACKING_STATE_NAMES.get(int(msg.tracking_state), "NO_IMAGES_YET")
        self._model.update_tracked(state, pose2d(msg.pose) if msg.pose_valid else None,
                                   int(msg.tracked_keypoints), now)
        self._tracking_rate.record(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_graph(self, msg: GraphSnapshot) -> None:
        keyframes = []
        for kf in msg.keyframes:
            if bool(kf.bad):
                continue
            p = kf.pose.position
            keyframes.append((int(kf.id), float(p.x), float(p.y)))
        loop_edges = [(int(e.from_id), int(e.to_id)) for e in msg.loop_edges]
        self._model.update_graph(int(msg.revision), bool(msg.active_map_connected),
                                 keyframes, loop_edges)

    def _on_event(self, msg: TrackingEvent) -> None:
        now = self._now_s()
        typ = EVENT_TYPE_NAMES.get(int(msg.type), "UNKNOWN")
        if typ == "LOST":
            self._model.start_loss(int(msg.graph_revision), now)
        elif typ == "RELOCALIZED":
            self._model.mark_relocalized(int(msg.graph_revision), now)
        self._events.append({"type": typ, "t": stamp_key(msg.header.stamp)[0] + stamp_key(msg.header.stamp)[1] * 1e-9,
                             "graph_revision": int(msg.graph_revision), "detail": str(msg.detail)})

    def _render_map(self) -> None:
        candidate = self._model.next_map_to_render()
        if candidate is None:
            return
        map_value, revision = candidate
        gray = occupancy_to_gray(map_value.cells, map_value.width, map_value.height)
        buf = io.BytesIO()
        from PIL import Image
        Image.fromarray(gray, mode="L").save(buf, format="PNG")
        self._model.publish_rendered_map(map_value.stamp, buf.getvalue())

    def state_json(self) -> bytes:
        now = self._now_s()
        state = self._model.snapshot(now)
        state["tracking"]["hz"] = round(self._tracking_rate.hz(now), 1)
        state.setdefault("odom", {})["hz"] = round(self._odom_rate.hz(now), 1)
        state["events"] = list(self._events)[-30:][::-1]
        state["paths"]["corrected"] = state["paths"].get("corrected", [])
        return json.dumps(state).encode("utf-8")

    def map_png(self) -> bytes:
        return self._model.map_png()

    def _make_handler(self):
        server, web_dir = self, self._web_dir
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass
            def _send(self, code, content_type, body, no_cache=False):
                self.send_response(code); self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                if no_cache: self.send_header("Cache-Control", "no-store")
                self.end_headers()
                if self.command != "HEAD": self.wfile.write(body)
            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path == "/state": self._send(200, "application/json", server.state_json(), True); return
                if path == "/map.png":
                    png = server.map_png()
                    self._send(200 if png else 503, "image/png" if png else "text/plain", png or b"no map yet", True); return
                rel = "index.html" if path in ("/", "") else path.lstrip("/")
                # web_dir is resolved once at startup. With colcon
                # --symlink-install, every FILE under web_dir is itself a
                # symlink back into the source tree by design, so fully
                # resolving the candidate file (following that symlink) and
                # checking containment against web_dir would reject every
                # legitimate file. Instead we resolve only the *directory*
                # component of the request (collapsing any ".." segments)
                # and require that resolved directory to be web_dir itself
                # or a real descendant of it; the final filename is then
                # joined unresolved, so its own symlink target is irrelevant
                # to the traversal check. This still blocks "../" escapes,
                # which escape via the directory component, not the leaf.
                rel_path = FsPath(rel)
                if rel_path.is_absolute() or not rel_path.name:
                    self._send(404, "text/plain", b"not found"); return
                try:
                    dir_resolved = (web_dir / rel_path.parent).resolve()
                    if dir_resolved != web_dir:
                        dir_resolved.relative_to(web_dir)
                except (OSError, ValueError):
                    self._send(404, "text/plain", b"not found"); return
                fpath = dir_resolved / rel_path.name
                if not fpath.is_file(): self._send(404, "text/plain", b"not found"); return
                ctype = {".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".png": "image/png"}.get(fpath.suffix, "application/octet-stream")
                self._send(200, ctype, fpath.read_bytes())
        return Handler

    def shutdown_http(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardServer()
    # SingleThreadedExecutor avoids the busy-wait CPU overhead of
    # MultiThreadedExecutor with MutuallyExclusive callback groups.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.shutdown_http(); executor.remove_node(node); node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
