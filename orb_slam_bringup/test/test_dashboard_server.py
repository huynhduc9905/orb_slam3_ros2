"""Unit tests for dashboard_server pure helpers (no live ROS graph)."""

from __future__ import annotations

import math
import threading
from http.client import HTTPConnection

import numpy as np
import pytest
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from orb_slam3_msgs.msg import MapRevision
from visualization_msgs.msg import Marker

from orb_slam_bringup.dashboard_server import (
    PNG_FREE,
    PNG_OCCUPIED,
    PNG_UNKNOWN,
    RateTracker,
    decimate_path,
    occupancy_to_gray,
    pose2d,
    stamp_key,
    yaw_from_quaternion,
)
from orb_slam_bringup.dashboard_model import DashboardModel, MapEnvelope, RevisionEnvelope
from orb_slam_bringup.dashboard_server import DashboardServer


# ── RateTracker ──────────────────────────────────────────────────────────────

def test_rate_zero_when_empty():
    assert RateTracker(2.0).hz(0.0) == 0.0


def test_rate_count_over_window():
    rt = RateTracker(2.0)
    for t in (0.0, 0.5, 1.0, 1.5):
        rt.record(t)
    assert rt.hz(1.5) == pytest.approx(2.0)


def test_rate_prunes_old():
    rt = RateTracker(1.0)
    rt.record(0.0)
    rt.record(0.5)
    assert rt.hz(1.6) == pytest.approx(0.0)


def test_rate_invalid_window():
    with pytest.raises(ValueError):
        RateTracker(0.0)


# ── yaw_from_quaternion ──────────────────────────────────────────────────────

def test_yaw_identity_is_zero():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_yaw_ninety_degrees():
    half = math.pi / 4  # 90 deg total
    assert yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(
        math.pi / 2
    )


# ── occupancy_to_gray ────────────────────────────────────────────────────────

def test_occupancy_mapping_and_vertical_flip():
    # 2x2 grid, row-major (ROS row 0 = bottom):
    #   row0: free(0), occupied(100)
    #   row1: unknown(-1), mid-occupied(50)
    data = [0, 100, -1, 50]
    gray = occupancy_to_gray(data, 2, 2)
    assert gray.shape == (2, 2)
    # flipud: image row 0 == ROS row 1 (unknown, mid-occupied)
    assert gray[0, 0] == PNG_UNKNOWN
    assert gray[0, 1] == PNG_OCCUPIED
    # image row 1 == ROS row 0 (free, occupied)
    assert gray[1, 0] == PNG_FREE
    assert gray[1, 1] == PNG_OCCUPIED


def test_occupancy_zero_sized():
    assert occupancy_to_gray([], 0, 0).size == 0


def test_occupancy_threshold_at_50():
    gray = occupancy_to_gray([49, 50], 2, 1)
    assert gray[0, 0] == PNG_FREE
    assert gray[0, 1] == PNG_OCCUPIED


# ── decimate_path ────────────────────────────────────────────────────────────

def test_decimate_noop_when_under_limit():
    pts = [(0.0, 0.0), (1.0, 1.0)]
    assert decimate_path(pts, 10) == pts


def test_decimate_downsamples_and_keeps_last():
    pts = [(float(i), 0.0) for i in range(100)]
    out = decimate_path(pts, 10)
    assert len(out) <= 11
    assert out[-1] == (99.0, 0.0)  # last always preserved
    assert out[0] == (0.0, 0.0)


def test_decimate_empty():
    assert decimate_path([], 10) == []


def test_stamp_key_and_pose_conversion():
    class Stamp: sec, nanosec = 12, 34
    assert stamp_key(Stamp()) == (12, 34)

    class Position:
        x = 1.25; y = -2.0
    class Orientation:
        x = 0.0; y = 0.0; z = 0.0; w = 1.0
    class P:
        position = Position(); orientation = Orientation()
    assert pose2d(P()).x == pytest.approx(1.25)
    assert pose2d(P()).yaw == pytest.approx(0.0)


def test_model_replaces_revisioned_path_and_does_not_render_non_published():
    model = DashboardModel()
    stamp = (1, 0)
    model.ingest_map(MapEnvelope(stamp, 1.0, 0.0, 0.0, 2, 1, [0, 100]))
    for state in ("BUILDING", "FAILED"):
        model.ingest_revision(RevisionEnvelope(stamp, state, 8, 3, 1))
        assert model.next_map_to_render() is None
    model.replace_corrected_path(9, [])
    model.replace_corrected_path(10, [])
    assert model.snapshot(0)["paths"]["corrected"] == []


def _serve(web_dir):
    class Stub:
        _web_dir = web_dir
        def state_json(self): return b"{}"
        def map_png(self): return b""
    return __import__("http.server", fromlist=["ThreadingHTTPServer"]).ThreadingHTTPServer(
        ("127.0.0.1", 0), DashboardServer._make_handler(Stub()))


def test_static_request_path_traversal_is_rejected(tmp_path):
    # The only attacker-controlled input to the static handler is the
    # request path; files under web_dir are placed at build/install time
    # (see below), not by a runtime request. This test asserts a request
    # cannot walk out of web_dir via "..".
    (tmp_path / "index.html").write_text("ok")
    secret = tmp_path.parent / "web-secret"
    secret.mkdir(exist_ok=True)
    (secret / "file").write_text("secret")

    httpd = _serve(tmp_path.resolve())
    thread = threading.Thread(target=httpd.serve_forever, daemon=True); thread.start()
    try:
        conn = HTTPConnection("127.0.0.1", httpd.server_port)
        conn.request("GET", "/../web-secret/file")
        assert conn.getresponse().status == 404
    finally:
        httpd.shutdown(); httpd.server_close(); thread.join()


def test_static_nested_traversal_via_subdirectory_is_rejected(tmp_path):
    (tmp_path / "index.html").write_text("ok")
    sub = tmp_path / "assets"
    sub.mkdir()
    (sub / "app.js").write_text("//js")
    secret = tmp_path.parent / "web-secret2"
    secret.mkdir(exist_ok=True)
    (secret / "file").write_text("secret")

    httpd = _serve(tmp_path.resolve())
    thread = threading.Thread(target=httpd.serve_forever, daemon=True); thread.start()
    try:
        conn = HTTPConnection("127.0.0.1", httpd.server_port)
        conn.request("GET", "/assets/../../web-secret2/file")
        assert conn.getresponse().status == 404
    finally:
        httpd.shutdown(); httpd.server_close(); thread.join()


def test_static_symlink_install_layout_serves_per_file_symlinks(tmp_path):
    # colcon --symlink-install places a *symlink per file* under the
    # installed web_dir, each pointing at the real file elsewhere in the
    # workspace (source tree or build/ dir) — this is the normal, expected
    # layout, not an attacker-controlled escape, and must keep working.
    source_root = tmp_path / "source-web"
    source_root.mkdir()
    (source_root / "index.html").write_text("ok")
    installed_root = tmp_path / "installed-web"
    installed_root.mkdir()
    (installed_root / "index.html").symlink_to(source_root / "index.html")

    httpd = _serve(installed_root.resolve())
    thread = threading.Thread(target=httpd.serve_forever, daemon=True); thread.start()
    try:
        conn = HTTPConnection("127.0.0.1", httpd.server_port)
        conn.request("GET", "/")
        response = conn.getresponse()
        assert response.status == 200
        assert response.read() == b"ok"
    finally:
        httpd.shutdown(); httpd.server_close(); thread.join()


def test_static_root_symlink_serves_in_root_files(tmp_path):
    source_root = tmp_path / "source-web"
    source_root.mkdir()
    (source_root / "index.html").write_text("ok")
    installed_root = tmp_path / "installed-web"
    installed_root.symlink_to(source_root, target_is_directory=True)

    httpd = _serve(installed_root.resolve())
    thread = threading.Thread(target=httpd.serve_forever, daemon=True); thread.start()
    try:
        conn = HTTPConnection("127.0.0.1", httpd.server_port)
        conn.request("GET", "/")
        response = conn.getresponse()
        assert response.status == 200
        assert response.read() == b"ok"
    finally:
        httpd.shutdown(); httpd.server_close(); thread.join()


def adapter(max_path_points=3, map_frame="orb_map"):
    class Adapter:
        _model = DashboardModel()
        _max_path_points = max_path_points
        _map_frame = map_frame
    return Adapter()


def point(x, y):
    value = Point()
    value.x, value.y = float(x), float(y)
    return value


def test_render_timer_publishes_real_png_with_matching_visible_revisions():
    server = adapter()
    grid = OccupancyGrid()
    grid.header.stamp.sec, grid.header.stamp.nanosec = 2, 3
    grid.info.resolution = 0.1
    grid.info.width, grid.info.height = 2, 2
    grid.info.origin.position.x, grid.info.origin.position.y = -1.0, 2.0
    grid.data = [-1, 0, 50, 100]
    revision = MapRevision()
    revision.header.stamp = grid.header.stamp
    revision.state = MapRevision.PUBLISHED
    revision.graph_revision, revision.map_revision = 12, 7
    revision.committed_scan_count = 4

    DashboardServer._on_map(server, grid)
    DashboardServer._on_map_revision(server, revision)
    DashboardServer._render_map(server)

    png = server._model.map_png()
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    from PIL import Image
    with Image.open(__import__("io").BytesIO(png)) as image:
        assert image.format == "PNG"
        assert image.size == (2, 2)
    snapshot = server._model.snapshot(0)
    assert snapshot["map"]["graph_revision"] == 12
    assert snapshot["map"]["revision"] == 7
    assert snapshot["map_revision"]["graph_revision"] == 12
    assert snapshot["map_revision"]["map_revision"] == 7


def test_provisional_marker_requires_map_points_and_uses_configured_bound():
    server = adapter(max_path_points=3)
    marker = Marker()
    marker.header.frame_id = "orb_map"
    marker.type, marker.action = Marker.POINTS, Marker.ADD
    marker.points = [point(i, i + 0.5) for i in range(5)]
    DashboardServer._on_provisional_scan(server, marker)
    points = server._model.snapshot(0)["paths"]["provisional"]
    assert len(points) == 3
    assert points[0] == [0.0, 0.5]
    assert points[-1] == [4.0, 4.5]

    for frame_id in ("map", "odom"):
        wrong_frame = Marker()
        wrong_frame.header.frame_id = frame_id
        wrong_frame.type, wrong_frame.action = Marker.POINTS, Marker.ADD
        wrong_frame.points = [point(99, 99)]
        DashboardServer._on_provisional_scan(server, wrong_frame)
        assert server._model.snapshot(0)["paths"]["provisional"] == points

    wrong_type = Marker()
    wrong_type.header.frame_id = "orb_map"
    wrong_type.type, wrong_type.action = Marker.LINE_LIST, Marker.ADD
    wrong_type.points = [point(88, 88)]
    DashboardServer._on_provisional_scan(server, wrong_type)
    assert server._model.snapshot(0)["paths"]["provisional"] == points


@pytest.mark.parametrize("action", [Marker.DELETE, Marker.DELETEALL])
def test_provisional_marker_delete_actions_clear(action):
    server = adapter()
    add = Marker()
    add.header.frame_id = "orb_map"
    add.type, add.action = Marker.POINTS, Marker.ADD
    add.points = [point(1, 2)]
    DashboardServer._on_provisional_scan(server, add)

    delete = Marker()
    delete.header.frame_id = "orb_map"
    delete.action = action
    DashboardServer._on_provisional_scan(server, delete)
    assert server._model.snapshot(0)["paths"]["provisional"] == []


def test_provisional_marker_accepts_nondefault_configured_frame():
    server = adapter(map_frame="site_map")
    marker = Marker()
    marker.header.frame_id = "site_map"
    marker.type, marker.action = Marker.POINTS, Marker.ADD
    marker.points = [point(3, 4)]

    DashboardServer._on_provisional_scan(server, marker)

    assert server._model.snapshot(0)["paths"]["provisional"] == [[3.0, 4.0]]


@pytest.mark.parametrize("action", [Marker.DELETE, Marker.DELETEALL])
@pytest.mark.parametrize("frame_id", ["map", "odom"])
def test_provisional_marker_delete_actions_ignore_other_frames(action, frame_id):
    server = adapter()
    add = Marker()
    add.header.frame_id = "orb_map"
    add.type, add.action = Marker.POINTS, Marker.ADD
    add.points = [point(1, 2)]
    DashboardServer._on_provisional_scan(server, add)

    delete = Marker()
    delete.header.frame_id = frame_id
    delete.action = action
    DashboardServer._on_provisional_scan(server, delete)

    assert server._model.snapshot(0)["paths"]["provisional"] == [[1.0, 2.0]]
