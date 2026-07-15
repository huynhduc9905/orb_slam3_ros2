"""Explicit bag profile facts for the tasteRobot integration fixture."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml


PROFILE_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "tasterobot_bag.yaml"
)

# Seven required input topics for bag replay (images, infos, odom, lidar, tf_static).
REQUIRED_INPUT_TOPICS = [
    "/camera/camera/infra1/image_rect_raw",
    "/camera/camera/infra2/image_rect_raw",
    "/camera/camera/infra1/camera_info",
    "/camera/camera/infra2/camera_info",
    "/odom_wheel",
    "/scan_origin",
    "/tf_static",
]


def load_profile() -> dict:
    with PROFILE_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_tasterobot_bag_profile_is_explicit():
    profile = load_profile()
    assert profile["camera"]["width"] == 848
    assert profile["camera"]["height"] == 480
    assert profile["camera"]["fx"] == pytest.approx(426.9840393066406)
    assert profile["camera"]["baseline_m"] == pytest.approx(0.0501881428)
    assert profile["supplemental_tf"]["camera_link"]["xyz"] == [0.346, 0.01, 0.1]
    assert profile["supplemental_tf"]["base_scan"]["yaw"] == pytest.approx(math.pi)


def test_profile_lists_seven_required_input_topics():
    profile = load_profile()
    required = profile["required_input_topics"]
    assert required == REQUIRED_INPUT_TOPICS
    assert len(required) == 7
    # Cross-check camera / odometry / lidar topic fields match the list.
    cam = profile["camera"]
    assert cam["left_image"] in required
    assert cam["right_image"] in required
    assert cam["left_info"] in required
    assert cam["right_info"] in required
    assert profile["odometry"]["topic"] in required
    assert profile["lidar"]["topic"] in required
    assert "/tf_static" in required


def test_profile_documents_right_camera_info_wrong_frame_id():
    profile = load_profile()
    assert profile["camera"]["right_camera_info_frame_is_wrong"] is True


BRIDGE_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "read_only_bridge.yaml"
)

# Exactly the three anchored topic regexes allowed on the read-only bridge.
APPROVED_TOPIC_WHITELIST = [
    r"^/orb_lidar/(map|map_revision|corrected_path_revisioned|wheel_path|provisional_scan)$",
    r"^/orb_slam3/(tracked_frame|events|keyframes|loop_edges|tracking_image/compressed)$",
    r"^/diagnostics$",
]


def load_bridge_config() -> dict:
    with BRIDGE_CONFIG_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_read_only_bridge_locks_down_capabilities():
    """foxglove_bridge must be read-only: no real caps/write-whitelists, approved topics only.

    Empty lockdown lists must be typed as string arrays (e.g. [""]) so rclcpp can
    load them; bare [] crashes foxglove_bridge. Security property: no non-empty
    entry (so capabilities: ["clientPublish"] fails this test).
    """
    cfg = load_bridge_config()
    params = cfg["foxglove_bridge"]["ros__parameters"]

    for key in (
        "capabilities",
        "service_whitelist",
        "param_whitelist",
        "client_topic_whitelist",
    ):
        values = params[key]
        assert isinstance(values, list), f"{key} must be a list"
        # Security: no real capability / write-whitelist name (would FAIL on
        # e.g. capabilities: ["clientPublish"]).
        assert [c for c in values if c] == [], (
            f"{key} must contain no non-empty entries (read-only); got {values!r}"
        )
        # Typing: bare [] crashes foxglove_bridge (rclcpp cannot infer type);
        # require the verified empty string-array form.
        assert values == [""], (
            f"{key} must be [\"\"] so rclcpp types it as string[]; got {values!r}"
        )

    topic_whitelist = params["topic_whitelist"]
    assert topic_whitelist == APPROVED_TOPIC_WHITELIST
    assert len(topic_whitelist) == 3
    for pattern in topic_whitelist:
        assert pattern.startswith("^"), f"topic regex must be anchored: {pattern}"
        assert pattern.endswith("$"), f"topic regex must be anchored: {pattern}"
