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
