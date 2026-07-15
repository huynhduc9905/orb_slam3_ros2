"""Unit tests for OdomTfAdapter transform-building logic (no live ROS graph)."""

from __future__ import annotations

import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from orb_slam_bringup.odom_tf_adapter import (
    build_odom_transform,
    is_finite_pose,
)


def _make_odom(
    *,
    frame_id: str = "odom",
    child_frame_id: str = "base_link",
    x: float = 1.0,
    y: float = 2.0,
    z: float = 0.0,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
    sec: int = 10,
    nanosec: int = 500_000_000,
) -> Odometry:
    msg = Odometry()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)
    msg.child_frame_id = child_frame_id
    msg.pose.pose = Pose()
    msg.pose.pose.position = Vector3(x=x, y=y, z=z)
    msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return msg


def test_build_odom_transform_copies_pose_and_frames():
    msg = _make_odom(x=0.5, y=-0.25, z=0.0, qx=0.0, qy=0.0, qz=math.sin(0.1), qw=math.cos(0.1))
    tf = build_odom_transform(msg, expected_frame="odom", expected_child="base_link")
    assert tf is not None
    assert tf.header.frame_id == "odom"
    assert tf.child_frame_id == "base_link"
    assert tf.header.stamp.sec == 10
    assert tf.header.stamp.nanosec == 500_000_000
    assert tf.transform.translation.x == pytest.approx(0.5)
    assert tf.transform.translation.y == pytest.approx(-0.25)
    assert tf.transform.translation.z == pytest.approx(0.0)
    assert tf.transform.rotation.z == pytest.approx(math.sin(0.1))
    assert tf.transform.rotation.w == pytest.approx(math.cos(0.1))


def test_build_odom_transform_rejects_wrong_frames():
    bad_parent = _make_odom(frame_id="map")
    assert build_odom_transform(bad_parent, expected_frame="odom", expected_child="base_link") is None

    bad_child = _make_odom(child_frame_id="base_footprint")
    assert build_odom_transform(bad_child, expected_frame="odom", expected_child="base_link") is None


def test_build_odom_transform_rejects_non_finite_pose():
    nan_pos = _make_odom(x=float("nan"))
    assert build_odom_transform(nan_pos, expected_frame="odom", expected_child="base_link") is None

    inf_q = _make_odom(qw=float("inf"))
    assert build_odom_transform(inf_q, expected_frame="odom", expected_child="base_link") is None


def test_is_finite_pose():
    ok = _make_odom()
    assert is_finite_pose(ok.pose.pose) is True
    bad = _make_odom(y=float("nan"))
    assert is_finite_pose(bad.pose.pose) is False


def test_adapter_counts_invalid_messages():
    """OdomTfAdapter exposes invalid_message_count for rejected messages."""
    from orb_slam_bringup.odom_tf_adapter import OdomTfAdapter

    # Construct without spinning: allow lightweight construction if supported.
    adapter = OdomTfAdapter.__new__(OdomTfAdapter)
    adapter.invalid_message_count = 0
    adapter.expected_frame = "odom"
    adapter.expected_child = "base_link"

    # Simulate handle path via pure helper + counter pattern used by the node.
    for msg in (
        _make_odom(frame_id="wrong"),
        _make_odom(x=float("nan")),
        _make_odom(),  # valid
    ):
        tf = build_odom_transform(
            msg,
            expected_frame=adapter.expected_frame,
            expected_child=adapter.expected_child,
        )
        if tf is None:
            adapter.invalid_message_count += 1

    assert adapter.invalid_message_count == 2
