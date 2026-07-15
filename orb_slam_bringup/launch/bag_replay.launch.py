"""Bag replay launch: supplemental TF, odom adapter, TF audit, wrapper, mapper.

Metrics recorder (Task 5) and dashboard (Task 3/4) are optional integration
points — referenced defensively so this launch remains importable before those
packages exist.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_profile(profile_path: Path) -> dict:
    with profile_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _inspect_bag_static_pairs(bag_path: str) -> List[Tuple[str, str]]:
    """Read recorded /tf_static parent/child pairs from the bag (once)."""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - env issue
        raise RuntimeError(f"rosbag2_py required to inspect bag TF: {exc}") from exc

    storage = StorageOptions(uri=bag_path, storage_id="mcap")
    converter = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage, converter)
    TfMessage = get_message("tf2_msgs/msg/TFMessage")
    pairs: List[Tuple[str, str]] = []
    seen = set()
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != "/tf_static":
            continue
        msg = deserialize_message(data, TfMessage)
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            if key not in seen:
                seen.add(key)
                pairs.append(key)
        # /tf_static is typically one latched message early in the bag.
        if pairs:
            break
    return pairs


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def _static_transform_node(
    name: str,
    parent: str,
    child: str,
    xyz: Sequence[float],
    rpy: Sequence[float],
) -> Node:
    qx, qy, qz, qw = _rpy_to_quaternion(rpy[0], rpy[1], rpy[2])
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x",
            str(xyz[0]),
            "--y",
            str(xyz[1]),
            "--z",
            str(xyz[2]),
            "--qx",
            str(qx),
            "--qy",
            str(qy),
            "--qz",
            str(qz),
            "--qw",
            str(qw),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


def _optional_include(path: Path, launch_arguments: Optional[dict] = None):
    """Return IncludeLaunchDescription if path exists, else a LogInfo stub."""
    if not path.is_file():
        return LogInfo(msg=f"[bag_replay] optional launch not found (skipped): {path}")
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _setup(context, *args, **kwargs):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    artifact_dir = LaunchConfiguration("artifact_dir").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    ros_domain_id = LaunchConfiguration("ros_domain_id").perform(context)
    publish_odom_tf = LaunchConfiguration("publish_odom_tf").perform(context).lower()
    start_dashboard = LaunchConfiguration("start_dashboard").perform(context).lower()
    dashboard_host = LaunchConfiguration("dashboard_host").perform(context)

    bringup_share = Path(get_package_share_directory("orb_slam_bringup"))
    profile_path = bringup_share / "config" / "tasterobot_bag.yaml"
    # Fall back to source tree when package is not yet installed (dev).
    if not profile_path.is_file():
        profile_path = Path(__file__).resolve().parents[1] / "config" / "tasterobot_bag.yaml"
    profile = _load_profile(profile_path)

    if not bag_path:
        bag_path = profile["bag"]["path"]

    supplemental = profile["supplemental_tf"]
    supplemental_pairs = [
        (supplemental["camera_link"]["parent"], supplemental["camera_link"]["child"]),
        (supplemental["base_scan"]["parent"], supplemental["base_scan"]["child"]),
    ]

    recorded_pairs = _inspect_bag_static_pairs(bag_path)
    # Fail early if supplemental would duplicate recorded parent/child pairs.
    recorded_set = set(recorded_pairs)
    for parent, child in supplemental_pairs:
        if (parent, child) in recorded_set:
            raise RuntimeError(
                f"Supplemental TF {parent}->{child} would duplicate a recorded "
                f"/tf_static edge; refusing to start publishers."
            )
    # Confirm only the two configured mount edges are the ones we must add
    # among the required mount set (base_link->camera_link, base_link->base_scan).
    missing_mounts = [
        pair for pair in supplemental_pairs if pair not in recorded_set
    ]
    if set(missing_mounts) != set(supplemental_pairs):
        raise RuntimeError(
            f"Expected both mount edges absent from bag; missing={missing_mounts} "
            f"configured={supplemental_pairs}"
        )

    os.makedirs(artifact_dir, exist_ok=True)

    actions = []

    # Isolate domain if requested.
    if ros_domain_id:
        actions.append(SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id))

    # Supplemental static mounts (authoritative recorded edges stay from bag).
    cam = supplemental["camera_link"]
    scan = supplemental["base_scan"]
    actions.append(
        _static_transform_node(
            "supplemental_tf_camera_link",
            cam["parent"],
            cam["child"],
            cam["xyz"],
            cam["rpy"],
        )
    )
    actions.append(
        _static_transform_node(
            "supplemental_tf_base_scan",
            scan["parent"],
            scan["child"],
            scan["xyz"],
            scan["rpy"],
        )
    )

    # Replay-only odom -> base_link TF (default true for bag replay).
    if publish_odom_tf in ("true", "1", "yes"):
        actions.append(
            Node(
                package="orb_slam_bringup",
                executable="odom_tf_adapter",
                name="odom_tf_adapter",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": profile["odometry"]["topic"],
                        "odom_frame": profile["odometry"]["frame"],
                        "base_frame": profile["odometry"]["child_frame"],
                    }
                ],
                output="screen",
            )
        )

    actions.append(
        Node(
            package="orb_slam_bringup",
            executable="tf_audit",
            name="tf_audit",
            parameters=[
                {
                    "use_sim_time": True,
                    "artifact_dir": artifact_dir,
                    "timeout_s": 10.0,
                    "recorded_static_pairs": json.dumps(
                        [[p, c] for p, c in recorded_pairs]
                    ),
                    "supplemental_pairs": json.dumps(
                        [[p, c] for p, c in supplemental_pairs]
                    ),
                }
            ],
            output="screen",
        )
    )

    # ORB-SLAM3 wrapper
    try:
        wrapper_share = get_package_share_directory("orb_slam3_wrapper")
        settings_file = str(Path(wrapper_share) / "config" / "tasterobot_stereo.yaml")
    except Exception:  # pragma: no cover
        settings_file = ""

    actions.append(
        Node(
            package="orb_slam3_wrapper",
            executable="orb_slam3_wrapper_node",
            name="orb_slam3_wrapper",
            parameters=[
                {
                    "use_sim_time": True,
                    "left_image_topic": profile["camera"]["left_image"],
                    "right_image_topic": profile["camera"]["right_image"],
                    "left_info_topic": profile["camera"]["left_info"],
                    "right_info_topic": profile["camera"]["right_info"],
                    "base_frame": "base_link",
                    "map_frame": "orb_map",
                    "settings_file": settings_file,
                }
            ],
            output="screen",
        )
    )

    # Lidar mapper
    actions.append(
        Node(
            package="orb_lidar_mapper",
            executable="orb_lidar_mapper_node",
            name="orb_lidar_mapper",
            parameters=[
                {
                    "use_sim_time": True,
                    "odom_topic": profile["odometry"]["topic"],
                    "scan_topic": profile["lidar"]["topic"],
                    "map_frame": "orb_map",
                    "base_frame": "base_link",
                }
            ],
            output="screen",
        )
    )

    # Metrics recorder (Task 5): read-only subscribers; flushes on bag-exit Shutdown.
    actions.append(
        Node(
            package="orb_slam_bringup",
            executable="metrics_recorder",
            name="metrics_recorder",
            parameters=[
                {
                    "use_sim_time": True,
                    "artifact_dir": artifact_dir,
                    "bag_path": bag_path,
                    "bag_duration_s": float(profile.get("bag", {}).get("duration_s", 0.0)),
                    "config_path": str(profile_path),
                    "repo_dir": str(Path(__file__).resolve().parents[2]),
                    "expected_stereo_pairs": 6633,
                }
            ],
            output="screen",
        )
    )

    # Dashboard (Task 4): HTTP + read-only foxglove_bridge when requested.
    if start_dashboard in ("true", "1", "yes"):
        dashboard_candidates = [
            bringup_share / "launch" / "dashboard.launch.py",
            Path(__file__).resolve().parent / "dashboard.launch.py",
        ]
        dashboard_launch = next((p for p in dashboard_candidates if p.is_file()), None)
        if dashboard_launch is not None:
            actions.append(
                _optional_include(
                    dashboard_launch,
                    {
                        "dashboard_host": dashboard_host,
                    },
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg=(
                        "[bag_replay] start_dashboard=true but dashboard.launch.py "
                        "not found; skipping"
                    )
                )
            )

    # Bag playback with /clock for use_sim_time.
    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--rate",
            rate,
        ],
        name="bag_play",
        output="screen",
    )
    actions.append(bag_play)

    # Shutdown the whole launch when bag playback exits so metrics can flush.
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_play,
                on_exit=[
                    LogInfo(msg="[bag_replay] bag playback exited; shutting down"),
                    EmitEvent(event=Shutdown(reason="bag playback finished")),
                ],
            )
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    # Default profile path (used only for default bag_path value).
    try:
        share = get_package_share_directory("orb_slam_bringup")
        default_profile = Path(share) / "config" / "tasterobot_bag.yaml"
    except Exception:
        default_profile = (
            Path(__file__).resolve().parents[1] / "config" / "tasterobot_bag.yaml"
        )
    default_bag = "/home/duc/robot/20260713_152907"
    if default_profile.is_file():
        try:
            default_bag = _load_profile(default_profile)["bag"]["path"]
        except Exception:  # noqa: BLE001
            pass

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value=default_bag,
                description="Path to the recorded bag directory",
            ),
            DeclareLaunchArgument(
                "artifact_dir",
                default_value="/tmp/orb_slam_artifacts",
                description="Directory for tf_audit.json and later metrics",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="1.0",
                description="Bag playback rate",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="42",
                description="Isolated ROS_DOMAIN_ID for replay",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Publish odom->base_link from /odom_wheel (replay only)",
            ),
            DeclareLaunchArgument(
                "start_dashboard",
                default_value="false",
                description="Include dashboard.launch.py (HTTP + read-only bridge)",
            ),
            DeclareLaunchArgument(
                "dashboard_host",
                default_value="100.102.92.45",
                description=(
                    "Tailscale/LAN bind host for dashboard when start_dashboard:=true"
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
