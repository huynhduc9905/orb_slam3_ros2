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
    
    benchmark_mode = LaunchConfiguration("benchmark_mode").perform(context).strip().lower()
    benchmark_min_duration_s = float(
        LaunchConfiguration("benchmark_min_duration_s").perform(context)
    )
    if benchmark_mode not in ("off", "orb_only", "full_stack"):
        raise RuntimeError("benchmark_mode must be one of: off, orb_only, full_stack")
    if benchmark_min_duration_s < 0.0:
        raise RuntimeError("benchmark_min_duration_s must be nonnegative")

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
    recorded_set = set(recorded_pairs)
    # Publish profile supplemental mounts only when the bag is missing them.
    # Newer bags (e.g. forward-and-back-origin) already record
    # base_link→camera_link and base_link→base_scan; older 20260713 bags do not.
    missing_mounts = [
        pair for pair in supplemental_pairs if pair not in recorded_set
    ]
    present_mounts = [
        pair for pair in supplemental_pairs if pair in recorded_set
    ]

    os.makedirs(artifact_dir, exist_ok=True)

    actions = []

    # Isolate domain if requested.
    if ros_domain_id:
        actions.append(SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id))

    if present_mounts:
        actions.append(
            LogInfo(
                msg=(
                    "[bag_replay] using bag /tf_static for mounts: "
                    + ", ".join(f"{p}->{c}" for p, c in present_mounts)
                )
            )
        )

    # Supplemental static mounts for edges absent from the bag only.
    cam = supplemental["camera_link"]
    scan = supplemental["base_scan"]
    mount_specs = {
        (cam["parent"], cam["child"]): (
            "supplemental_tf_camera_link",
            cam["xyz"],
            cam["rpy"],
        ),
        (scan["parent"], scan["child"]): (
            "supplemental_tf_base_scan",
            scan["xyz"],
            scan["rpy"],
        ),
    }
    for parent, child in missing_mounts:
        name, xyz, rpy = mount_specs[(parent, child)]
        actions.append(
            LogInfo(msg=f"[bag_replay] publishing supplemental TF {parent}->{child}")
        )
        actions.append(_static_transform_node(name, parent, child, xyz, rpy))

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
                        [[p, c] for p, c in missing_mounts]
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

    wrapper_log_path = str(Path(artifact_dir) / "orb_slam3_wrapper.log")

    actions.append(
        ExecuteProcess(
            cmd=[
                "sh",
                "-c",
                f"ros2 run orb_slam3_wrapper orb_slam3_wrapper_node --ros-args "
                f"-p use_sim_time:=true "
                f"-p left_image_topic:={profile["camera"]["left_image"]} "
                f"-p right_image_topic:={profile["camera"]["right_image"]} "
                f"-p left_info_topic:={profile["camera"]["left_info"]} "
                f"-p right_info_topic:={profile["camera"]["right_info"]} "
                f"-p base_frame:=base_link "
                f"-p map_frame:=orb_map "
                f"-p settings_file:={settings_file} "
                f"2>&1 | tee -a \"{wrapper_log_path}\""
            ],
            name="orb_slam3_wrapper",
            output="screen",
            shell=True,
        )
    )


    # Lidar mapper
    if benchmark_mode in ("off", "full_stack"):
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
    if benchmark_mode in ("off", "full_stack"):
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
                        # Soft expected count for report gates; bag-specific.
                        # 0 disables strict expected-pairs matching (recorder still counts).
                        "expected_stereo_pairs": int(
                            profile.get("bag", {}).get("expected_stereo_pairs", 0)
                        ),
                    }
                ],
                output="screen",
            )
        )

    # Tracking benchmark probe (Task 2)
    if benchmark_mode != "off":
        actions.append(
            LogInfo(msg="[bag_replay] benchmark modes do not start the dashboard")
        )
        actions.append(
            Node(
                package="orb_slam_bringup",
                executable="tracking_benchmark_probe",
                name="tracking_benchmark_probe",
                parameters=[
                    {
                        "use_sim_time": True,
                        "artifact_dir": artifact_dir,
                        "mode": benchmark_mode,
                        "playback_rate": float(rate),
                        "min_duration_s": benchmark_min_duration_s,
                    }
                ],
                output="screen",
            )
        )

    # Dashboard: custom read-only dashboard_server (no foxglove) when requested.
    if start_dashboard in ("true", "1", "yes") and benchmark_mode == "off":
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
                        "use_sim_time": "true",
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
                    Shutdown(reason="bag playback finished"),
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
            DeclareLaunchArgument(
                "benchmark_mode",
                default_value="off",
                description="Mode for tracking benchmark: off, orb_only, or full_stack",
            ),
            DeclareLaunchArgument(
                "benchmark_min_duration_s",
                default_value="10.0",
                description="Minimum tracking duration in seconds for benchmark mode",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
