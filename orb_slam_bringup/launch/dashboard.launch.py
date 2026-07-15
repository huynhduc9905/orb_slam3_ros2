"""Dashboard launch: static HTTP server + read-only foxglove_bridge.

Serves the orb_slam_dashboard web app and a capability-restricted ROS
WebSocket bridge bound to a configurable Tailscale/LAN host. Never
imports foxglove_bridge Python modules (package may be absent at parse
time); the Node only resolves at launch time.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    dashboard_host = LaunchConfiguration("dashboard_host").perform(context)
    http_port = LaunchConfiguration("http_port").perform(context)
    websocket_port = LaunchConfiguration("websocket_port").perform(context)

    try:
        bringup_share = Path(get_package_share_directory("orb_slam_bringup"))
    except Exception:  # pragma: no cover - source-tree fallback
        bringup_share = Path(__file__).resolve().parents[1]

    bridge_config = bringup_share / "config" / "read_only_bridge.yaml"
    if not bridge_config.is_file():
        bridge_config = (
            Path(__file__).resolve().parents[1] / "config" / "read_only_bridge.yaml"
        )

    # Resolve the plain Python HTTP server executable. It is NOT a ROS node
    # (argparse rejects --ros-args that Node would inject).
    try:
        dash_prefix = Path(get_package_prefix("orb_slam_dashboard"))
        server_exe = (
            dash_prefix / "lib" / "orb_slam_dashboard" / "orb_slam_dashboard_server"
        )
    except Exception:  # pragma: no cover
        server_exe = Path("orb_slam_dashboard_server")

    # URL host: if bound to 0.0.0.0, tell the operator to open localhost.
    url_host = "127.0.0.1" if dashboard_host in ("0.0.0.0", "::", "") else dashboard_host
    url = (
        f"http://{url_host}:{http_port}/"
        f"?ws=ws://{url_host}:{websocket_port}"
    )

    return [
        LogInfo(msg=f"[dashboard] open {url}"),
        # Static HTTP server for the built dashboard (Task 3).
        # ExecuteProcess (not Node) so launch does not append --ros-args.
        ExecuteProcess(
            cmd=[
                str(server_exe),
                "--host",
                dashboard_host,
                "--port",
                http_port,
            ],
            name="orb_slam_dashboard_server",
            output="screen",
        ),
        # Read-only foxglove_bridge: yaml locks capabilities/whitelists;
        # address/port overridden from launch args so the bridge binds the
        # configured Tailscale/LAN host (yaml defaults are 0.0.0.0:8765).
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[
                str(bridge_config),
                {
                    "address": dashboard_host,
                    "port": int(websocket_port),
                },
            ],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dashboard_host",
                default_value="100.102.92.45",
                description=(
                    "Tailscale/LAN bind address for HTTP server and foxglove_bridge"
                ),
            ),
            DeclareLaunchArgument(
                "http_port",
                default_value="51871",
                description="Static dashboard HTTP port",
            ),
            DeclareLaunchArgument(
                "websocket_port",
                default_value="8765",
                description="foxglove_bridge WebSocket port",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
