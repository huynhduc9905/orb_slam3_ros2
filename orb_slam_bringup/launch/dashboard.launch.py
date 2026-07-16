"""Dashboard launch: custom read-only dashboard_server (no foxglove).

Starts a single ROS node that subscribes to the SLAM/mapper topics in-graph and
serves a server-rendered map PNG + a JSON /state endpoint over plain HTTP to a
minimal vanilla-JS frontend. An external ROS WebSocket bridge is intentionally
not used: its mere presence in the DDS graph was measured to degrade ORB-SLAM3
tracking ~5x, while a plain in-graph subscriber node does not. The wrapper and
mapper are untouched.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    dashboard_host = LaunchConfiguration("dashboard_host").perform(context)
    http_port = LaunchConfiguration("http_port").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) in (
        "true", "True", "1", "yes",
    )

    url_host = "127.0.0.1" if dashboard_host in ("0.0.0.0", "::", "") else dashboard_host
    url = f"http://{url_host}:{http_port}/"

    return [
        LogInfo(msg=f"[dashboard] open {url}"),
        Node(
            package="orb_slam_bringup",
            executable="dashboard_server",
            name="dashboard_server",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "host": dashboard_host,
                    "port": int(http_port),
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dashboard_host",
                default_value="0.0.0.0",
                description="Bind address for the dashboard HTTP server",
            ),
            DeclareLaunchArgument(
                "http_port",
                default_value="51871",
                description="Dashboard HTTP port",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time (true during bag replay)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
