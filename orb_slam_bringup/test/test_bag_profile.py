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


DASHBOARD_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "dashboard.launch.py"
)
BAG_REPLAY_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "bag_replay.launch.py"
)


def test_dashboard_launch_uses_direct_read_only_server_without_foxglove():
    text = DASHBOARD_LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'executable="dashboard_server"' in text
    assert "foxglove_bridge" not in text
    assert "read_only_bridge.yaml" not in text
    assert "websocket_port" not in text


def test_bag_replay_forwards_sim_time_to_direct_dashboard():
    text = BAG_REPLAY_LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"use_sim_time": "true"' in text
    assert "foxglove_bridge" not in text


def test_bag_replay_declares_explicit_tracking_benchmark_modes():
    text = BAG_REPLAY_LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"benchmark_mode"' in text
    assert '"benchmark_min_duration_s"' in text
    assert 'tracking_benchmark_probe' in text
    assert '"orb_only"' in text
    assert '"full_stack"' in text


def test_benchmark_launch_keeps_dashboard_disabled_by_mode():
    text = BAG_REPLAY_LAUNCH_PATH.read_text(encoding="utf-8")
    assert "benchmark modes do not start the dashboard" in text

def test_readme_documents_tracking_performance_benchmark():
    readme = Path(__file__).resolve().parents[2] / "README.md"
    text = readme.read_text(encoding="utf-8")
    assert "run_tracking_performance_benchmark.sh" in text
    assert "80%" in text
    assert "first ORB-only rate" in text
    assert "tracking_benchmark.json" in text


def test_wrapper_log_capture_configuration():
    from launch import LaunchContext
    import sys
    import os
    import shlex
    import subprocess

    sys.path.insert(0, str(BAG_REPLAY_LAUNCH_PATH.parent))
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("bag_replay", str(BAG_REPLAY_LAUNCH_PATH))
        bag_replay = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bag_replay)
        desc = bag_replay.generate_launch_description()

        opaque_action = None
        for entity in desc.entities:
            if type(entity).__name__ == "OpaqueFunction":
                opaque_action = entity
                break
        
        assert opaque_action is not None
        
        context = LaunchContext()
        os.environ['ROS_DOMAIN_ID'] = '42'
        context.launch_configurations['bag_path'] = '/home/duc/robot/mock_bag'
        context.launch_configurations['artifact_dir'] = '/tmp/path with spaces and $meta'
        context.launch_configurations['rate'] = '1.0'
        context.launch_configurations['ros_domain_id'] = '42'
        context.launch_configurations['publish_odom_tf'] = 'true'
        context.launch_configurations['start_dashboard'] = 'false'
        context.launch_configurations['dashboard_host'] = 'localhost'
        context.launch_configurations['benchmark_mode'] = 'off'
        context.launch_configurations['benchmark_min_duration_s'] = '10.0'
        
        import unittest.mock
        with unittest.mock.patch.object(bag_replay, "_inspect_bag_static_pairs", return_value=[]):
            actions = opaque_action.execute(context)

        wrapper_action = None
        for action in actions:
            if type(action).__name__ == "ExecuteProcess":
                cmd = action.cmd
                resolved_cmd = []
                for c in cmd:
                    if isinstance(c, list):
                        resolved_cmd.append("".join([sub_c.perform(context) if hasattr(sub_c, "perform") else str(sub_c) for sub_c in c]))
                    else:
                        resolved_cmd.append(c.perform(context) if hasattr(c, "perform") else str(c))
                
                if resolved_cmd and resolved_cmd[0] == "bash" and "orb_slam3_wrapper" in resolved_cmd[-1]:
                    wrapper_action = action
                    break

        assert wrapper_action is not None

        cmd = wrapper_action.cmd
        resolved_cmd = []
        for c in cmd:
            if isinstance(c, list):
                resolved_cmd.append("".join([sub_c.perform(context) if hasattr(sub_c, "perform") else str(sub_c) for sub_c in c]))
            else:
                resolved_cmd.append(c.perform(context) if hasattr(c, "perform") else str(c))

        assert resolved_cmd[:4] == ["bash", "-o", "pipefail", "-c"]
        script_content = resolved_cmd[4]

        parsed = shlex.split(script_content)
        assert parsed[0] == "ros2"
        assert parsed[1] == "run"
        assert parsed[2] == "orb_slam3_wrapper"
        assert parsed[3] == "orb_slam3_wrapper_node"
        
        assert "__node:=orb_slam3_wrapper" in parsed
        assert "use_sim_time:=true" in parsed
        
        # Verify quotation of settings path
        settings_idx = next(i for i, arg in enumerate(parsed) if arg.startswith("settings_file:="))
        assert parsed[settings_idx].startswith("settings_file:=")
        
        # Verify the tee part
        assert "2>&1" in script_content
        assert "| tee -a" in script_content
        assert "'/tmp/path with spaces and $meta/orb_slam3_wrapper.log'" in script_content

        assert getattr(wrapper_action, 'shell', False) == False

        # Subprocess probe to verify pipefail
        test_script = "bash", "-o", "pipefail", "-c", "false 2>&1 | tee /dev/null"
        result = subprocess.run(test_script, capture_output=True)
        assert result.returncode != 0

    finally:
        sys.path.pop(0)
