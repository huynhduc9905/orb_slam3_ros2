from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from orb_slam3_msgs.msg import TrackedFrame
    TRACK_OK = getattr(TrackedFrame, "OK", 2)
except ImportError:
    TrackedFrame = None
    TRACK_OK = 2

from orb_slam_bringup.tracking_benchmark import TrackingRateCounter


class TrackingBenchmarkProbe(Node):
    """ROS node that measures ORB tracking rate and outputs benchmark results on shutdown."""

    def __init__(self, node_name: str = "tracking_benchmark_probe", _for_test: bool = False):
        self._flushed = False
        self._counter: TrackingRateCounter
        self._artifact_dir: Path

        if _for_test:
            return

        super().__init__(node_name)
        self.declare_parameter("artifact_dir", "")
        self.declare_parameter("mode", "orb_only")
        self.declare_parameter("playback_rate", 1.0)
        self.declare_parameter("min_duration_s", 0.0)
        self.declare_parameter("tracked_frame_topic", "/orb_slam3/tracked_frame")

        artifact_dir_str = self.get_parameter("artifact_dir").get_parameter_value().string_value
        mode = self.get_parameter("mode").get_parameter_value().string_value
        playback_rate = self.get_parameter("playback_rate").get_parameter_value().double_value
        min_duration_s = self.get_parameter("min_duration_s").get_parameter_value().double_value
        tracked_frame_topic = self.get_parameter("tracked_frame_topic").get_parameter_value().string_value

        self._artifact_dir = Path(artifact_dir_str)
        self._counter = TrackingRateCounter(
            mode=mode,
            playback_rate=playback_rate,
            min_duration_s=min_duration_s,
        )

        msg_type = TrackedFrame
        if msg_type is None:
            from rosidl_runtime_py.utilities import get_message
            msg_type = get_message("orb_slam3_msgs/msg/TrackedFrame")

        self.create_subscription(
            msg_type,
            tracked_frame_topic,
            self.on_tracked_frame,
            qos_profile_sensor_data,
        )

        try:
            self.context.on_shutdown(self.flush)
        except Exception:
            pass

    @classmethod
    def for_test(
        cls,
        artifact_dir: Path | str,
        mode: str,
        playback_rate: float,
        min_duration_s: float,
    ) -> TrackingBenchmarkProbe:
        probe = cls(_for_test=True)
        probe._artifact_dir = Path(artifact_dir)
        probe._counter = TrackingRateCounter(
            mode=mode,
            playback_rate=playback_rate,
            min_duration_s=min_duration_s,
        )
        return probe

    def on_tracked_frame(self, msg) -> None:
        tracking_state = getattr(msg, "tracking_state", None)
        is_ok = tracking_state == TRACK_OK
        if is_ok or self._counter.initialized:
            now_s = time.monotonic()
        else:
            now_s = 0.0
        self._counter.observe(is_ok, now_s)

    def flush(self) -> None:
        if self._flushed:
            return
        self._flushed = True
        result = self._counter.finish(time.monotonic())
        output_path = self._artifact_dir / "tracking_benchmark.json"

        summary = (
            f"[TrackingBenchmarkProbe] Flushed benchmark result to {output_path}: "
            f"mode={result.mode}, rate={result.playback_rate}x, received={result.received_frames}, "
            f"ok={result.ok_frames}, fps={result.tracking_fps:.2f}, initialized={result.initialized}, "
            f"invalid_reason={result.invalid_reason}"
        )
        try:
            self.get_logger().info(summary)
        except Exception:
            sys.stdout.write(summary + "\n")

        try:
            result.write(output_path)
        except Exception as exc:
            err_msg = f"Failed to write benchmark result to {output_path}: {exc}"
            try:
                self.get_logger().error(err_msg)
            except Exception:
                sys.stderr.write(err_msg + "\n")
            raise


def main(args=None) -> int:
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    probe = TrackingBenchmarkProbe()
    try:
        rclpy.spin(probe)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            probe.flush()
        finally:
            try:
                probe.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
