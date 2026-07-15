"""TF tree audit for bag replay.

Waits up to 10 simulated seconds for required TF edges, writes tf_audit.json,
and fails if a supplemental publisher would duplicate a recorded parent/child pair.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# (parent, child, source_label)
REQUIRED_EDGES: List[Tuple[str, str, str]] = [
    ("odom", "base_link", "odometry"),
    ("base_link", "camera_link", "supplemental"),
    ("camera_link", "camera_infra1_optical_frame", "recorded"),
    ("camera_link", "camera_infra2_optical_frame", "recorded"),
    ("base_link", "base_scan", "supplemental"),
]


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert RPY (radians) to quaternion (x, y, z, w)."""
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


def check_no_duplicate_supplemental(
    recorded_pairs: Sequence[Tuple[str, str]],
    supplemental_pairs: Sequence[Tuple[str, str]],
) -> List[str]:
    """Return list of error strings if supplemental would duplicate recorded edges."""
    recorded = set(recorded_pairs)
    errors: List[str] = []
    for parent, child in supplemental_pairs:
        if (parent, child) in recorded:
            errors.append(
                f"supplemental TF would duplicate recorded edge {parent}->{child}"
            )
    return errors


class TfAuditNode(Node):
    """Audit required TF edges and write machine-readable report."""

    def __init__(self) -> None:
        super().__init__("tf_audit")
        self.declare_parameter("artifact_dir", ".")
        self.declare_parameter("timeout_s", 10.0)
        self.declare_parameter("poll_period_s", 0.1)
        # JSON list of recorded parent/child pairs from bag inspection (launch sets this).
        self.declare_parameter("recorded_static_pairs", "[]")
        # JSON list of supplemental parent/child pairs that will be published.
        self.declare_parameter("supplemental_pairs", "[]")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self._artifact_dir = Path(
            self.get_parameter("artifact_dir").get_parameter_value().string_value
        )
        self._timeout_s = self.get_parameter("timeout_s").get_parameter_value().double_value
        self._poll_period_s = (
            self.get_parameter("poll_period_s").get_parameter_value().double_value
        )

        recorded_raw = self.get_parameter("recorded_static_pairs").get_parameter_value().string_value
        supplemental_raw = self.get_parameter("supplemental_pairs").get_parameter_value().string_value
        self._recorded_pairs: List[Tuple[str, str]] = [
            (p[0], p[1]) for p in json.loads(recorded_raw)
        ]
        self._supplemental_pairs: List[Tuple[str, str]] = [
            (p[0], p[1]) for p in json.loads(supplemental_raw)
        ]

        self._buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._listener = TransformListener(self._buffer, self)
        self._start_time: Optional[Time] = None
        self._done = False
        self._exit_code = 0

        # Immediate duplicate check (does not need TF).
        dup_errors = check_no_duplicate_supplemental(
            self._recorded_pairs, self._supplemental_pairs
        )
        if dup_errors:
            self.get_logger().error("; ".join(dup_errors))
            self._write_report(
                edges=[],
                overall_pass=False,
                errors=dup_errors,
            )
            self._done = True
            self._exit_code = 1
            # Schedule shutdown after a tick so spin can exit cleanly.
            self.create_timer(0.01, self._finish)
            return

        self._timer = self.create_timer(self._poll_period_s, self._poll)
        self.get_logger().info(
            f"TF audit waiting up to {self._timeout_s}s (sim time) for required edges"
        )

    def _finish(self) -> None:
        """Destroy timers and request shutdown."""
        try:
            if hasattr(self, "_timer") and self._timer is not None:
                self._timer.cancel()
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()

    def _poll(self) -> None:
        if self._done:
            return
        now = self.get_clock().now()
        if self._start_time is None:
            # Wait until clock is non-zero when using sim time.
            if now.nanoseconds == 0:
                return
            self._start_time = now
            return

        elapsed = (now - self._start_time).nanoseconds * 1e-9
        edge_results: List[Dict[str, Any]] = []
        all_ok = True
        for parent, child, source in REQUIRED_EDGES:
            result = self._lookup_edge(parent, child, source, now)
            edge_results.append(result)
            if not result["pass"]:
                all_ok = False

        if all_ok or elapsed >= self._timeout_s:
            self._done = True
            if not all_ok:
                self.get_logger().error(
                    f"TF audit FAILED after {elapsed:.2f}s; missing edges"
                )
                self._exit_code = 1
            else:
                self.get_logger().info(f"TF audit PASSED in {elapsed:.2f}s")
                self._exit_code = 0
            self._write_report(edges=edge_results, overall_pass=all_ok, errors=[])
            self._finish()

    def _lookup_edge(
        self, parent: str, child: str, source: str, now: Time
    ) -> Dict[str, Any]:
        entry: Dict[str, Any] = {
            "parent": parent,
            "child": child,
            "source": source,
            "pass": False,
            "translation": None,
            "quaternion": None,
            "error": None,
        }
        try:
            # Lookup at Time(0) for latest available (static + dynamic).
            tf = self._buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=0.0)
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            entry["translation"] = [t.x, t.y, t.z]
            entry["quaternion"] = [r.x, r.y, r.z, r.w]
            entry["pass"] = True
        except TransformException as exc:
            entry["error"] = str(exc)
        return entry

    def _write_report(
        self,
        *,
        edges: List[Dict[str, Any]],
        overall_pass: bool,
        errors: List[str],
    ) -> None:
        self._artifact_dir.mkdir(parents=True, exist_ok=True)
        report = {
            "pass": overall_pass,
            "errors": errors,
            "edges": edges,
            "recorded_static_pairs": [
                {"parent": p, "child": c} for p, c in self._recorded_pairs
            ],
            "supplemental_pairs": [
                {"parent": p, "child": c} for p, c in self._supplemental_pairs
            ],
        }
        out = self._artifact_dir / "tf_audit.json"
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(f"Wrote TF audit report to {out}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfAuditNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
        # One more spin to allow finish timer if needed.
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = getattr(node, "_exit_code", 0)
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
