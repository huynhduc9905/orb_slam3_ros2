"""Regression tests for metrics recorder process shutdown."""

from __future__ import annotations

import sys
import types

from orb_slam_bringup import metrics_recorder


def test_main_treats_external_shutdown_as_clean_and_flushes(monkeypatch):
    calls = []

    class FakeRecorder:
        def __init__(self):
            self.node = types.SimpleNamespace(destroy_node=destroy_node)

        def _flush(self):
            calls.append("flush")

    class ExternalShutdownException(Exception):
        pass

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda args: calls.append(("init", args))
    fake_rclpy.spin = lambda node: (_ for _ in ()).throw(
        ExternalShutdownException()
    )
    fake_rclpy.ok = lambda: True
    fake_rclpy.shutdown = lambda: calls.append("shutdown")

    class FakeMultiThreadedExecutor:
        def __init__(self, num_threads=None):
            pass
        def add_node(self, node):
            pass
        def spin(self):
            raise ExternalShutdownException()
        def shutdown(self):
            pass

    fake_executors = types.ModuleType("rclpy.executors")
    fake_executors.ExternalShutdownException = ExternalShutdownException
    fake_executors.MultiThreadedExecutor = FakeMultiThreadedExecutor
    fake_rclpy.executors = fake_executors

    def destroy_node():
        calls.append("destroy")

    monkeypatch.setattr(metrics_recorder, "MetricsRecorderNode", FakeRecorder)
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", fake_executors)

    # The real main() must absorb the executor shutdown exception and finish
    # its normal flush/destroy/shutdown cleanup path.
    metrics_recorder.main(args=["--test"])

    assert calls == [("init", ["--test"]), "flush", "destroy", "shutdown"]
