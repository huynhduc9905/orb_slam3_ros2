from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Optional, Sequence


@dataclass(frozen=True)
class BenchmarkResult:
    mode: str
    playback_rate: float
    wall_duration_s: float
    received_frames: int
    tracking_fps: float
    ok_frames: int
    ok_ratio: float
    initialized: bool
    invalid_reason: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "playback_rate": self.playback_rate,
            "wall_duration_s": self.wall_duration_s,
            "received_frames": self.received_frames,
            "tracking_fps": self.tracking_fps,
            "ok_frames": self.ok_frames,
            "ok_ratio": self.ok_ratio,
            "initialized": self.initialized,
            "invalid_reason": self.invalid_reason,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> BenchmarkResult:
        return cls(
            mode=str(data["mode"]),
            playback_rate=float(data["playback_rate"]),
            wall_duration_s=float(data["wall_duration_s"]),
            received_frames=int(data["received_frames"]),
            tracking_fps=float(data["tracking_fps"]),
            ok_frames=int(data["ok_frames"]),
            ok_ratio=float(data["ok_ratio"]),
            initialized=bool(data["initialized"]),
            invalid_reason=data.get("invalid_reason"),
        )

    def write(self, path: Path | str) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp = p.with_name(f".{p.name}.tmp.{os.getpid()}")
        try:
            tmp.write_text(json.dumps(self.as_dict(), indent=2) + "\n", encoding="utf-8")
            os.replace(tmp, p)
        finally:
            if tmp.exists():
                try:
                    tmp.unlink()
                except OSError:
                    pass


def load_result(path: Path | str) -> BenchmarkResult:
    p = Path(path)
    data = json.loads(p.read_text(encoding="utf-8"))
    return BenchmarkResult.from_dict(data)


class TrackingRateCounter:
    def __init__(self, mode: str, playback_rate: float, min_duration_s: float):
        self.mode = mode
        self.playback_rate = float(playback_rate)
        self.min_duration_s = float(min_duration_s)
        self.start_s: Optional[float] = None
        self.received_frames: int = 0
        self.ok_frames: int = 0
        self.initialized: bool = False

    def observe(self, tracking_ok: bool, now_s: float) -> None:
        if not self.initialized:
            if tracking_ok:
                self.initialized = True
                self.start_s = float(now_s)
                self.received_frames = 1
                self.ok_frames = 1
        else:
            self.received_frames += 1
            if tracking_ok:
                self.ok_frames += 1

    def finish(self, now_s: float) -> BenchmarkResult:
        if not self.initialized or self.start_s is None:
            return BenchmarkResult(
                mode=self.mode,
                playback_rate=self.playback_rate,
                wall_duration_s=0.0,
                received_frames=0,
                tracking_fps=0.0,
                ok_frames=0,
                ok_ratio=0.0,
                initialized=False,
                invalid_reason="orb_never_initialized",
            )

        wall_duration_s = float(now_s - self.start_s)
        if self.received_frames == 0:
            return BenchmarkResult(
                mode=self.mode,
                playback_rate=self.playback_rate,
                wall_duration_s=wall_duration_s,
                received_frames=0,
                tracking_fps=0.0,
                ok_frames=0,
                ok_ratio=0.0,
                initialized=True,
                invalid_reason="no_eligible_frames",
            )

        ok_ratio = float(self.ok_frames / self.received_frames)

        if wall_duration_s < self.min_duration_s:
            return BenchmarkResult(
                mode=self.mode,
                playback_rate=self.playback_rate,
                wall_duration_s=wall_duration_s,
                received_frames=self.received_frames,
                tracking_fps=0.0,
                ok_frames=self.ok_frames,
                ok_ratio=ok_ratio,
                initialized=True,
                invalid_reason="measurement_duration_below_minimum",
            )

        tracking_fps = float(self.received_frames / wall_duration_s) if wall_duration_s > 0 else 0.0
        return BenchmarkResult(
            mode=self.mode,
            playback_rate=self.playback_rate,
            wall_duration_s=wall_duration_s,
            received_frames=self.received_frames,
            tracking_fps=tracking_fps,
            ok_frames=self.ok_frames,
            ok_ratio=ok_ratio,
            initialized=True,
            invalid_reason=None,
        )


def select_stress_point(
    results: Sequence[BenchmarkResult], source_camera_hz: float
) -> BenchmarkResult:
    for result in sorted(results, key=lambda item: item.playback_rate):
        if result.invalid_reason is not None:
            raise ValueError(f"invalid ORB-only result at {result.playback_rate}x: {result.invalid_reason}")
        if result.tracking_fps < source_camera_hz * result.playback_rate:
            return result
    raise ValueError("ORB-only saturation point was not found")


def compare_full_stack(
    baseline: BenchmarkResult, full_stack: BenchmarkResult
) -> Dict[str, Any]:
    if baseline.invalid_reason or full_stack.invalid_reason:
        raise ValueError("cannot compare invalid benchmark results")
    if baseline.playback_rate != full_stack.playback_rate:
        raise ValueError("baseline and full-stack playback rates differ")
    threshold = 0.80 * baseline.tracking_fps
    return {
        "playback_rate": baseline.playback_rate,
        "orb_only_tracking_fps": baseline.tracking_fps,
        "full_stack_tracking_fps": full_stack.tracking_fps,
        "threshold_fps": threshold,
        "passed": full_stack.tracking_fps >= threshold,
    }


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Tracking benchmark utility")
    subparsers = parser.add_subparsers(dest="subcommand", required=True)

    select_parser = subparsers.add_parser("select", help="Select stress point result")
    select_parser.add_argument("--source-camera-hz", type=float, required=True)
    select_parser.add_argument("--result", action="append", dest="results", type=Path, required=True)

    compare_parser = subparsers.add_parser("compare", help="Compare full stack result against baseline")
    compare_parser.add_argument("--baseline", type=Path, required=True)
    compare_parser.add_argument("--full-stack", type=Path, required=True)
    compare_parser.add_argument("--output", type=Path, required=True)

    try:
        parsed = parser.parse_args(args)
    except SystemExit as exc:
        return exc.code if isinstance(exc.code, int) else 2

    if parsed.subcommand == "select":
        try:
            benchmark_results = [load_result(p) for p in parsed.results]
            selected = select_stress_point(benchmark_results, parsed.source_camera_hz)
            print(json.dumps(selected.as_dict(), indent=2))
            return 0
        except Exception as exc:
            sys.stderr.write(f"Error selecting stress point: {exc}\n")
            return 1

    elif parsed.subcommand == "compare":
        try:
            baseline_result = load_result(parsed.baseline)
            full_stack_result = load_result(parsed.full_stack)
            comparison = compare_full_stack(baseline_result, full_stack_result)

            out_path = parsed.output
            out_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = out_path.with_name(f".{out_path.name}.tmp.{os.getpid()}")
            try:
                tmp.write_text(json.dumps(comparison, indent=2) + "\n", encoding="utf-8")
                os.replace(tmp, out_path)
            finally:
                if tmp.exists():
                    try:
                        tmp.unlink()
                    except OSError:
                        pass

            return 0 if comparison["passed"] else 1
        except ValueError as exc:
            sys.stderr.write(f"Error comparing results: {exc}\n")
            return 2
        except Exception as exc:
            sys.stderr.write(f"Error comparing results: {exc}\n")
            return 2

    return 0
