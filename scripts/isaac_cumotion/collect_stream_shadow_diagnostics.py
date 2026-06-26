#!/usr/bin/env python3
"""Collect stream-bridge shadow diagnostics for hardware bring-up.

Run this while the Isaac Sim 6 cuMotion stream server and ROS bridge are running.
It subscribes to the topics needed to prove the shadow path is healthy before
enabling `publish_safe_targets:=true` and `collision_safety:=true`.
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from collections import Counter, defaultdict
from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


SIDES = ("left", "right")


def _parse_sides(value: str) -> tuple[str, ...]:
    sides = tuple(side.strip() for side in value.split(",") if side.strip())
    if not sides:
        raise argparse.ArgumentTypeError("expected at least one side")
    invalid = [side for side in sides if side not in SIDES]
    if invalid:
        raise argparse.ArgumentTypeError(f"invalid side(s) {invalid}; expected left and/or right")
    return tuple(dict.fromkeys(sides))


def _stats(values: Iterable[float]) -> dict:
    values = [float(value) for value in values]
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
        "last": values[-1],
    }


class ShadowDiagnostics(Node):
    def __init__(self, *, active_sides: tuple[str, ...]) -> None:
        super().__init__("isaac_cumotion_shadow_diagnostics")
        self._active_sides = active_sides
        self._start_mono = time.monotonic()
        self._last_seen: dict[str, float] = {}
        self._counts: Counter[str] = Counter()
        self._status_counts: Counter[str] = Counter()
        self._reason_counts: Counter[str] = Counter()
        self._controller_hz = []
        self._input_age_ms = []
        self._safe_error = defaultdict(list)
        self._last_status = None
        self._last_reason = None

        self.create_subscription(String, "/factr_teleop/isaac_cumotion_stream/status", self._status_cb, 10)
        self.create_subscription(String, "/factr_teleop/isaac_cumotion_stream/reason", self._reason_cb, 10)
        self.create_subscription(Float32, "/factr_teleop/isaac_cumotion_stream/controller_hz", self._controller_hz_cb, 10)
        self.create_subscription(Float32, "/factr_teleop/isaac_cumotion_stream/input_age_ms", self._input_age_cb, 10)

        for side in SIDES:
            self.create_subscription(
                JointState,
                f"/ur/{side}/obs_ur_state",
                lambda msg, side=side: self._joint_cb(f"{side}.obs", msg),
                10,
            )
            self.create_subscription(
                JointState,
                f"/factr_teleop/{side}/desired_ur_pos",
                lambda msg, side=side: self._joint_cb(f"{side}.desired", msg),
                10,
            )
            self.create_subscription(
                JointState,
                f"/factr_teleop/{side}/safe_ur_pos",
                lambda msg, side=side: self._joint_cb(f"{side}.safe", msg),
                10,
            )
            self.create_subscription(
                Float32,
                f"/factr_teleop/{side}/isaac_cumotion_safe_error",
                lambda msg, side=side: self._safe_error_cb(side, msg),
                10,
            )

    def _mark(self, key: str) -> None:
        self._counts[key] += 1
        self._last_seen[key] = time.monotonic()

    def _status_cb(self, msg: String) -> None:
        self._mark("stream.status")
        self._last_status = msg.data
        self._status_counts[msg.data] += 1

    def _reason_cb(self, msg: String) -> None:
        self._mark("stream.reason")
        self._last_reason = msg.data
        self._reason_counts[msg.data] += 1

    def _controller_hz_cb(self, msg: Float32) -> None:
        self._mark("stream.controller_hz")
        self._controller_hz.append(float(msg.data))

    def _input_age_cb(self, msg: Float32) -> None:
        self._mark("stream.input_age_ms")
        self._input_age_ms.append(float(msg.data))

    def _joint_cb(self, key: str, msg: JointState) -> None:
        del msg
        self._mark(key)

    def _safe_error_cb(self, side: str, msg: Float32) -> None:
        self._mark(f"{side}.safe_error")
        self._safe_error[side].append(float(msg.data))

    def summary(self) -> dict:
        elapsed = max(time.monotonic() - self._start_mono, 1e-9)
        rates = {key: value / elapsed for key, value in sorted(self._counts.items())}
        now = time.monotonic()
        ages = {key: now - value for key, value in sorted(self._last_seen.items())}
        required = []
        for side in self._active_sides:
            required.extend([f"{side}.obs", f"{side}.desired", f"{side}.safe_error"])
        if "right" in self._active_sides:
            required.append("left.obs")
        missing = [key for key in required if self._counts[key] == 0]
        stale = [key for key in required if key in ages and ages[key] > 0.50]
        safe_counts = {side: self._counts[f"{side}.safe"] for side in self._active_sides}
        return {
            "duration_s": elapsed,
            "active_sides": list(self._active_sides),
            "last_status": self._last_status,
            "last_reason": self._last_reason,
            "status_counts": dict(self._status_counts),
            "reason_counts": dict(self._reason_counts),
            "rates_hz": rates,
            "last_seen_age_s": ages,
            "missing_required_topics": missing,
            "stale_required_topics": stale,
            "safe_ur_pos_counts": safe_counts,
            "controller_hz": _stats(self._controller_hz),
            "input_age_ms": _stats(self._input_age_ms),
            "safe_error": {side: _stats(values) for side, values in sorted(self._safe_error.items())},
            "shadow_healthy": (
                not missing
                and not stale
                and self._last_status in ("shadow", "filtered", "pass_through")
                and bool(self._controller_hz)
                and all(count == 0 for count in safe_counts.values())
            ),
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--active-sides", type=_parse_sides, default=("right",))
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    rclpy.init()
    node = ShadowDiagnostics(active_sides=args.active_sides)
    try:
        deadline = time.monotonic() + max(float(args.duration_s), 0.1)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        summary = node.summary()
        text = json.dumps(summary, indent=2, sort_keys=True)
        print(text, flush=True)
        if args.output_json:
            with open(args.output_json, "w", encoding="utf-8") as stream:
                stream.write(text)
                stream.write("\n")
        if not summary["shadow_healthy"]:
            raise SystemExit("shadow diagnostics did not meet healthy criteria")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
