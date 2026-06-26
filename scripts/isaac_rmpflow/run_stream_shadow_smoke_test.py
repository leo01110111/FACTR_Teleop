#!/usr/bin/env python3
"""Run a local fake-ROS smoke test for stream-bridge shadow mode.

This test starts the pass-through stream server and ROS stream bridge on test
ports, publishes fake right-arm observed/desired joint states plus fake left-arm
observed state, and verifies that shadow mode stays alive without publishing
`/factr_teleop/right/safe_ur_pos`. It does not use Isaac, RTDE, Dynamixels, or
robot hardware.
"""

from __future__ import annotations

import argparse
import contextlib
import os
import signal
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")


class FakeShadowStreamNode(Node):
    def __init__(self, *, duration_s: float) -> None:
        super().__init__("fake_isaac_stream_shadow_smoke")
        self._duration_s = float(duration_s)
        self._right_state_pub = self.create_publisher(JointState, "/ur/right/obs_ur_state", 10)
        self._left_state_pub = self.create_publisher(JointState, "/ur/left/obs_ur_state", 10)
        self._desired_pub = self.create_publisher(JointState, "/factr_teleop/right/desired_ur_pos", 10)
        self.safe_count = 0
        self.status_counts: Counter[str] = Counter()
        self.reason_counts: Counter[str] = Counter()
        self.controller_hz = []
        self.input_age_ms = []
        self.safe_error_count = 0
        self.last_reason = None
        self.create_subscription(JointState, "/factr_teleop/right/safe_ur_pos", self._safe_cb, 10)
        self.create_subscription(String, "/factr_teleop/isaac_rmpflow_stream/status", self._status_cb, 10)
        self.create_subscription(String, "/factr_teleop/isaac_rmpflow_stream/reason", self._reason_cb, 10)
        self.create_subscription(Float32, "/factr_teleop/isaac_rmpflow_stream/controller_hz", self._hz_cb, 10)
        self.create_subscription(Float32, "/factr_teleop/isaac_rmpflow_stream/input_age_ms", self._input_age_cb, 10)
        self.create_subscription(
            Float32,
            "/factr_teleop/right/isaac_stream_safe_error",
            self._safe_error_cb,
            10,
        )
        self.create_timer(0.005, self._tick)

    def _safe_cb(self, msg: JointState) -> None:
        del msg
        self.safe_count += 1

    def _status_cb(self, msg: String) -> None:
        self.status_counts[msg.data] += 1

    def _reason_cb(self, msg: String) -> None:
        self.reason_counts[msg.data] += 1
        self.last_reason = msg.data

    def _hz_cb(self, msg: Float32) -> None:
        self.controller_hz.append(float(msg.data))

    def _input_age_cb(self, msg: Float32) -> None:
        self.input_age_ms.append(float(msg.data))

    def _safe_error_cb(self, msg: Float32) -> None:
        del msg
        self.safe_error_count += 1

    def _tick(self) -> None:
        right = JointState()
        right.position = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]
        left = JointState()
        left.position = [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]
        desired = JointState()
        desired.position = [-1.55, -1.58, -1.56, -1.56, 1.55, 0.02]
        self._right_state_pub.publish(right)
        self._left_state_pub.publish(left)
        self._desired_pub.publish(desired)

    def run(self) -> None:
        start = time.time()
        while time.time() - start < self._duration_s:
            rclpy.spin_once(self, timeout_sec=0.01)


def _terminate(process: subprocess.Popen) -> None:
    with contextlib.suppress(Exception):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5.0)
    with contextlib.suppress(Exception):
        if process.poll() is None:
            process.kill()


def _wait_for_ready(process: subprocess.Popen, *, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = process.stdout.readline() if process.stdout is not None else ""
        if line:
            print(line.rstrip(), flush=True)
            if '"event": "ready"' in line:
                return
        elif process.poll() is not None:
            raise RuntimeError(f"server exited early with code {process.returncode}")
    raise TimeoutError("stream server did not report ready")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-endpoint", default="tcp://127.0.0.1:5608")
    parser.add_argument("--output-endpoint", default="tcp://127.0.0.1:5609")
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--min-status-count", type=int, default=5)
    parser.add_argument("--min-controller-hz-count", type=int, default=5)
    parser.add_argument("--min-safe-error-count", type=int, default=5)
    args = parser.parse_args()

    env = os.environ.copy()
    env["PYTHONPATH"] = f"{REPO_DIR / 'scripts/isaac_rmpflow'}:{env.get('PYTHONPATH', '')}"

    server = subprocess.Popen(
        [
            sys.executable,
            str(REPO_DIR / "scripts/isaac_rmpflow/isaac_rmpflow_stream_server.py"),
            "--mode",
            "pass_through",
            "--loop-hz",
            "100.0",
            "--input-endpoint",
            args.input_endpoint,
            "--output-endpoint",
            args.output_endpoint,
            "--status-period-s",
            "0.5",
        ],
        cwd=str(REPO_DIR),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    bridge = None
    try:
        _wait_for_ready(server, timeout_s=5.0)
        bridge = subprocess.Popen(
            [
                "ros2",
                "launch",
                "launch/isaac_rmpflow_stream_bridge.py",
                "active_sides:=right",
                f"input_endpoint:={args.input_endpoint}",
                f"output_endpoint:={args.output_endpoint}",
                "publish_hz:=100.0",
                "max_joint_step_rad:=0.05",
                "max_safe_target_distance_rad:=0.05",
                "publish_safe_targets:=false",
                "require_rmp_policy:=false",
            ],
            cwd=str(REPO_DIR),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(1.0)
        if bridge.poll() is not None:
            raise RuntimeError(f"bridge exited early with code {bridge.returncode}")

        rclpy.init()
        node = FakeShadowStreamNode(duration_s=args.duration_s)
        try:
            node.run()
            print(f"safe_count {node.safe_count}", flush=True)
            print(f"status_counts {dict(node.status_counts)}", flush=True)
            print(f"reason_counts {dict(node.reason_counts)}", flush=True)
            print(f"controller_hz_count {len(node.controller_hz)}", flush=True)
            print(f"controller_hz_last {node.controller_hz[-1] if node.controller_hz else None}", flush=True)
            print(f"input_age_ms_last {node.input_age_ms[-1] if node.input_age_ms else None}", flush=True)
            print(f"safe_error_count {node.safe_error_count}", flush=True)
            print(f"last_reason {node.last_reason}", flush=True)
            if bridge.poll() is not None:
                raise RuntimeError(f"bridge exited during test with code {bridge.returncode}")
            if node.safe_count != 0:
                raise SystemExit("expected zero right safe targets in shadow mode")
            if node.status_counts["shadow"] < args.min_status_count:
                raise SystemExit(f"expected at least {args.min_status_count} shadow status messages")
            if len(node.controller_hz) < args.min_controller_hz_count:
                raise SystemExit(f"expected at least {args.min_controller_hz_count} controller_hz messages")
            if max(node.controller_hz, default=0.0) <= 0.0:
                raise SystemExit("expected positive controller_hz values")
            if node.safe_error_count < args.min_safe_error_count:
                raise SystemExit(f"expected at least {args.min_safe_error_count} right safe-error diagnostics")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    finally:
        if bridge is not None:
            _terminate(bridge)
        _terminate(server)


if __name__ == "__main__":
    main()
