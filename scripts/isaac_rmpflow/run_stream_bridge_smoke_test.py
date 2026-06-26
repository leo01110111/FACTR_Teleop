#!/usr/bin/env python3
"""Run a local fake-ROS smoke test for the streaming bridge.

This test starts the pass-through stream server and ROS stream bridge on test
ports, publishes fake right-arm observed/desired joint states, and verifies that
fresh `/factr_teleop/right/safe_ur_pos` messages are produced. It does not use
Isaac, RTDE, Dynamixels, or robot hardware.
"""

from __future__ import annotations

import argparse
import contextlib
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")


class FakeStreamNode(Node):
    def __init__(self, *, duration_s: float) -> None:
        super().__init__("fake_isaac_stream_bridge_smoke")
        self._duration_s = float(duration_s)
        self._right_state_pub = self.create_publisher(JointState, "/ur/right/obs_ur_state", 10)
        self._desired_pub = self.create_publisher(JointState, "/factr_teleop/right/desired_ur_pos", 10)
        self.safe_count = 0
        self.last_safe = None
        self.create_subscription(JointState, "/factr_teleop/right/safe_ur_pos", self._safe_cb, 10)
        self.create_timer(0.005, self._tick)

    def _safe_cb(self, msg: JointState) -> None:
        self.safe_count += 1
        self.last_safe = list(msg.position[:6])

    def _tick(self) -> None:
        right = JointState()
        right.position = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]
        desired = JointState()
        desired.position = [-1.55, -1.58, -1.56, -1.56, 1.55, 0.02]
        self._right_state_pub.publish(right)
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
    parser.add_argument("--input-endpoint", default="tcp://127.0.0.1:5598")
    parser.add_argument("--output-endpoint", default="tcp://127.0.0.1:5599")
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--min-safe-count", type=int, default=20)
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
                "publish_safe_targets:=true",
                "require_rmp_policy:=false",
            ],
            cwd=str(REPO_DIR),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(1.0)

        rclpy.init()
        node = FakeStreamNode(duration_s=args.duration_s)
        try:
            node.run()
            print(f"safe_count {node.safe_count}", flush=True)
            print(f"last_safe {node.last_safe}", flush=True)
            if node.safe_count < args.min_safe_count:
                raise SystemExit(f"expected at least {args.min_safe_count} safe targets")
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
