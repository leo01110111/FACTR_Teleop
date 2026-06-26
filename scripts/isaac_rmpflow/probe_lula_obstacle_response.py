#!/usr/bin/env python3
"""Probe whether Lula RMPFlow responds to an explicit sphere obstacle.

This is an offline Isaac/Lula-only diagnostic. It compares the same right-arm
rollout with and without a sphere obstacle placed at the desired tool position.
It does not connect to ROS, RTDE, or hardware.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from isaac_rmpflow_zmq_server import DEFAULT_CONFIG_DIR, LulaRmpPolicy


DEFAULT_Q_CURRENT = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]
DEFAULT_Q_DESIRED = [-1.20, -1.32, -1.78, -1.38, 1.32, 0.25]


def _q_arg(value: str) -> np.ndarray:
    parts = [float(part.strip()) for part in value.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("expected six comma-separated joint values")
    q = np.asarray(parts, dtype=np.float64)
    if not np.all(np.isfinite(q)):
        raise argparse.ArgumentTypeError("joint values must be finite")
    return q


def _rollout(
    policy: LulaRmpPolicy,
    q_current: np.ndarray,
    q_desired: np.ndarray,
    *,
    steps: int,
    dt: float,
    max_step: float,
    obstacle_center: np.ndarray,
) -> dict:
    q = q_current.copy()
    min_tool_distance = math.inf
    max_joint_step = 0.0
    for _ in range(steps):
        q_next, _, _ = policy.compute("right", q, q_desired, max_step, dt_override=dt)
        max_joint_step = max(max_joint_step, float(np.max(np.abs(q_next - q))))
        q = q_next
        tool_distance = float(np.linalg.norm(policy.tool_translation(q) - obstacle_center))
        min_tool_distance = min(min_tool_distance, tool_distance)

    return {
        "q_final": q.tolist(),
        "target_error_l2": float(np.linalg.norm(q - q_desired)),
        "target_error_inf": float(np.linalg.norm(q - q_desired, ord=np.inf)),
        "tool_distance_to_obstacle_m": float(np.linalg.norm(policy.tool_translation(q) - obstacle_center)),
        "min_tool_distance_to_obstacle_m": min_tool_distance,
        "max_joint_step_rad": max_joint_step,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--right-wrist-3-offset", type=float, default=math.pi)
    parser.add_argument("--q-current", type=_q_arg, default=np.asarray(DEFAULT_Q_CURRENT, dtype=np.float64))
    parser.add_argument("--q-desired", type=_q_arg, default=np.asarray(DEFAULT_Q_DESIRED, dtype=np.float64))
    parser.add_argument("--loop-hz", type=float, default=500.0)
    parser.add_argument("--seconds", type=float, default=1.5)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.15)
    parser.add_argument("--obstacle-radius", type=float, default=0.18)
    parser.add_argument("--effect-threshold-rad", type=float, default=0.01)
    parser.add_argument("--fail-if-no-effect", action="store_true")
    args = parser.parse_args()

    dt = 1.0 / max(float(args.loop_hz), 1.0)
    steps = max(1, int(round(float(args.seconds) / dt)))

    baseline_policy = LulaRmpPolicy(args.config_dir.resolve(), wrist_3_offset=args.right_wrist_3_offset)
    obstacle_center = baseline_policy.tool_translation(args.q_desired)

    obstacle_policy = LulaRmpPolicy(args.config_dir.resolve(), wrist_3_offset=args.right_wrist_3_offset)
    obstacle_policy.add_sphere_obstacle(obstacle_center, float(args.obstacle_radius))

    baseline = _rollout(
        baseline_policy,
        args.q_current,
        args.q_desired,
        steps=steps,
        dt=dt,
        max_step=float(args.max_joint_step_rad),
        obstacle_center=obstacle_center,
    )
    with_obstacle = _rollout(
        obstacle_policy,
        args.q_current,
        args.q_desired,
        steps=steps,
        dt=dt,
        max_step=float(args.max_joint_step_rad),
        obstacle_center=obstacle_center,
    )

    q_delta = float(np.linalg.norm(np.asarray(with_obstacle["q_final"]) - np.asarray(baseline["q_final"])))
    result = {
        "ok": q_delta >= float(args.effect_threshold_rad),
        "steps": steps,
        "dt_s": dt,
        "obstacle_center_m": obstacle_center.tolist(),
        "obstacle_radius_m": float(args.obstacle_radius),
        "obstacle_count": obstacle_policy.num_enabled_obstacles(),
        "final_q_delta_l2_rad": q_delta,
        "baseline": baseline,
        "with_obstacle": with_obstacle,
    }
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    if args.fail_if_no_effect and not result["ok"]:
        raise SystemExit("RMPFlow obstacle response was below threshold")


if __name__ == "__main__":
    main()
