#!/usr/bin/env python3
"""High-rate streaming Isaac/Lula RMPFlow controller.

This server is closer to the original FACTR control structure than the REQ/REP
prototype: ROS streams the latest observed and desired joint targets in, this
process runs its own fixed-rate RMPFlow loop, and it publishes the latest safe
joint target back out.  There is no per-tick request/response round trip.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import time
from pathlib import Path
from typing import Iterable

import numpy as np
import zmq

from isaac_rmpflow_zmq_server import (
    DEFAULT_CONFIG_DIR,
    DEFAULT_SCENE_METADATA,
    OtherArmObstacleField,
    PassThroughPolicy,
    LulaRmpPolicy,
    REQUEST_SCHEMA,
    RESPONSE_SCHEMA,
    add_sphere_obstacles,
    _finite_q,
    _load_yaml,
    parse_sphere_obstacles,
)


DEFAULT_INPUT_ENDPOINT = "tcp://127.0.0.1:5558"
DEFAULT_OUTPUT_ENDPOINT = "tcp://127.0.0.1:5559"
SIDES = ("left", "right")


def _parse_sides(value: str) -> tuple[str, ...]:
    sides = tuple(side.strip() for side in value.split(",") if side.strip())
    if not sides:
        raise ValueError("expected at least one side")
    invalid = [side for side in sides if side not in SIDES]
    if invalid:
        raise ValueError(f"invalid side(s) {invalid}; expected left and/or right")
    return tuple(dict.fromkeys(sides))


def _opposite_side(side: str) -> str:
    if side == "left":
        return "right"
    if side == "right":
        return "left"
    raise ValueError(f"invalid side {side}")


def _scene_wrist_offsets(scene_metadata_path: Path) -> dict[str, float]:
    scene = _load_yaml(scene_metadata_path)
    offsets = scene.get("factr_real_to_sim_offsets", {})
    try:
        return {
            "left": float(offsets["left_wrist_3"]),
            "right": float(offsets["right_wrist_3"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"scene metadata missing factr_real_to_sim_offsets: {scene_metadata_path}") from exc


class LulaRmpPolicySet:
    """Own one Lula RMPFlow policy per controlled arm side."""

    def __init__(
        self,
        *,
        config_dir: Path,
        policy_sides: Iterable[str],
        left_wrist_3_offset: float,
        right_wrist_3_offset: float,
        static_obstacles: Iterable[tuple[np.ndarray, float]],
        dynamic_other_arm_obstacles: bool,
        scene_metadata_path: Path,
    ) -> None:
        self._policies = {}
        self._dynamic_fields = {}
        self._obstacle_count = 0
        offsets = {
            "left": float(left_wrist_3_offset),
            "right": float(right_wrist_3_offset),
        }
        for side in policy_sides:
            policy = LulaRmpPolicy(config_dir.resolve(), wrist_3_offset=offsets[side])
            self._obstacle_count += add_sphere_obstacles(policy, static_obstacles)
            if dynamic_other_arm_obstacles:
                obstacle_side = _opposite_side(side)
                field = OtherArmObstacleField(
                    policy,
                    controlled_side=side,
                    obstacle_side=obstacle_side,
                    scene_metadata_path=scene_metadata_path.resolve(),
                    descriptor_path=config_dir.resolve() / "rmpflow/maxlab_ur7e_right_robot_description.yaml",
                    obstacle_wrist_3_offset=offsets[obstacle_side],
                )
                self._dynamic_fields[side] = field
                self._obstacle_count += field.count()
            self._policies[side] = policy

    def update_dynamic_obstacles(self, request: dict, *, require_other_arm_state: bool) -> tuple[bool, str]:
        observed_arms = request.get("observed_arms", {})
        for controlled_side, field in self._dynamic_fields.items():
            obstacle_side = field.obstacle_side
            obstacle_arm = observed_arms.get(obstacle_side)
            if obstacle_arm is None:
                if require_other_arm_state:
                    return False, f"missing observed {obstacle_side} arm for {controlled_side} dynamic obstacles"
                continue
            field.update(_finite_q(obstacle_arm.get("q_current"), name=f"{obstacle_side}.q_current"))
        return True, ""

    def compute(
        self,
        side: str,
        q_current: np.ndarray,
        q_desired: np.ndarray,
        max_step: float,
        dt_override: float | None = None,
    ) -> tuple[np.ndarray, str, str]:
        policy = self._policies.get(side)
        if policy is None:
            raise ValueError(f"no RMPFlow policy was created for active side {side}")
        return policy.compute(side, q_current, q_desired, max_step, dt_override=dt_override)

    @property
    def obstacle_count(self) -> int:
        return self._obstacle_count


def _policy_name(policy) -> str:
    if isinstance(policy, PassThroughPolicy):
        return "pass_through"
    return str(getattr(policy, "policy_name", "rmp"))


def _error_response(sequence: int, reason: str, *, policy: str = "unknown") -> dict:
    return {
        "schema": RESPONSE_SCHEMA,
        "sequence": sequence,
        "stamp": time.time(),
        "ok": False,
        "policy": policy,
        "mode": "hold",
        "arms": {},
        "reason": reason,
    }


def _drain_latest(socket) -> tuple[dict | None, int]:
    latest = None
    count = 0
    while True:
        try:
            latest = socket.recv_json(flags=zmq.NOBLOCK)
            count += 1
        except zmq.Again:
            return latest, count


def _validate_request(request: dict) -> tuple[int, tuple[str, ...], dict, float]:
    sequence = int(request.get("sequence", -1))
    if request.get("schema") != REQUEST_SCHEMA:
        raise ValueError(f"bad schema {request.get('schema')}")
    active_sides = tuple(request.get("active_sides", []))
    arms = request.get("arms", {})
    max_step = float(request.get("limits", {}).get("max_joint_step_rad", 0.05))
    max_step = float(np.clip(max_step, 1e-4, 0.5))
    if not active_sides:
        raise ValueError("active_sides is empty")
    for side in active_sides:
        arm = arms.get(side)
        if arm is None:
            raise ValueError(f"missing arm {side}")
        _finite_q(arm.get("q_current"), name=f"{side}.q_current")
        _finite_q(arm.get("q_desired"), name=f"{side}.q_desired")
    return sequence, active_sides, arms, max_step


def _compute_response(
    policy,
    request: dict,
    *,
    controller_dt: float,
    stale_after_s: float,
    require_other_arm_state: bool = False,
    policy_name: str | None = None,
) -> dict:
    policy_name = _policy_name(policy) if policy_name is None else str(policy_name)
    sequence, active_sides, arms, max_step = _validate_request(request)
    request_age = time.time() - float(request.get("stamp", time.time()))
    if request_age > stale_after_s:
        return _error_response(sequence, f"input age {request_age:.3f}s", policy=policy_name)

    if hasattr(policy, "update_dynamic_obstacles"):
        ok, reason = policy.update_dynamic_obstacles(request, require_other_arm_state=require_other_arm_state)
        if not ok:
            return _error_response(sequence, reason, policy=policy_name)

    response_arms = {}
    modes = set()
    reasons = set()
    for side in active_sides:
        arm = arms[side]
        q_current = _finite_q(arm.get("q_current"), name=f"{side}.q_current")
        q_desired = _finite_q(arm.get("q_desired"), name=f"{side}.q_desired")
        q_safe, mode, reason = policy.compute(
            side,
            q_current,
            q_desired,
            max_step,
            dt_override=controller_dt,
        )
        response_arms[side] = {
            "q_safe": q_safe.tolist(),
            "input_age_s": request_age,
        }
        modes.add(mode)
        reasons.add(reason)

    return {
        "schema": RESPONSE_SCHEMA,
        "sequence": sequence,
        "stamp": time.time(),
        "ok": True,
        "policy": policy_name,
        "mode": "filtered" if "filtered" in modes else "pass_through",
        "arms": response_arms,
        "reason": ",".join(sorted(reasons)),
        "controller_dt_s": controller_dt,
        "controller_hz": 1.0 / controller_dt,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-endpoint", default=DEFAULT_INPUT_ENDPOINT)
    parser.add_argument("--output-endpoint", default=DEFAULT_OUTPUT_ENDPOINT)
    parser.add_argument("--mode", choices=["pass_through", "rmp"], default="pass_through")
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG_DIR)
    parser.add_argument(
        "--policy-sides",
        default="left,right",
        help="Comma-separated controlled sides to instantiate RMPFlow policies for.",
    )
    parser.add_argument(
        "--right-wrist-3-offset",
        type=float,
        default=None,
        help="Override right wrist_3 real-to-Lula offset. Default comes from scene metadata.",
    )
    parser.add_argument("--loop-hz", type=float, default=500.0)
    parser.add_argument("--stale-input-timeout-s", type=float, default=0.10)
    parser.add_argument("--status-period-s", type=float, default=1.0)
    parser.add_argument("--dynamic-other-arm-obstacles", action="store_true")
    parser.add_argument("--scene-metadata", type=Path, default=DEFAULT_SCENE_METADATA)
    parser.add_argument(
        "--left-wrist-3-offset",
        type=float,
        default=None,
        help="Override left wrist_3 real-to-Lula offset. Default comes from scene metadata.",
    )
    parser.add_argument("--require-other-arm-state", action="store_true")
    parser.add_argument(
        "--obstacle-sphere",
        action="append",
        default=[],
        metavar="X,Y,Z,R",
        help="Static Lula sphere obstacle in robot base frame. Repeat for multiple obstacles.",
    )
    args = parser.parse_args()
    if args.require_other_arm_state and not args.dynamic_other_arm_obstacles:
        parser.error("--require-other-arm-state requires --dynamic-other-arm-obstacles")

    loop_hz = max(float(args.loop_hz), 1.0)
    controller_dt = 1.0 / loop_hz
    stale_after_s = max(float(args.stale_input_timeout_s), controller_dt)
    scene_wrist_offsets = _scene_wrist_offsets(args.scene_metadata.resolve())
    left_wrist_3_offset = (
        scene_wrist_offsets["left"] if args.left_wrist_3_offset is None else float(args.left_wrist_3_offset)
    )
    right_wrist_3_offset = (
        scene_wrist_offsets["right"] if args.right_wrist_3_offset is None else float(args.right_wrist_3_offset)
    )

    if args.mode == "rmp":
        policy = LulaRmpPolicySet(
            config_dir=args.config_dir.resolve(),
            policy_sides=_parse_sides(args.policy_sides),
            left_wrist_3_offset=left_wrist_3_offset,
            right_wrist_3_offset=right_wrist_3_offset,
            static_obstacles=parse_sphere_obstacles(args.obstacle_sphere),
            dynamic_other_arm_obstacles=bool(args.dynamic_other_arm_obstacles),
            scene_metadata_path=args.scene_metadata.resolve(),
        )
        obstacle_count = policy.obstacle_count
    else:
        policy = PassThroughPolicy()
        obstacle_count = 0

    context = zmq.Context.instance()
    input_socket = context.socket(zmq.SUB)
    input_socket.setsockopt(zmq.LINGER, 0)
    input_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    input_socket.bind(args.input_endpoint)
    output_socket = context.socket(zmq.PUB)
    output_socket.setsockopt(zmq.LINGER, 0)
    output_socket.setsockopt(zmq.SNDHWM, 1)
    output_socket.bind(args.output_endpoint)

    print(
        json.dumps(
            {
                "event": "ready",
                "input_endpoint": args.input_endpoint,
                "output_endpoint": args.output_endpoint,
                "mode": args.mode,
                "loop_hz": loop_hz,
                "obstacle_count": obstacle_count,
                "left_wrist_3_offset": left_wrist_3_offset,
                "right_wrist_3_offset": right_wrist_3_offset,
            }
        ),
        flush=True,
    )

    latest_request = None
    latest_sequence = -1
    last_status_t = time.monotonic()
    next_tick_t = time.monotonic()
    published = 0
    received = 0
    try:
        while True:
            request, count = _drain_latest(input_socket)
            if request is not None:
                latest_request = request
                latest_sequence = int(request.get("sequence", -1))
                received += count

            now = time.monotonic()
            if now < next_tick_t:
                time.sleep(min(next_tick_t - now, controller_dt))
                continue
            next_tick_t += controller_dt
            if next_tick_t < now - controller_dt:
                next_tick_t = now + controller_dt

            if latest_request is None:
                response = _error_response(-1, "waiting for streamed input", policy=args.mode)
            else:
                try:
                    response = _compute_response(
                        policy,
                        latest_request,
                        controller_dt=controller_dt,
                        stale_after_s=stale_after_s,
                        require_other_arm_state=args.require_other_arm_state,
                        policy_name=args.mode,
                    )
                except Exception as exc:
                    response = _error_response(latest_sequence, str(exc), policy=args.mode)
            try:
                output_socket.send_json(response, flags=zmq.NOBLOCK)
                published += 1
            except zmq.Again:
                pass

            now = time.monotonic()
            if now - last_status_t >= args.status_period_s:
                print(
                    json.dumps(
                        {
                            "event": "status",
                            "received": received,
                            "published": published,
                            "latest_sequence": latest_sequence,
                            "loop_hz": loop_hz,
                            "last_ok": bool(response.get("ok", False)),
                            "reason": response.get("reason", ""),
                        }
                    ),
                    flush=True,
                )
                received = 0
                published = 0
                last_status_t = now
    except KeyboardInterrupt:
        pass
    finally:
        with contextlib.suppress(Exception):
            input_socket.close(linger=0)
        with contextlib.suppress(Exception):
            output_socket.close(linger=0)


if __name__ == "__main__":
    main()
