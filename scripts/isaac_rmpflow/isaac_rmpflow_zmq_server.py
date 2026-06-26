#!/usr/bin/env python3
"""ZMQ server for Isaac/Lula RMPFlow UR7e safety filtering.

Default mode is pass-through with step clipping so the transport can be tested
without changing robot behavior. `--mode rmp` attempts to use bundled Lula
RMPFlow with the generated MaxLab primitive URDF/config scaffold.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import time
from pathlib import Path
from typing import Dict

import numpy as np
import zmq


REQUEST_SCHEMA = "factr.isaac_rmpflow.request.v1"
RESPONSE_SCHEMA = "factr.isaac_rmpflow.response.v1"
REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")
DEFAULT_CONFIG_DIR = REPO_DIR / "configs/isaac_rmpflow/maxlab_ur7e_right"
DEFAULT_ENDPOINT = "tcp://127.0.0.1:5557"


def _finite_q(value, *, name: str) -> np.ndarray:
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError(f"{name} must be six finite numbers")
    return q


def _clip_step(q_current: np.ndarray, q_target: np.ndarray, max_step: float) -> np.ndarray:
    return q_current + np.clip(q_target - q_current, -max_step, max_step)


class PassThroughPolicy:
    def compute(self, side: str, q_current: np.ndarray, q_desired: np.ndarray, max_step: float) -> tuple[np.ndarray, str, str]:
        q_safe = _clip_step(q_current, q_desired, max_step)
        mode = "pass_through" if np.allclose(q_safe, q_desired, atol=1e-9, rtol=0.0) else "filtered"
        reason = "pass_through" if mode == "pass_through" else "step_clipped"
        return q_safe, mode, reason


class LulaRmpPolicy:
    def __init__(self, config_dir: Path, *, wrist_3_offset: float, max_substep_size: float = 0.00334) -> None:
        import lula

        self._lula = lula
        self._wrist_3_offset = float(wrist_3_offset)
        self._max_substep_size = float(max_substep_size)
        self._last_q: Dict[str, np.ndarray] = {}
        self._last_t: Dict[str, float] = {}

        desc = config_dir / "rmpflow/maxlab_ur7e_right_robot_description.yaml"
        urdf = config_dir / "maxlab_ur7e_right.urdf"
        cfg = config_dir / "rmpflow/maxlab_ur7e_right_rmpflow_config.yaml"
        self._robot = lula.load_robot(str(desc), str(urdf))
        self._kinematics = self._robot.kinematics()
        self._world = lula.create_world()
        self._world_view = self._world.add_world_view()
        self._rmp_cfg = lula.create_rmpflow_config(str(cfg), self._robot, "tool0", self._world_view)
        self._rmp = lula.create_rmpflow(self._rmp_cfg)

    def _real_to_lula(self, q: np.ndarray) -> np.ndarray:
        q_lula = np.asarray(q, dtype=np.float64).copy()
        q_lula[5] += self._wrist_3_offset
        return q_lula

    def _lula_to_real(self, q: np.ndarray) -> np.ndarray:
        q_real = np.asarray(q, dtype=np.float64).copy()
        q_real[5] -= self._wrist_3_offset
        return q_real

    def _desired_tool_pose(self, q_desired: np.ndarray):
        pose = self._kinematics.pose(np.expand_dims(q_desired, 1), "tool0")
        return pose.translation, pose.rotation.matrix()

    def compute(self, side: str, q_current: np.ndarray, q_desired: np.ndarray, max_step: float) -> tuple[np.ndarray, str, str]:
        now = time.monotonic()
        q_cur = self._real_to_lula(q_current)
        q_des = self._real_to_lula(q_desired)
        last_t = self._last_t.get(side)
        dt = 1.0 / 100.0 if last_t is None else float(np.clip(now - last_t, 1.0 / 250.0, 1.0 / 20.0))
        last_q = self._last_q.get(side)
        qd_cur = np.zeros(6, dtype=np.float64) if last_q is None else np.clip((q_cur - last_q) / dt, -2.0, 2.0)

        target_pos, target_rot = self._desired_tool_pose(q_des)
        self._rmp.set_cspace_attractor(q_des.astype(np.float64))
        self._rmp.set_end_effector_position_attractor(target_pos)
        self._rmp.set_end_effector_orientation_attractor(self._lula.Rotation3(target_rot))
        self._world_view.update()
        self._rmp.update_world_view()

        q_next = q_cur.copy()
        qd_next = qd_cur.copy()
        substeps = max(1, int(math.ceil(dt / self._max_substep_size)))
        h = dt / substeps
        for _ in range(substeps):
            qdd = np.zeros(6, dtype=np.float64)
            self._rmp.eval_accel(q_next, qd_next, qdd)
            qd_next = qd_next + h * qdd
            q_next = q_next + h * qd_next

        self._last_q[side] = q_cur
        self._last_t[side] = now
        q_safe = _clip_step(q_current, self._lula_to_real(q_next), max_step)
        return q_safe, "filtered", "rmpflow_step"


def _error_response(sequence: int, reason: str) -> dict:
    return {
        "schema": RESPONSE_SCHEMA,
        "sequence": sequence,
        "stamp": time.time(),
        "ok": False,
        "mode": "hold",
        "arms": {},
        "reason": reason,
    }


def _handle_request(policy, request: dict, default_max_step: float) -> dict:
    sequence = int(request.get("sequence", -1))
    if request.get("schema") != REQUEST_SCHEMA:
        return _error_response(sequence, f"bad schema {request.get('schema')}")
    max_step = float(request.get("limits", {}).get("max_joint_step_rad", default_max_step))
    max_step = float(np.clip(max_step, 1e-4, 0.5))
    response_arms = {}
    modes = set()
    reasons = set()
    for side in request.get("active_sides", []):
        arm = request.get("arms", {}).get(side)
        if arm is None:
            return _error_response(sequence, f"missing arm {side}")
        q_current = _finite_q(arm.get("q_current"), name=f"{side}.q_current")
        q_desired = _finite_q(arm.get("q_desired"), name=f"{side}.q_desired")
        q_safe, mode, reason = policy.compute(side, q_current, q_desired, max_step)
        response_arms[side] = {"q_safe": q_safe.tolist()}
        modes.add(mode)
        reasons.add(reason)
    mode = "filtered" if "filtered" in modes else "pass_through"
    return {
        "schema": RESPONSE_SCHEMA,
        "sequence": sequence,
        "stamp": time.time(),
        "ok": True,
        "mode": mode,
        "arms": response_arms,
        "reason": ",".join(sorted(reasons)) if reasons else "empty_request",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--mode", choices=["pass_through", "rmp"], default="pass_through")
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--right-wrist-3-offset", type=float, default=math.pi)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.05)
    args = parser.parse_args()

    if args.mode == "rmp":
        policy = LulaRmpPolicy(args.config_dir.resolve(), wrist_3_offset=args.right_wrist_3_offset)
    else:
        policy = PassThroughPolicy()

    context = zmq.Context.instance()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.endpoint)
    print(json.dumps({"event": "ready", "endpoint": args.endpoint, "mode": args.mode}), flush=True)
    try:
        while True:
            request = socket.recv_json()
            try:
                response = _handle_request(policy, request, args.max_joint_step_rad)
            except Exception as exc:
                response = _error_response(int(request.get("sequence", -1)), str(exc))
            socket.send_json(response)
    except KeyboardInterrupt:
        pass
    finally:
        with contextlib.suppress(Exception):
            socket.close(linger=0)


if __name__ == "__main__":
    main()
