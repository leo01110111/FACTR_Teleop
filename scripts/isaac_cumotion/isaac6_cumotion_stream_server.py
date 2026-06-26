#!/usr/bin/env python3
"""Isaac Sim 6 / cuMotion streaming RMPFlow server for FACTR UR7e.

This intentionally lives beside, not instead of, the Isaac 5.1 Lula server in
``scripts/isaac_rmpflow``.  It speaks the same ZMQ request/response schema used
by ``isaac_rmpflow_stream_bridge.py`` so the ROS side can be reused unchanged.
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
import yaml
import zmq


REQUEST_SCHEMA = "factr.isaac_rmpflow.request.v1"
RESPONSE_SCHEMA = "factr.isaac_rmpflow.response.v1"
REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")
DEFAULT_CONFIG_DIR = REPO_DIR / "configs/isaac_cumotion/maxlab_ur7e_right"
DEFAULT_SCENE_METADATA = REPO_DIR / "configs/isaac_rmpflow/maxlab_ur7e_scene.yaml"
DEFAULT_INPUT_ENDPOINT = "tcp://127.0.0.1:5568"
DEFAULT_OUTPUT_ENDPOINT = "tcp://127.0.0.1:5569"
SIDES = ("left", "right")


def _finite_q(value, *, name: str) -> np.ndarray:
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError(f"{name} must be six finite numbers")
    return q


def _clip_step(q_current: np.ndarray, q_target: np.ndarray, max_step: float) -> np.ndarray:
    return q_current + np.clip(q_target - q_current, -max_step, max_step)


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


def _load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"expected YAML mapping in {path}")
    return data


def _yaw_rotation(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _quat_wxyz_rotation(quat: Iterable[float]) -> np.ndarray:
    w, x, y, z = np.asarray(list(quat), dtype=np.float64)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("invalid quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _load_collision_spheres(xrdf_path: Path) -> list[tuple[str, np.ndarray, float]]:
    data = _load_yaml(xrdf_path)
    world_geometry = str(data.get("world_collision", {}).get("geometry", ""))
    geometry = data.get("geometry", {})
    if world_geometry not in geometry:
        raise ValueError(f"XRDF missing world collision geometry {world_geometry!r}")
    raw_spheres = geometry[world_geometry].get("spheres", {})
    spheres = []
    for link_name, link_spheres in raw_spheres.items():
        for sphere in link_spheres:
            center = np.asarray(sphere["center"], dtype=np.float64)
            radius = float(sphere["radius"])
            if center.shape != (3,) or not np.all(np.isfinite(center)) or radius <= 0.0:
                raise ValueError(f"invalid collision sphere for {link_name}")
            spheres.append((str(link_name), center, radius))
    return spheres


def _scene_wrist_offsets(scene_metadata_path: Path) -> dict[str, float]:
    scene = _load_yaml(scene_metadata_path)
    offsets = scene.get("factr_real_to_sim_offsets", {})
    return {
        "left": float(offsets["left_wrist_3"]),
        "right": float(offsets["right_wrist_3"]),
    }


class PassThroughPolicy:
    def compute(
        self,
        side: str,
        q_current: np.ndarray,
        q_desired: np.ndarray,
        max_step: float,
        dt_override: float | None = None,
    ) -> tuple[np.ndarray, str, str]:
        del side, dt_override
        q_safe = _clip_step(q_current, q_desired, max_step)
        mode = "pass_through" if np.allclose(q_safe, q_desired, atol=1e-9, rtol=0.0) else "filtered"
        reason = "pass_through" if mode == "pass_through" else "step_clipped"
        return q_safe, mode, reason


class CuMotionRmpPolicy:
    def __init__(
        self,
        config_dir: Path,
        *,
        wrist_3_offset: float,
        maximum_substep_size: float = 1.0 / 120.0,
    ) -> None:
        import cumotion

        self._cumotion = cumotion
        self._wrist_3_offset = float(wrist_3_offset)
        self._maximum_substep_size = float(maximum_substep_size)
        self._last_q: dict[str, np.ndarray] = {}
        self._last_t: dict[str, float] = {}
        self._state_q: dict[str, np.ndarray] = {}
        self._state_qd: dict[str, np.ndarray] = {}
        self._rmp_by_side = {}
        self._obstacle_handles = []

        xrdf_text = (config_dir / "robot.xrdf").read_text(encoding="utf-8")
        urdf_text = (config_dir / "robot.urdf").read_text(encoding="utf-8")
        self._robot = cumotion.load_robot_from_memory(xrdf_text, urdf_text)
        self._kinematics = self._robot.kinematics()
        self._world = cumotion.create_world()
        self._world_view = self._world.add_world_view()
        self._tool_frame = self._robot.tool_frame_names()[0]
        self._rmp_config = cumotion.create_rmpflow_config_from_file(
            rmpflow_config_file=str(config_dir / "rmp_flow.yaml"),
            robot_description=self._robot,
            end_effector_frame=self._tool_frame,
            world_view=self._world_view,
        )

    @property
    def kinematics(self):
        return self._kinematics

    def add_sphere_obstacle(self, center: Iterable[float], radius: float):
        cumotion = self._cumotion
        center_arr = np.asarray(list(center), dtype=np.float64)
        if center_arr.shape != (3,) or not np.all(np.isfinite(center_arr)):
            raise ValueError("sphere obstacle center must be three finite numbers")
        obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.SPHERE)
        obstacle.set_attribute(cumotion.Obstacle.Attribute.RADIUS, cumotion.Obstacle.AttributeValue(float(radius)))
        handle = self._world.add_obstacle(obstacle, cumotion.Pose3.from_translation(center_arr))
        self._obstacle_handles.append(handle)
        self.refresh_world()
        return handle

    def set_sphere_obstacle_position(self, handle, center: Iterable[float]) -> None:
        center_arr = np.asarray(list(center), dtype=np.float64)
        if center_arr.shape != (3,) or not np.all(np.isfinite(center_arr)):
            raise ValueError("sphere obstacle center must be three finite numbers")
        self._world.set_pose(handle, self._cumotion.Pose3.from_translation(center_arr))

    def refresh_world(self) -> None:
        self._world_view.update()

    def _real_to_cumotion(self, q: np.ndarray) -> np.ndarray:
        q_backend = np.asarray(q, dtype=np.float64).copy()
        q_backend[5] += self._wrist_3_offset
        return q_backend

    def _cumotion_to_real(self, q: np.ndarray) -> np.ndarray:
        q_real = np.asarray(q, dtype=np.float64).copy()
        q_real[5] -= self._wrist_3_offset
        return q_real

    def _desired_tool_pose(self, q_backend: np.ndarray):
        pose = self._kinematics.pose(np.expand_dims(q_backend, 1), self._tool_frame)
        return np.asarray(pose.translation, dtype=np.float64), pose.rotation

    def _rmp_for_side(self, side: str):
        if side not in self._rmp_by_side:
            self._rmp_by_side[side] = self._cumotion.create_rmpflow(self._rmp_config)
        return self._rmp_by_side[side]

    def compute(
        self,
        side: str,
        q_current: np.ndarray,
        q_desired: np.ndarray,
        max_step: float,
        dt_override: float | None = None,
    ) -> tuple[np.ndarray, str, str]:
        now = time.monotonic()
        q_cur = self._real_to_cumotion(q_current)
        q_des = self._real_to_cumotion(q_desired)
        last_t = self._last_t.get(side)
        if dt_override is None:
            dt = 1.0 / 100.0 if last_t is None else float(np.clip(now - last_t, 1.0 / 250.0, 1.0 / 20.0))
        else:
            dt = float(np.clip(dt_override, 1.0 / 1000.0, 1.0 / 20.0))

        last_q = self._last_q.get(side)
        measured_qd = np.zeros(6, dtype=np.float64)
        if last_q is not None:
            measured_qd = np.clip((q_cur - last_q) / dt, -2.0, 2.0)

        q_state = self._state_q.get(side)
        qd_state = self._state_qd.get(side)
        if q_state is None or qd_state is None or np.linalg.norm(q_state - q_cur, ord=np.inf) > 0.75:
            q_state = q_cur.copy()
            qd_state = measured_qd.copy()

        target_pos, target_rot = self._desired_tool_pose(q_des)
        rmp = self._rmp_for_side(side)
        rmp.set_cspace_attractor(q_des)
        rmp.set_end_effector_position_attractor(target_pos)
        rmp.set_end_effector_orientation_attractor(target_rot)
        self.refresh_world()

        q_next = q_state.copy()
        qd_next = qd_state.copy()
        substeps = max(1, int(math.ceil(dt / self._maximum_substep_size)))
        h = dt / substeps
        for _ in range(substeps):
            qdd = np.zeros(6, dtype=np.float64)
            rmp.eval_accel(q_next, qd_next, qdd)
            qd_next += h * qdd
            q_next += h * qd_next

        q_safe = _clip_step(q_current, self._cumotion_to_real(q_next), max_step)
        self._state_q[side] = self._real_to_cumotion(q_safe)
        self._state_qd[side] = np.clip(qd_next, -8.0, 8.0)
        self._last_q[side] = q_cur
        self._last_t[side] = now
        return q_safe, "filtered", "cumotion_rmpflow_step"


class OtherArmObstacleField:
    def __init__(
        self,
        policy: CuMotionRmpPolicy,
        *,
        controlled_side: str,
        obstacle_side: str,
        scene_metadata_path: Path,
        xrdf_path: Path,
        obstacle_wrist_3_offset: float,
    ) -> None:
        self._policy = policy
        self._controlled_side = controlled_side
        self._obstacle_side = obstacle_side
        self._obstacle_wrist_3_offset = float(obstacle_wrist_3_offset)
        scene = _load_yaml(scene_metadata_path)
        bases = scene["bases"]
        self._world_from_controlled_R, self._world_from_controlled_t = self._base_transform(bases[controlled_side])
        self._world_from_obstacle_R, self._world_from_obstacle_t = self._base_transform(bases[obstacle_side])
        self._sphere_specs = _load_collision_spheres(xrdf_path)
        initial_q = np.asarray(scene["factr_initial_match_joint_pos_real"][obstacle_side], dtype=np.float64)
        centers = self._compute_centers(initial_q, keep_valid=True)
        self._handles = [policy.add_sphere_obstacle(center, radius) for center, (_, _, radius) in zip(centers, self._sphere_specs)]
        policy.refresh_world()

    @property
    def obstacle_side(self) -> str:
        return self._obstacle_side

    def count(self) -> int:
        return len(self._handles)

    def _base_transform(self, base_config: dict) -> tuple[np.ndarray, np.ndarray]:
        transform = base_config.get("world_from_lula_base", base_config)
        translation = np.asarray(transform["pos"], dtype=np.float64)
        if "quat_wxyz" in transform:
            return _quat_wxyz_rotation(transform["quat_wxyz"]), translation
        return _yaw_rotation(float(transform["yaw_rad"])), translation

    def _real_to_obstacle_backend(self, q: np.ndarray) -> np.ndarray:
        q_backend = np.asarray(q, dtype=np.float64).copy()
        q_backend[5] += self._obstacle_wrist_3_offset
        return q_backend

    def _compute_centers(self, q_obstacle_real: np.ndarray, *, keep_valid: bool = False) -> list[np.ndarray]:
        q_backend = self._real_to_obstacle_backend(q_obstacle_real)
        q_col = np.expand_dims(q_backend, 1)
        centers = []
        valid_specs = []
        for link_name, center_link, radius in self._sphere_specs:
            try:
                pose = self._policy.kinematics.pose(q_col, link_name)
            except Exception:
                if keep_valid:
                    continue
                raise
            center_obstacle = np.asarray(pose.rotation.matrix(), dtype=np.float64) @ center_link
            center_obstacle = center_obstacle + np.asarray(pose.translation, dtype=np.float64)
            center_world = self._world_from_obstacle_R @ center_obstacle + self._world_from_obstacle_t
            center_controlled = self._world_from_controlled_R.T @ (center_world - self._world_from_controlled_t)
            centers.append(center_controlled)
            if keep_valid:
                valid_specs.append((link_name, center_link, radius))
        if keep_valid:
            self._sphere_specs = valid_specs
        return centers

    def update(self, q_obstacle_real: np.ndarray) -> None:
        centers = self._compute_centers(q_obstacle_real)
        for handle, center in zip(self._handles, centers):
            self._policy.set_sphere_obstacle_position(handle, center)
        self._policy.refresh_world()


class CuMotionRmpPolicySet:
    def __init__(
        self,
        *,
        config_dir: Path,
        policy_sides: Iterable[str],
        scene_metadata_path: Path,
        left_wrist_3_offset: float,
        right_wrist_3_offset: float,
        dynamic_other_arm_obstacles: bool,
    ) -> None:
        offsets = {"left": left_wrist_3_offset, "right": right_wrist_3_offset}
        self._policies = {
            side: CuMotionRmpPolicy(config_dir, wrist_3_offset=offsets[side])
            for side in policy_sides
        }
        self._dynamic_fields = {}
        self._obstacle_count = 0
        if dynamic_other_arm_obstacles:
            for side, policy in self._policies.items():
                obstacle_side = _opposite_side(side)
                field = OtherArmObstacleField(
                    policy,
                    controlled_side=side,
                    obstacle_side=obstacle_side,
                    scene_metadata_path=scene_metadata_path,
                    xrdf_path=config_dir / "robot.xrdf",
                    obstacle_wrist_3_offset=offsets[obstacle_side],
                )
                self._dynamic_fields[side] = field
                self._obstacle_count += field.count()

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

    def compute(self, side: str, q_current: np.ndarray, q_desired: np.ndarray, max_step: float, dt_override=None):
        policy = self._policies.get(side)
        if policy is None:
            raise ValueError(f"no cuMotion RMPFlow policy for active side {side}")
        return policy.compute(side, q_current, q_desired, max_step, dt_override=dt_override)

    @property
    def obstacle_count(self) -> int:
        return self._obstacle_count


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


def _compute_response(policy, request: dict, *, controller_dt: float, stale_after_s: float, require_other_arm_state: bool, policy_name: str) -> dict:
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
        q_safe, mode, reason = policy.compute(side, q_current, q_desired, max_step, dt_override=controller_dt)
        response_arms[side] = {"q_safe": q_safe.tolist(), "input_age_s": request_age}
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
    parser.add_argument("--scene-metadata", type=Path, default=DEFAULT_SCENE_METADATA)
    parser.add_argument("--policy-sides", default="left,right")
    parser.add_argument("--loop-hz", type=float, default=500.0)
    parser.add_argument("--stale-input-timeout-s", type=float, default=5.0)
    parser.add_argument("--status-period-s", type=float, default=1.0)
    parser.add_argument("--dynamic-other-arm-obstacles", action="store_true")
    parser.add_argument("--require-other-arm-state", action="store_true")
    args = parser.parse_args()
    if args.require_other_arm_state and not args.dynamic_other_arm_obstacles:
        parser.error("--require-other-arm-state requires --dynamic-other-arm-obstacles")

    loop_hz = max(float(args.loop_hz), 1.0)
    controller_dt = 1.0 / loop_hz
    stale_after_s = max(float(args.stale_input_timeout_s), controller_dt)

    if args.mode == "rmp":
        offsets = _scene_wrist_offsets(args.scene_metadata.resolve())
        policy = CuMotionRmpPolicySet(
            config_dir=args.config_dir.resolve(),
            policy_sides=_parse_sides(args.policy_sides),
            scene_metadata_path=args.scene_metadata.resolve(),
            left_wrist_3_offset=offsets["left"],
            right_wrist_3_offset=offsets["right"],
            dynamic_other_arm_obstacles=bool(args.dynamic_other_arm_obstacles),
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

    print(json.dumps({
        "event": "ready",
        "backend": "isaacsim6_cumotion",
        "input_endpoint": args.input_endpoint,
        "output_endpoint": args.output_endpoint,
        "mode": args.mode,
        "loop_hz": loop_hz,
        "obstacle_count": obstacle_count,
    }), flush=True)

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
                print(json.dumps({
                    "event": "status",
                    "received": received,
                    "published": published,
                    "latest_sequence": latest_sequence,
                    "loop_hz": loop_hz,
                    "last_ok": bool(response.get("ok", False)),
                    "reason": response.get("reason", ""),
                }), flush=True)
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
