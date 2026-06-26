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
from typing import Dict, Iterable

import numpy as np
import zmq


REQUEST_SCHEMA = "factr.isaac_rmpflow.request.v1"
RESPONSE_SCHEMA = "factr.isaac_rmpflow.response.v1"
REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")
DEFAULT_CONFIG_DIR = REPO_DIR / "configs/isaac_rmpflow/maxlab_ur7e_right"
DEFAULT_ENDPOINT = "tcp://127.0.0.1:5557"
DEFAULT_SCENE_METADATA = REPO_DIR / "configs/isaac_rmpflow/maxlab_ur7e_scene.yaml"


def _finite_q(value, *, name: str) -> np.ndarray:
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError(f"{name} must be six finite numbers")
    return q


def _clip_step(q_current: np.ndarray, q_target: np.ndarray, max_step: float) -> np.ndarray:
    return q_current + np.clip(q_target - q_current, -max_step, max_step)


class PassThroughPolicy:
    def compute(
        self,
        side: str,
        q_current: np.ndarray,
        q_desired: np.ndarray,
        max_step: float,
        dt_override: float | None = None,
    ) -> tuple[np.ndarray, str, str]:
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
        self._state_q: Dict[str, np.ndarray] = {}
        self._state_qd: Dict[str, np.ndarray] = {}
        self._obstacle_handles = []

        desc = config_dir / "rmpflow/maxlab_ur7e_right_robot_description.yaml"
        urdf = config_dir / "maxlab_ur7e_right.urdf"
        cfg = config_dir / "rmpflow/maxlab_ur7e_right_rmpflow_config.yaml"
        self._robot = lula.load_robot(str(desc), str(urdf))
        self._kinematics = self._robot.kinematics()
        self._world = lula.create_world()
        self._world_view = self._world.add_world_view()
        self._rmp_cfg = lula.create_rmpflow_config(str(cfg), self._robot, "tool0", self._world_view)
        self._rmp = lula.create_rmpflow(self._rmp_cfg)

    def add_sphere_obstacle(self, center: Iterable[float], radius: float):
        center_arr = np.asarray(list(center), dtype=np.float64)
        if center_arr.shape != (3,) or not np.all(np.isfinite(center_arr)):
            raise ValueError("sphere obstacle center must be three finite numbers")
        radius = float(radius)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("sphere obstacle radius must be positive")
        obstacle = self._lula.create_obstacle(self._lula.Obstacle.Type.SPHERE)
        obstacle.set_attribute(self._lula.Obstacle.Attribute.RADIUS, radius)
        handle = self._world.add_obstacle(
            obstacle,
            self._lula.Pose3.from_translation(center_arr),
        )
        self._obstacle_handles.append(handle)
        self._world_view.update()
        self._rmp.update_world_view()
        return handle

    def set_sphere_obstacle_position(self, handle, center: Iterable[float]) -> None:
        center_arr = np.asarray(list(center), dtype=np.float64)
        if center_arr.shape != (3,) or not np.all(np.isfinite(center_arr)):
            raise ValueError("sphere obstacle center must be three finite numbers")
        self._world.set_pose(handle, self._lula.Pose3.from_translation(center_arr))

    def refresh_world(self) -> None:
        self._world_view.update()
        self._rmp.update_world_view()

    def num_enabled_obstacles(self) -> int:
        try:
            return int(self._world_view.num_enabled_obstacles())
        except Exception:
            return len(self._obstacle_handles)

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

    def tool_translation(self, q_real: np.ndarray) -> np.ndarray:
        translation, _ = self._desired_tool_pose(self._real_to_lula(q_real))
        return np.asarray(translation, dtype=np.float64)

    def compute(
        self,
        side: str,
        q_current: np.ndarray,
        q_desired: np.ndarray,
        max_step: float,
        dt_override: float | None = None,
    ) -> tuple[np.ndarray, str, str]:
        now = time.monotonic()
        q_cur = self._real_to_lula(q_current)
        q_des = self._real_to_lula(q_desired)
        last_t = self._last_t.get(side)
        if dt_override is None:
            dt = 1.0 / 100.0 if last_t is None else float(np.clip(now - last_t, 1.0 / 250.0, 1.0 / 20.0))
        else:
            dt = float(np.clip(dt_override, 1.0 / 1000.0, 1.0 / 20.0))
        measured_qd = np.zeros(6, dtype=np.float64)
        last_q = self._last_q.get(side)
        if last_q is not None:
            measured_qd = np.clip((q_cur - last_q) / dt, -2.0, 2.0)

        q_state = self._state_q.get(side)
        qd_state = self._state_qd.get(side)
        if q_state is None or qd_state is None or np.linalg.norm(q_state - q_cur, ord=np.inf) > 0.75:
            q_state = q_cur.copy()
            qd_state = measured_qd.copy()

        target_pos, target_rot = self._desired_tool_pose(q_des)
        self._rmp.set_cspace_attractor(q_des.astype(np.float64))
        self._rmp.set_end_effector_position_attractor(target_pos)
        self._rmp.set_end_effector_orientation_attractor(self._lula.Rotation3(target_rot))
        self._world_view.update()
        self._rmp.update_world_view()

        q_next = q_state.copy()
        qd_next = qd_state.copy()
        substeps = max(1, int(math.ceil(dt / self._max_substep_size)))
        h = dt / substeps
        for _ in range(substeps):
            qdd = np.zeros(6, dtype=np.float64)
            self._rmp.eval_accel(q_next, qd_next, qdd)
            q_next = q_next + h * qd_next
            qd_next = qd_next + h * qdd

        q_safe = _clip_step(q_current, self._lula_to_real(q_next), max_step)
        q_safe_lula = self._real_to_lula(q_safe)
        self._state_q[side] = q_safe_lula
        self._state_qd[side] = np.clip(qd_next, -8.0, 8.0)
        self._last_q[side] = q_cur
        self._last_t[side] = now
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


def parse_sphere_obstacles(values: Iterable[str]) -> list[tuple[np.ndarray, float]]:
    obstacles = []
    for value in values:
        parts = [part.strip() for part in value.split(",")]
        if len(parts) != 4:
            raise ValueError(f"obstacle sphere must be x,y,z,radius, got {value!r}")
        x, y, z, radius = (float(part) for part in parts)
        center = np.asarray([x, y, z], dtype=np.float64)
        if not np.all(np.isfinite(center)) or not math.isfinite(radius) or radius <= 0.0:
            raise ValueError(f"invalid obstacle sphere {value!r}")
        obstacles.append((center, radius))
    return obstacles


def add_sphere_obstacles(policy, obstacle_specs: Iterable[tuple[np.ndarray, float]]) -> int:
    count = 0
    if not hasattr(policy, "add_sphere_obstacle"):
        return count
    for center, radius in obstacle_specs:
        policy.add_sphere_obstacle(center, radius)
        count += 1
    return count


def _yaw_rotation(yaw: float) -> np.ndarray:
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.asarray(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _quat_wxyz_rotation(quat_wxyz: Iterable[float]) -> np.ndarray:
    quat = np.asarray(list(quat_wxyz), dtype=np.float64)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        raise ValueError("quat_wxyz must be four finite numbers")
    norm = float(np.linalg.norm(quat))
    if norm <= 0.0:
        raise ValueError("quat_wxyz must be nonzero")
    w, x, y, z = quat / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _quat_wxyz_multiply(left: Iterable[float], right: Iterable[float]) -> np.ndarray:
    left_q = np.asarray(list(left), dtype=np.float64)
    right_q = np.asarray(list(right), dtype=np.float64)
    if left_q.shape != (4,) or right_q.shape != (4,):
        raise ValueError("quaternion multiplication expects two wxyz quaternions")
    lw, lx, ly, lz = left_q
    rw, rx, ry, rz = right_q
    result = np.asarray(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(result))
    if norm <= 0.0:
        raise ValueError("quaternion product is zero")
    return result / norm


def _load_yaml(path: Path) -> dict:
    import yaml

    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data


def _load_collision_spheres(descriptor_path: Path) -> list[tuple[str, np.ndarray, float]]:
    descriptor = _load_yaml(descriptor_path)
    spheres = []
    for group in descriptor.get("collision_spheres", []):
        if not isinstance(group, dict):
            continue
        for link_name, link_spheres in group.items():
            for sphere in link_spheres or []:
                center = np.asarray(sphere["center"], dtype=np.float64)
                radius = float(sphere["radius"])
                if center.shape == (3,) and np.all(np.isfinite(center)) and radius > 0.0:
                    spheres.append((str(link_name), center, radius))
    return spheres


class OtherArmObstacleField:
    """Represent one observed arm as moving Lula sphere obstacles."""

    def __init__(
        self,
        policy: LulaRmpPolicy,
        *,
        controlled_side: str,
        obstacle_side: str,
        scene_metadata_path: Path = DEFAULT_SCENE_METADATA,
        descriptor_path: Path | None = None,
        obstacle_wrist_3_offset: float | None = None,
        initial_obstacle_q: Iterable[float] | None = None,
    ) -> None:
        self._policy = policy
        self._controlled_side = controlled_side
        self._obstacle_side = obstacle_side
        if obstacle_wrist_3_offset is None:
            raise ValueError("obstacle_wrist_3_offset must be provided for the obstacle side")
        self._obstacle_wrist_3_offset = float(obstacle_wrist_3_offset)
        scene = _load_yaml(scene_metadata_path)
        bases = scene.get("bases", {})
        if controlled_side not in bases or obstacle_side not in bases:
            raise ValueError(f"scene metadata missing bases for {controlled_side}/{obstacle_side}")
        self._world_from_controlled_R, self._world_from_controlled_t = self._base_transform(bases[controlled_side])
        self._world_from_obstacle_R, self._world_from_obstacle_t = self._base_transform(bases[obstacle_side])

        if descriptor_path is None:
            descriptor_path = DEFAULT_CONFIG_DIR / "rmpflow/maxlab_ur7e_right_robot_description.yaml"
        raw_spheres = _load_collision_spheres(descriptor_path)
        self._sphere_specs: list[tuple[str, np.ndarray, float]] = []
        initial_pose_map = scene.get("factr_initial_match_joint_pos_real", {})
        if initial_obstacle_q is None and obstacle_side not in initial_pose_map:
            raise ValueError(f"scene metadata missing initial pose for {obstacle_side}")
        initial_q = np.asarray(
            list(initial_obstacle_q)
            if initial_obstacle_q is not None
            else initial_pose_map[obstacle_side],
            dtype=np.float64,
        )
        _finite_q(initial_q, name=f"{obstacle_side}.initial_obstacle_q")
        initial_centers = self._compute_centers(initial_q, raw_spheres, keep_valid=True)
        self._handles = []
        for (link_name, center_link, radius), center_controlled in zip(self._sphere_specs, initial_centers):
            del link_name, center_link
            self._handles.append(policy.add_sphere_obstacle(center_controlled, radius))
        policy.refresh_world()

    def _base_transform(self, base_config: dict) -> tuple[np.ndarray, np.ndarray]:
        transform_config = base_config.get("world_from_lula_base", base_config)
        translation = np.asarray(transform_config["pos"], dtype=np.float64)
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError("base pos must be three finite numbers")

        if "quat_wxyz" in transform_config:
            rotation = _quat_wxyz_rotation(transform_config["quat_wxyz"])
            if "yaw_rad" in transform_config:
                yaw_rotation = _yaw_rotation(float(transform_config["yaw_rad"]))
                if not np.allclose(rotation, yaw_rotation, atol=1e-6, rtol=0.0):
                    raise ValueError("base yaw_rad and quat_wxyz do not describe the same rotation")
            if transform_config is not base_config and "quat_wxyz" in base_config and "mjcf_base_quat_wxyz" in base_config:
                composed_quat = _quat_wxyz_multiply(base_config["quat_wxyz"], base_config["mjcf_base_quat_wxyz"])
                composed_rotation = _quat_wxyz_rotation(composed_quat)
                if not np.allclose(rotation, composed_rotation, atol=1e-6, rtol=0.0):
                    raise ValueError("world_from_lula_base does not match mount quat composed with mjcf_base_quat_wxyz")
            return rotation, translation

        return _yaw_rotation(float(transform_config["yaw_rad"])), translation

    def _real_to_obstacle_lula(self, q: np.ndarray) -> np.ndarray:
        q_lula = np.asarray(q, dtype=np.float64).copy()
        q_lula[5] += self._obstacle_wrist_3_offset
        return q_lula

    def _compute_centers(
        self,
        q_obstacle_real: np.ndarray,
        sphere_specs: list[tuple[str, np.ndarray, float]] | None = None,
        *,
        keep_valid: bool = False,
    ) -> list[np.ndarray]:
        q_lula = self._real_to_obstacle_lula(q_obstacle_real)
        q_col = np.expand_dims(q_lula, 1)
        centers = []
        specs = self._sphere_specs if sphere_specs is None else sphere_specs
        valid_specs = []
        for link_name, center_link, radius in specs:
            try:
                pose = self._policy._kinematics.pose(q_col, link_name)
            except Exception:
                if keep_valid:
                    continue
                raise
            center_obstacle_base = np.asarray(pose.rotation.matrix(), dtype=np.float64) @ center_link
            center_obstacle_base = center_obstacle_base + np.asarray(pose.translation, dtype=np.float64)
            center_world = self._world_from_obstacle_R @ center_obstacle_base + self._world_from_obstacle_t
            center_controlled = self._world_from_controlled_R.T @ (center_world - self._world_from_controlled_t)
            centers.append(center_controlled)
            if keep_valid:
                valid_specs.append((link_name, center_link, radius))
        if keep_valid:
            self._sphere_specs = valid_specs
        return centers

    def update(self, q_obstacle_real: np.ndarray) -> None:
        centers = self._compute_centers(q_obstacle_real)
        if len(centers) != len(self._handles):
            raise RuntimeError("dynamic obstacle sphere count changed")
        for handle, center in zip(self._handles, centers):
            self._policy.set_sphere_obstacle_position(handle, center)
        self._policy.refresh_world()

    def count(self) -> int:
        return len(self._handles)

    @property
    def obstacle_side(self) -> str:
        return self._obstacle_side


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
        if side != "right":
            return _error_response(sequence, "REQ/REP RMPFlow server currently supports only right active side")
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
    parser.add_argument(
        "--obstacle-sphere",
        action="append",
        default=[],
        metavar="X,Y,Z,R",
        help="Static Lula sphere obstacle in robot base frame. Repeat for multiple obstacles.",
    )
    args = parser.parse_args()

    if args.mode == "rmp":
        policy = LulaRmpPolicy(args.config_dir.resolve(), wrist_3_offset=args.right_wrist_3_offset)
        obstacle_count = add_sphere_obstacles(policy, parse_sphere_obstacles(args.obstacle_sphere))
    else:
        policy = PassThroughPolicy()
        obstacle_count = 0

    context = zmq.Context.instance()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.endpoint)
    print(
        json.dumps(
            {
                "event": "ready",
                "endpoint": args.endpoint,
                "mode": args.mode,
                "obstacle_count": obstacle_count,
            }
        ),
        flush=True,
    )
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
