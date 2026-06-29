"""Bimanual UR7e safety node using openpi-yam's MINK/QP controller."""

from __future__ import annotations

import os
import sys
import time
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String


def _array_msg(data) -> JointState:
    msg = JointState()
    msg.position = [float(x) for x in data]
    return msg


def _bool_msg(data: bool) -> Bool:
    msg = Bool()
    msg.data = bool(data)
    return msg


def _float_msg(data: float) -> Float32:
    msg = Float32()
    msg.data = float(data)
    return msg


def _string_msg(data: str) -> String:
    msg = String()
    msg.data = data
    return msg


class UR7eCollisionMonitor(Node):
    def __init__(self) -> None:
        super().__init__("ur7e_collision_monitor")

        self.declare_parameter("i2rt_path", "/home/srianumakonda/openpi-yam-maxlab/gello/i2rt")
        self.declare_parameter(
            "scene_yaml",
            "/home/srianumakonda/openpi-yam-maxlab/gello/i2rt/i2rt/robot_models/ur7e_maxlab/scene.yaml",
        )
        self.declare_parameter("left_state_topic", "/ur/left/obs_ur_state")
        self.declare_parameter("right_state_topic", "/ur/right/obs_ur_state")
        self.declare_parameter("left_desired_topic", "/factr_teleop/left/desired_ur_pos")
        self.declare_parameter("right_desired_topic", "/factr_teleop/right/desired_ur_pos")
        self.declare_parameter("left_safe_topic", "/factr_teleop/left/safe_ur_pos")
        self.declare_parameter("right_safe_topic", "/factr_teleop/right/safe_ur_pos")
        self.declare_parameter("min_distance_topic", "/factr_teleop/collision/min_interarm_distance")
        self.declare_parameter("stop_topic", "/factr_teleop/collision/stop")
        self.declare_parameter("left_wrist_3_offset", float(np.pi / 2))
        self.declare_parameter("right_wrist_3_offset", float(np.pi))
        self.declare_parameter("left_fallback_q", "1.5700,-1.5700,1.5700,-1.5700,-1.5700,-1.5700")
        self.declare_parameter("right_fallback_q", "-1.5700,-1.5700,-1.5700,-1.5700,1.5700,0.0000")
        self.declare_parameter("stale_timeout", 0.25)
        self.declare_parameter("rate_hz", 500.0)
        self.declare_parameter("activation_mode", "interarm_distance")
        self.declare_parameter("interarm_activation_distance", 0.15)
        self.declare_parameter("interarm_release_distance", 0.18)
        self.declare_parameter("release_velocity_limits", "2.0,2.0,2.0,3.0,3.0,3.0")
        self.declare_parameter("safety_activation_clearance", 0.10)
        self.declare_parameter("active_sides", "left,right")
        self.declare_parameter("command_mode", "posture")
        self.declare_parameter("velocity_tracking_gain", 8.0)

        i2rt_path = os.path.abspath(os.path.expanduser(str(self.get_parameter("i2rt_path").value)))
        if i2rt_path not in sys.path:
            sys.path.insert(0, i2rt_path)
        from i2rt.sim.safety_controller import BimanualYAMSafetyController

        self._controller = BimanualYAMSafetyController(str(self.get_parameter("scene_yaml").value))
        self._active_sides = self._parse_active_sides(str(self.get_parameter("active_sides").value))
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._wrist_3_offsets = {
            "left": float(self.get_parameter("left_wrist_3_offset").value),
            "right": float(self.get_parameter("right_wrist_3_offset").value),
        }
        self._fallback_q = {
            side: self._real_to_sim(side, self._fallback_param(side))
            for side in ("left", "right")
        }
        self._activation_mode = str(self.get_parameter("activation_mode").value).strip().lower()
        if self._activation_mode not in ("interarm_distance", "clearance"):
            raise ValueError("activation_mode must be 'interarm_distance' or 'clearance'.")
        self._interarm_activation_distance = float(self.get_parameter("interarm_activation_distance").value)
        self._interarm_release_distance = max(
            float(self.get_parameter("interarm_release_distance").value),
            self._interarm_activation_distance,
        )
        self._release_velocity_limits = self._vector_param("release_velocity_limits")
        self._activation_clearance = float(self.get_parameter("safety_activation_clearance").value)
        self._command_mode = str(self.get_parameter("command_mode").value).strip().lower()
        if self._command_mode not in ("velocity", "posture"):
            raise ValueError("command_mode must be 'velocity' or 'posture'.")
        self._velocity_tracking_gain = float(self.get_parameter("velocity_tracking_gain").value)
        self._latest_state: Dict[str, Tuple[np.ndarray, float]] = {}
        self._latest_desired: Dict[str, Tuple[np.ndarray, float]] = {}
        self._prev_desired_for_velocity: Dict[str, Tuple[np.ndarray, float]] = {}
        self._latest_desired_velocity: Dict[str, np.ndarray] = {}
        self._qp_latched = False
        self._release_ramp = {side: False for side in ("left", "right")}
        self._last_safe_q: Dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._last_tick_t = time.monotonic()

        self.create_subscription(JointState, str(self.get_parameter("left_state_topic").value), self._left_state_cb, 10)
        self.create_subscription(JointState, str(self.get_parameter("right_state_topic").value), self._right_state_cb, 10)
        self.create_subscription(
            JointState,
            str(self.get_parameter("left_desired_topic").value),
            self._left_desired_cb,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("right_desired_topic").value),
            self._right_desired_cb,
            10,
        )

        self._left_safe_pub = self.create_publisher(JointState, str(self.get_parameter("left_safe_topic").value), 10)
        self._right_safe_pub = self.create_publisher(JointState, str(self.get_parameter("right_safe_topic").value), 10)
        self._min_distance_pub = self.create_publisher(Float32, str(self.get_parameter("min_distance_topic").value), 10)
        self._stop_pub = self.create_publisher(Bool, str(self.get_parameter("stop_topic").value), 10)
        self._pass_through_pub = self.create_publisher(Bool, "/factr_teleop/collision/pass_through", 10)
        self._reason_pub = self.create_publisher(String, "/factr_teleop/collision/reason", 10)
        self._current_clearance_pub = self.create_publisher(Float32, "/factr_teleop/collision/current_clearance", 10)
        self._desired_clearance_pub = self.create_publisher(Float32, "/factr_teleop/collision/desired_clearance", 10)
        self._current_interarm_distance_pub = self.create_publisher(
            Float32, "/factr_teleop/collision/current_interarm_distance", 10
        )
        self._desired_interarm_distance_pub = self.create_publisher(
            Float32, "/factr_teleop/collision/desired_interarm_distance", 10
        )
        self._tick_dt_pub = self.create_publisher(Float32, "/factr_teleop/collision/tick_dt", 10)
        self._safe_error_pub = {
            side: self.create_publisher(Float32, f"/factr_teleop/{side}/safe_error", 10)
            for side in ("left", "right")
        }
        self._side_reason_pub = {
            side: self.create_publisher(String, f"/factr_teleop/{side}/safety_reason", 10)
            for side in ("left", "right")
        }
        self._min_self_pub = {
            side: self.create_publisher(Float32, f"/factr_teleop/{side}/min_self_distance", 10)
            for side in ("left", "right")
        }
        self._min_world_pub = {
            side: self.create_publisher(Float32, f"/factr_teleop/{side}/min_world_distance", 10)
            for side in ("left", "right")
        }

        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"UR7e openpi-yam MINK/QP safety controller ready "
            f"(active_sides={','.join(self._active_sides)}, command_mode={self._command_mode}, "
            f"activation_mode={self._activation_mode})."
        )
        self.get_logger().info(
            "Inactive-arm fallback q (real rad): "
            f"left={self._fallback_param('left').round(4).tolist()}, "
            f"right={self._fallback_param('right').round(4).tolist()}."
        )

    def _parse_active_sides(self, value: str) -> Tuple[str, ...]:
        sides = tuple(
            side.strip()
            for side in value.split(",")
            if side.strip()
        )
        if not sides:
            raise ValueError("active_sides must contain at least one side: left, right, or left,right.")
        invalid = [side for side in sides if side not in ("left", "right")]
        if invalid:
            raise ValueError(f"Invalid active_sides entries {invalid}; expected left and/or right.")
        return tuple(dict.fromkeys(sides))

    def _fallback_param(self, side: str) -> np.ndarray:
        q = self._vector_param(f"{side}_fallback_q")
        if q.shape != (6,):
            raise ValueError(f"{side}_fallback_q must contain 6 joint values, got {q.shape}.")
        return q

    def _vector_param(self, name: str) -> np.ndarray:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            value = value.strip().strip("[]")
            value = [part.strip() for part in value.split(",") if part.strip()]
        q = np.asarray(value, dtype=np.float64)
        return q

    def _should_activate_qp(
        self,
        cur_clearance: float,
        des_clearance: float,
        cur_interarm_distance: float,
        des_interarm_distance: float,
    ) -> bool:
        if self._activation_mode == "interarm_distance":
            interarm_distance = min(cur_interarm_distance, des_interarm_distance)
            if self._qp_latched:
                self._qp_latched = interarm_distance <= self._interarm_release_distance
            else:
                self._qp_latched = interarm_distance <= self._interarm_activation_distance
            return self._qp_latched
        return min(cur_clearance, des_clearance) <= self._activation_clearance

    def _release_to_desired(self, side: str, q_des: np.ndarray, dt: float) -> Tuple[np.ndarray, bool]:
        if not self._release_ramp[side]:
            return q_des, False

        q_prev = self._last_safe_q[side]
        if q_prev is None:
            self._release_ramp[side] = False
            return q_des, False

        delta = q_des - q_prev
        max_step = self._release_velocity_limits * dt
        step = np.clip(delta, -max_step, max_step)
        q_safe = q_prev + step
        ramping = bool(np.linalg.norm(q_des - q_safe) > 1e-3)
        self._release_ramp[side] = ramping
        return q_safe, True

    def _left_state_cb(self, msg: JointState) -> None:
        self._record(self._latest_state, "left", msg, "state")

    def _right_state_cb(self, msg: JointState) -> None:
        self._record(self._latest_state, "right", msg, "state")

    def _left_desired_cb(self, msg: JointState) -> None:
        self._record(self._latest_desired, "left", msg, "desired")

    def _right_desired_cb(self, msg: JointState) -> None:
        self._record(self._latest_desired, "right", msg, "desired")

    def _record(self, store: Dict[str, Tuple[np.ndarray, float]], side: str, msg: JointState, label: str) -> None:
        if len(msg.position) < 6:
            self.get_logger().warn(f"Ignoring {side} {label} with {len(msg.position)} positions; expected 6.")
            return
        store[side] = (np.asarray(msg.position[:6], dtype=np.float64), time.monotonic())

    def _tick(self) -> None:
        now = time.monotonic()
        dt = float(np.clip(now - self._last_tick_t, 0.002, 0.05))
        self._last_tick_t = now

        stale_reasons = self._stale_reasons(now)
        if stale_reasons:
            reason = "stale:" + ",".join(stale_reasons)
            self._stop_pub.publish(_bool_msg(True))
            self._pass_through_pub.publish(_bool_msg(False))
            self._reason_pub.publish(_string_msg(reason))
            self._tick_dt_pub.publish(_float_msg(dt))
            for side in self._active_sides:
                self._side_reason_pub[side].publish(_string_msg(reason))
            return

        q_cur_left = self._get_sim_q("left", self._latest_state)
        q_cur_right = self._get_sim_q("right", self._latest_state)
        q_des_left = self._get_sim_q("left", self._latest_desired)
        q_des_right = self._get_sim_q("right", self._latest_desired)

        cur_clearance, cur_distances = self._min_clearance(q_cur_left, q_cur_right)
        des_clearance, des_distances = self._min_clearance(q_des_left, q_des_right)
        cur_interarm_distance = float(cur_distances.get("arm_inter", 1.0))
        des_interarm_distance = float(des_distances.get("arm_inter", 1.0))
        self._current_clearance_pub.publish(_float_msg(cur_clearance))
        self._desired_clearance_pub.publish(_float_msg(des_clearance))
        self._current_interarm_distance_pub.publish(_float_msg(cur_interarm_distance))
        self._desired_interarm_distance_pub.publish(_float_msg(des_interarm_distance))
        self._tick_dt_pub.publish(_float_msg(dt))

        qp_active = self._should_activate_qp(
            cur_clearance,
            des_clearance,
            cur_interarm_distance,
            des_interarm_distance,
        )
        if not qp_active:
            q_safe_left, left_release_ramp = self._release_to_desired("left", q_des_left, dt)
            q_safe_right, right_release_ramp = self._release_to_desired("right", q_des_right, dt)
            release_ramping = left_release_ramp or right_release_ramp
            min_interarm_distance = cur_interarm_distance
            intervened = False
            pass_through = not release_ramping
            side_reasons = {
                "left": "release_ramp" if left_release_ramp else "pass_through",
                "right": "release_ramp" if right_release_ramp else "pass_through",
            }
            side_warnings = {side: "" for side in ("left", "right")}
            distance_summary = self._distance_summary(cur_distances)
        else:
            for side in self._active_sides:
                self._release_ramp[side] = True
            if self._command_mode == "velocity":
                qdot_des_left = self._desired_velocity("left", q_des_left, q_cur_left)
                qdot_des_right = self._desired_velocity("right", q_des_right, q_cur_right)
                safe = self._controller.filter_bimanual_velocity(
                    qdot_des_left,
                    qdot_des_right,
                    q_cur_left,
                    q_cur_right,
                    dt,
                )
            else:
                safe = self._controller.filter_bimanual(q_des_left, q_des_right, q_cur_left, q_cur_right, dt)
            q_safe_left = safe.q_safe_left
            q_safe_right = safe.q_safe_right
            min_interarm_distance = float(safe.diag_left.min_interarm_distance)
            intervened = bool(safe.diag_left.intervened or safe.diag_right.intervened)
            pass_through = False
            side_reasons = {
                "left": self._diag_reason(safe.diag_left, q_safe_left, q_des_left),
                "right": self._diag_reason(safe.diag_right, q_safe_right, q_des_right),
            }
            side_warnings = {
                "left": ",".join(safe.diag_left.warnings),
                "right": ",".join(safe.diag_right.warnings),
            }
            distance_summary = {
                "min_self": {
                    "left": float(safe.diag_left.min_self_distance),
                    "right": float(safe.diag_right.min_self_distance),
                },
                "min_world": {
                    "left": float(safe.diag_left.min_world_distance),
                    "right": float(safe.diag_right.min_world_distance),
                },
            }

        self._last_safe_q["left"] = q_safe_left.copy()
        self._last_safe_q["right"] = q_safe_right.copy()

        if "left" in self._active_sides:
            self._left_safe_pub.publish(_array_msg(self._sim_to_real("left", q_safe_left)))
        if "right" in self._active_sides:
            self._right_safe_pub.publish(_array_msg(self._sim_to_real("right", q_safe_right)))

        self._publish_diagnostics(
            q_safe={"left": q_safe_left, "right": q_safe_right},
            q_des={"left": q_des_left, "right": q_des_right},
            min_interarm_distance=min_interarm_distance,
            pass_through=pass_through,
            intervened=intervened,
            side_reasons=side_reasons,
            side_warnings=side_warnings,
            distance_summary=distance_summary,
        )

    def _min_clearance(self, q_left: np.ndarray, q_right: np.ndarray) -> Tuple[float, Dict[str, float]]:
        full_q = self._controller._compose_full_q(q_left, q_right)
        self._controller._configuration.update(full_q)

        pair_distances = self._controller._pairwise_distances()
        cfg = self._controller.get_config()
        clearances = [
            pair_distances[pair.name] - pair.margin
            for pair in cfg.collision_pairs
        ]

        workspace = cfg.workspace_bounds
        if workspace is not None:
            workspace_distances = self._controller._workspace_distances()
            clearances.extend(
                workspace_distances[side] - workspace.margin
                for side in ("left", "right")
            )

        return min(clearances) if clearances else 1.0, pair_distances

    def _stale_reasons(self, now: float) -> List[str]:
        reasons: List[str] = []
        for store in (self._latest_state, self._latest_desired):
            label = "state" if store is self._latest_state else "desired"
            for side in self._active_sides:
                if side not in store:
                    reasons.append(f"missing_{side}_{label}")
                else:
                    age = now - store[side][1]
                    if age > self._stale_timeout:
                        reasons.append(f"old_{side}_{label}_{age:.3f}s")
        return reasons

    def _distance_summary(self, pair_distances: Dict[str, float]) -> Dict[str, Dict[str, float]]:
        min_self = {"left": 1.0, "right": 1.0}
        min_world = {"left": 1.0, "right": 1.0}
        cfg = self._controller.get_config()
        for pair in cfg.collision_pairs:
            distance = float(pair_distances[pair.name])
            if pair.name == "arm_self_left":
                min_self["left"] = min(min_self["left"], distance)
            elif pair.name == "arm_self_right":
                min_self["right"] = min(min_self["right"], distance)
            elif pair.name == "arm_world":
                min_world["left"] = min(min_world["left"], distance)
                min_world["right"] = min(min_world["right"], distance)
        return {"min_self": min_self, "min_world": min_world}

    def _diag_reason(self, diag, q_safe: np.ndarray, q_des: np.ndarray) -> str:
        reasons = list(diag.active_constraints)
        if diag.warnings:
            reasons.extend(f"warning:{warning}" for warning in diag.warnings)
        if not reasons:
            safe_error = float(np.linalg.norm(q_safe - q_des))
            reasons.append("qp_filtered" if safe_error > 1e-4 else "qp_no_change")
        return ",".join(reasons)

    def _publish_diagnostics(
        self,
        q_safe: Dict[str, np.ndarray],
        q_des: Dict[str, np.ndarray],
        min_interarm_distance: float,
        pass_through: bool,
        intervened: bool,
        side_reasons: Dict[str, str],
        side_warnings: Dict[str, str],
        distance_summary: Dict[str, Dict[str, float]],
    ) -> None:
        self._min_distance_pub.publish(_float_msg(min_interarm_distance))
        self._stop_pub.publish(_bool_msg(intervened))
        self._pass_through_pub.publish(_bool_msg(pass_through))

        reason_parts = [
            "pass_through" if pass_through else "qp",
            "stop" if intervened else "no_stop",
        ]
        for side in self._active_sides:
            reason = side_reasons[side]
            if side_warnings[side]:
                reason = f"{reason};warnings={side_warnings[side]}"
            reason_parts.append(f"{side}={reason}")
        self._reason_pub.publish(_string_msg(";".join(reason_parts)))

        for side in self._active_sides:
            q_safe_real = self._sim_to_real(side, q_safe[side])
            q_des_real = self._sim_to_real(side, q_des[side])
            safe_error = float(np.linalg.norm(q_safe_real - q_des_real))
            self._safe_error_pub[side].publish(_float_msg(safe_error))
            self._side_reason_pub[side].publish(_string_msg(side_reasons[side]))
            self._min_self_pub[side].publish(_float_msg(distance_summary["min_self"][side]))
            self._min_world_pub[side].publish(_float_msg(distance_summary["min_world"][side]))

    def _get_sim_q(self, side: str, store: Dict[str, Tuple[np.ndarray, float]]) -> np.ndarray:
        if side in self._active_sides:
            return self._real_to_sim(side, store[side][0])
        return self._fallback_q[side].copy()

    def _desired_velocity(self, side: str, q_des: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
        if side not in self._active_sides:
            return np.zeros_like(q_des)

        desired_t = self._latest_desired[side][1]
        prev = self._prev_desired_for_velocity.get(side)
        if prev is None:
            leader_velocity = np.zeros_like(q_des)
        else:
            prev_q, prev_t = prev
            dt = desired_t - prev_t
            if dt > 1e-6:
                leader_velocity = (q_des - prev_q) / dt
            else:
                leader_velocity = self._latest_desired_velocity.get(side, np.zeros_like(q_des))

        if prev is None or desired_t > prev[1]:
            self._prev_desired_for_velocity[side] = (q_des.copy(), desired_t)
            self._latest_desired_velocity[side] = leader_velocity.copy()

        return leader_velocity + self._velocity_tracking_gain * (q_des - q_cur)

    def _real_to_sim(self, side: str, q: np.ndarray) -> np.ndarray:
        out = np.asarray(q, dtype=np.float64).copy()
        out[5] += self._wrist_3_offsets[side]
        return out

    def _sim_to_real(self, side: str, q: np.ndarray) -> np.ndarray:
        out = np.asarray(q, dtype=np.float64).copy()
        out[5] -= self._wrist_3_offsets[side]
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UR7eCollisionMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if "context is not valid" not in str(exc) and "rcl_shutdown" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
