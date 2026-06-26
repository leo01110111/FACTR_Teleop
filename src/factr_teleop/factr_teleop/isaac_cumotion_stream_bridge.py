"""ROS bridge for the high-rate Isaac Sim 6 cuMotion RMPFlow controller."""

from __future__ import annotations

import time
from typing import Dict, Iterable, Tuple

import numpy as np
import rclpy
import zmq
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


REQUEST_SCHEMA = "factr.isaac_cumotion.request.v1"
RESPONSE_SCHEMA = "factr.isaac_cumotion.response.v1"
SIDES = ("left", "right")


def _joint_msg(q: Iterable[float]) -> JointState:
    msg = JointState()
    msg.position = [float(value) for value in q]
    return msg


def _string_msg(value: str) -> String:
    msg = String()
    msg.data = str(value)
    return msg


def _float_msg(value: float) -> Float32:
    msg = Float32()
    msg.data = float(value)
    return msg


class IsaacCuMotionStreamBridge(Node):
    def __init__(self) -> None:
        super().__init__("isaac_cumotion_stream_bridge")

        self.declare_parameter("active_sides", "right")
        self.declare_parameter("input_endpoint", "tcp://127.0.0.1:5568")
        self.declare_parameter("output_endpoint", "tcp://127.0.0.1:5569")
        self.declare_parameter("publish_hz", 500.0)
        self.declare_parameter("state_timeout_s", 0.10)
        self.declare_parameter("desired_timeout_s", 0.10)
        self.declare_parameter("safe_response_timeout_s", 0.10)
        self.declare_parameter("max_joint_step_rad", 0.05)
        self.declare_parameter("max_safe_target_distance_rad", 0.05)
        self.declare_parameter("max_sequence_lag", 2)
        self.declare_parameter("publish_safe_targets", False)
        self.declare_parameter("require_rmp_policy", True)
        self.declare_parameter("hold_stale_state", True)
        self.declare_parameter("hold_stale_desired", True)
        self.declare_parameter("left_state_topic", "/ur/left/obs_ur_state")
        self.declare_parameter("right_state_topic", "/ur/right/obs_ur_state")
        self.declare_parameter("left_desired_topic", "/factr_teleop/left/desired_ur_pos")
        self.declare_parameter("right_desired_topic", "/factr_teleop/right/desired_ur_pos")
        self.declare_parameter("left_safe_topic", "/factr_teleop/left/safe_ur_pos")
        self.declare_parameter("right_safe_topic", "/factr_teleop/right/safe_ur_pos")

        self._active_sides = self._parse_active_sides(str(self.get_parameter("active_sides").value))
        self._input_endpoint = str(self.get_parameter("input_endpoint").value)
        self._output_endpoint = str(self.get_parameter("output_endpoint").value)
        self._state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self._desired_timeout_s = float(self.get_parameter("desired_timeout_s").value)
        self._safe_response_timeout_s = float(self.get_parameter("safe_response_timeout_s").value)
        self._max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self._max_safe_target_distance_rad = float(self.get_parameter("max_safe_target_distance_rad").value)
        self._max_sequence_lag = max(int(self.get_parameter("max_sequence_lag").value), 0)
        self._publish_safe_targets = bool(self.get_parameter("publish_safe_targets").value)
        self._require_rmp_policy = bool(self.get_parameter("require_rmp_policy").value)
        self._hold_stale_state = bool(self.get_parameter("hold_stale_state").value)
        self._hold_stale_desired = bool(self.get_parameter("hold_stale_desired").value)
        self._sequence = 0
        self._last_sent_sequence = -1
        self._last_safe_sequence = -1
        self._latest_state: Dict[str, Tuple[np.ndarray, float]] = {}
        self._latest_desired: Dict[str, Tuple[np.ndarray, float]] = {}

        self.create_subscription(JointState, str(self.get_parameter("left_state_topic").value), self._left_state_cb, 10)
        self.create_subscription(
            JointState, str(self.get_parameter("right_state_topic").value), self._right_state_cb, 10
        )
        self.create_subscription(
            JointState, str(self.get_parameter("left_desired_topic").value), self._left_desired_cb, 10
        )
        self.create_subscription(
            JointState, str(self.get_parameter("right_desired_topic").value), self._right_desired_cb, 10
        )

        self._safe_pub = {
            "left": self.create_publisher(JointState, str(self.get_parameter("left_safe_topic").value), 10),
            "right": self.create_publisher(JointState, str(self.get_parameter("right_safe_topic").value), 10),
        }
        self._status_pub = self.create_publisher(String, "/factr_teleop/isaac_cumotion_stream/status", 10)
        self._reason_pub = self.create_publisher(String, "/factr_teleop/isaac_cumotion_stream/reason", 10)
        self._controller_hz_pub = self.create_publisher(Float32, "/factr_teleop/isaac_cumotion_stream/controller_hz", 10)
        self._input_age_pub = self.create_publisher(Float32, "/factr_teleop/isaac_cumotion_stream/input_age_ms", 10)
        self._safe_error_pub = {
            side: self.create_publisher(Float32, f"/factr_teleop/{side}/isaac_cumotion_safe_error", 10)
            for side in SIDES
        }

        self._zmq_context = zmq.Context.instance()
        self._input_pub = self._zmq_context.socket(zmq.PUB)
        self._input_pub.setsockopt(zmq.LINGER, 0)
        self._input_pub.setsockopt(zmq.SNDHWM, 1)
        self._input_pub.connect(self._input_endpoint)
        self._output_sub = self._zmq_context.socket(zmq.SUB)
        self._output_sub.setsockopt(zmq.LINGER, 0)
        self._output_sub.setsockopt(zmq.RCVHWM, 1)
        self._output_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        try:
            self._output_sub.setsockopt(zmq.CONFLATE, 1)
        except Exception:
            pass
        self._output_sub.connect(self._output_endpoint)

        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / publish_hz, self._tick)
        self.get_logger().info(
            "Isaac Sim 6 cuMotion streaming RMPFlow bridge ready: "
            f"input={self._input_endpoint}, output={self._output_endpoint}, "
            f"active_sides={','.join(self._active_sides)}, publish_hz={publish_hz:.1f}, "
            f"publish_safe_targets={self._publish_safe_targets}, "
            f"require_rmp_policy={self._require_rmp_policy}."
        )

    def destroy_node(self):
        self._input_pub.close(linger=0)
        self._output_sub.close(linger=0)
        return super().destroy_node()

    def _parse_active_sides(self, value: str) -> Tuple[str, ...]:
        sides = tuple(side.strip() for side in value.split(",") if side.strip())
        if not sides:
            raise ValueError("active_sides must be left, right, or left,right.")
        invalid = [side for side in sides if side not in SIDES]
        if invalid:
            raise ValueError(f"Invalid active_sides entries {invalid}; expected left and/or right.")
        return tuple(dict.fromkeys(sides))

    def _left_state_cb(self, msg: JointState) -> None:
        self._record(self._latest_state, "left", msg)

    def _right_state_cb(self, msg: JointState) -> None:
        self._record(self._latest_state, "right", msg)

    def _left_desired_cb(self, msg: JointState) -> None:
        self._record(self._latest_desired, "left", msg)

    def _right_desired_cb(self, msg: JointState) -> None:
        self._record(self._latest_desired, "right", msg)

    def _record(self, store: Dict[str, Tuple[np.ndarray, float]], side: str, msg: JointState) -> None:
        if len(msg.position) < 6:
            self._publish_status("bad_input", f"{side} message has {len(msg.position)} positions")
            return
        q = np.asarray(msg.position[:6], dtype=np.float64)
        if not np.all(np.isfinite(q)):
            self._publish_status("bad_input", f"{side} message has non-finite positions")
            return
        store[side] = (q, time.monotonic())

    def _publish_status(self, status: str, reason: str) -> None:
        self._status_pub.publish(_string_msg(status))
        self._reason_pub.publish(_string_msg(reason))

    def _build_request(self) -> dict | None:
        now_mono = time.monotonic()
        arms = {}
        observed_arms = {}
        for side in self._active_sides:
            if side not in self._latest_state:
                self._publish_status("waiting", f"missing {side} state")
                return None
            q_current, state_t = self._latest_state[side]
            state_age = now_mono - state_t
            if state_age > self._state_timeout_s:
                if not self._hold_stale_state:
                    self._publish_status("stale_input", f"{side} state age {state_age:.3f}s")
                    return None
                self._publish_status("holding_state", f"{side} state age {state_age:.3f}s; holding last state")
            if side not in self._latest_desired:
                if not self._hold_stale_desired:
                    self._publish_status("waiting", f"missing {side} desired")
                    return None
                q_desired = q_current
                desired_age = 0.0
                self._publish_status("holding_desired", f"missing {side} desired; holding current")
            else:
                q_desired, desired_t = self._latest_desired[side]
                desired_age = now_mono - desired_t
                if desired_age > self._desired_timeout_s:
                    if not self._hold_stale_desired:
                        self._publish_status("stale_input", f"{side} desired age {desired_age:.3f}s")
                        return None
                    q_desired = q_current
                    self._publish_status("holding_desired", f"{side} desired age {desired_age:.3f}s; holding current")
            arms[side] = {
                "q_current": q_current.tolist(),
                "q_desired": q_desired.tolist(),
                "state_age_s": state_age,
                "desired_age_s": desired_age,
            }
        for side, (q_current, state_t) in self._latest_state.items():
            state_age = now_mono - state_t
            if side in SIDES and (state_age <= self._state_timeout_s or self._hold_stale_state):
                observed_arms[side] = {
                    "q_current": q_current.tolist(),
                    "state_age_s": state_age,
                }
        self._sequence += 1
        return {
            "schema": REQUEST_SCHEMA,
            "sequence": self._sequence,
            "stamp": time.time(),
            "active_sides": list(self._active_sides),
            "arms": arms,
            "observed_arms": observed_arms,
            "limits": {"max_joint_step_rad": self._max_joint_step_rad},
        }

    def _tick(self) -> None:
        request = self._build_request()
        sent_request = False
        if request is not None:
            try:
                self._input_pub.send_json(request, flags=zmq.NOBLOCK)
                self._last_sent_sequence = int(request["sequence"])
                sent_request = True
            except zmq.Again:
                self._publish_status("zmq_backpressure", "stream input send would block")
            except Exception as exc:
                self._publish_status("zmq_error", str(exc))
        else:
            self._drain_latest_response()
            return

        response = self._drain_latest_response()
        if response is None:
            if sent_request:
                self._publish_status("waiting_response", "no Isaac stream response")
            return
        try:
            safe_targets, mode, reason, controller_hz, input_age_ms = self._validate_response(response)
        except ValueError as exc:
            self._publish_status("bad_response", str(exc))
            return

        self._controller_hz_pub.publish(_float_msg(controller_hz))
        self._input_age_pub.publish(_float_msg(input_age_ms))
        for side, q_safe in safe_targets.items():
            if side in self._latest_desired:
                q_desired = self._latest_desired[side][0]
                self._safe_error_pub[side].publish(_float_msg(float(np.linalg.norm(q_safe - q_desired))))
        if not self._publish_safe_targets:
            self._publish_status("shadow", f"{mode}: {reason}")
            return

        for side, q_safe in safe_targets.items():
            self._safe_pub[side].publish(_joint_msg(q_safe))
        self._publish_status(mode, reason)

    def _drain_latest_response(self) -> dict | None:
        latest = None
        while True:
            try:
                latest = self._output_sub.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                return latest
            except Exception as exc:
                self._publish_status("zmq_error", str(exc))
                return None

    def _validate_response(self, response: dict) -> tuple[Dict[str, np.ndarray], str, str, float, float]:
        if response.get("schema") != RESPONSE_SCHEMA:
            raise ValueError(f"schema mismatch: {response.get('schema')}")
        sequence = int(response.get("sequence", -1))
        if sequence < self._last_safe_sequence:
            raise ValueError(f"stale sequence {sequence} < {self._last_safe_sequence}")
        if self._last_sent_sequence < 0:
            raise ValueError("no local request has been sent yet")
        if sequence < self._last_sent_sequence - self._max_sequence_lag:
            raise ValueError(
                f"response sequence {sequence} lags latest request "
                f"{self._last_sent_sequence} by more than {self._max_sequence_lag}"
            )
        if not bool(response.get("ok", False)):
            raise ValueError(str(response.get("reason", "Isaac response ok=false")))
        policy = str(response.get("policy", ""))
        if self._publish_safe_targets and self._require_rmp_policy and policy != "rmp":
            raise ValueError(f"refusing response policy {policy or '<missing>'}; expected rmp")
        response_age = time.time() - float(response.get("stamp", 0.0))
        if response_age > self._safe_response_timeout_s:
            raise ValueError(f"safe response age {response_age:.3f}s")
        mode = str(response.get("mode", ""))
        if mode not in ("pass_through", "filtered"):
            raise ValueError(f"refusing response mode {mode}")
        arms = response.get("arms", {})
        safe_targets: Dict[str, np.ndarray] = {}
        input_ages = []
        for side in self._active_sides:
            if side not in arms:
                raise ValueError(f"missing {side} response")
            q_safe = np.asarray(arms[side].get("q_safe", []), dtype=np.float64)
            if q_safe.shape != (6,) or not np.all(np.isfinite(q_safe)):
                raise ValueError(f"{side} q_safe must be six finite numbers")
            if side in self._latest_state:
                q_current = self._latest_state[side][0]
                max_distance = min(self._max_safe_target_distance_rad, self._max_joint_step_rad)
                if np.max(np.abs(q_safe - q_current)) > max_distance:
                    raise ValueError(f"{side} q_safe too far from current state")
            input_age = float(arms[side].get("input_age_s", 0.0))
            if input_age > self._safe_response_timeout_s:
                raise ValueError(f"{side} response input age {input_age:.3f}s")
            input_ages.append(input_age)
            safe_targets[side] = q_safe
        self._last_safe_sequence = sequence
        input_age_ms = 1000.0 * max(input_ages) if input_ages else 0.0
        return (
            safe_targets,
            mode,
            str(response.get("reason", "")),
            float(response.get("controller_hz", 0.0)),
            input_age_ms,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacCuMotionStreamBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
