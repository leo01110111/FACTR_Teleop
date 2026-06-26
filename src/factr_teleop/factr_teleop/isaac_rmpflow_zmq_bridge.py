"""ROS bridge between FACTR UR7e topics and an Isaac/Lula ZMQ safety server."""

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


REQUEST_SCHEMA = "factr.isaac_rmpflow.request.v1"
RESPONSE_SCHEMA = "factr.isaac_rmpflow.response.v1"
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


class IsaacRmpflowZmqBridge(Node):
    def __init__(self) -> None:
        super().__init__("isaac_rmpflow_zmq_bridge")

        self.declare_parameter("active_sides", "right")
        self.declare_parameter("isaac_endpoint", "tcp://127.0.0.1:5557")
        self.declare_parameter("request_hz", 100.0)
        self.declare_parameter("state_timeout_s", 0.10)
        self.declare_parameter("desired_timeout_s", 0.10)
        self.declare_parameter("isaac_response_timeout_s", 0.05)
        self.declare_parameter("max_joint_step_rad", 0.05)
        self.declare_parameter("publish_safe_targets", True)
        self.declare_parameter("left_state_topic", "/ur/left/obs_ur_state")
        self.declare_parameter("right_state_topic", "/ur/right/obs_ur_state")
        self.declare_parameter("left_desired_topic", "/factr_teleop/left/desired_ur_pos")
        self.declare_parameter("right_desired_topic", "/factr_teleop/right/desired_ur_pos")
        self.declare_parameter("left_safe_topic", "/factr_teleop/left/safe_ur_pos")
        self.declare_parameter("right_safe_topic", "/factr_teleop/right/safe_ur_pos")

        self._active_sides = self._parse_active_sides(str(self.get_parameter("active_sides").value))
        self._endpoint = str(self.get_parameter("isaac_endpoint").value)
        self._state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self._desired_timeout_s = float(self.get_parameter("desired_timeout_s").value)
        self._response_timeout_s = float(self.get_parameter("isaac_response_timeout_s").value)
        self._max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self._publish_safe_targets = bool(self.get_parameter("publish_safe_targets").value)
        self._sequence = 0
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
        self._status_pub = self.create_publisher(String, "/factr_teleop/isaac_rmpflow/status", 10)
        self._reason_pub = self.create_publisher(String, "/factr_teleop/isaac_rmpflow/reason", 10)
        self._roundtrip_pub = self.create_publisher(Float32, "/factr_teleop/isaac_rmpflow/roundtrip_ms", 10)
        self._safe_error_pub = {
            side: self.create_publisher(Float32, f"/factr_teleop/{side}/isaac_safe_error", 10)
            for side in SIDES
        }

        self._zmq_context = zmq.Context.instance()
        self._socket = None
        self._connect_socket()

        rate_hz = max(float(self.get_parameter("request_hz").value), 1.0)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"Isaac RMPFlow ZMQ bridge ready: endpoint={self._endpoint}, "
            f"active_sides={','.join(self._active_sides)}, publish_safe_targets={self._publish_safe_targets}."
        )

    def destroy_node(self):
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
        return super().destroy_node()

    def _connect_socket(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
        self._socket = self._zmq_context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(self._endpoint)

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
        for side in self._active_sides:
            if side not in self._latest_state or side not in self._latest_desired:
                self._publish_status("waiting", f"missing {side} state or desired")
                return None
            q_current, state_t = self._latest_state[side]
            q_desired, desired_t = self._latest_desired[side]
            state_age = now_mono - state_t
            desired_age = now_mono - desired_t
            if state_age > self._state_timeout_s:
                self._publish_status("stale_input", f"{side} state age {state_age:.3f}s")
                return None
            if desired_age > self._desired_timeout_s:
                self._publish_status("stale_input", f"{side} desired age {desired_age:.3f}s")
                return None
            arms[side] = {
                "q_current": q_current.tolist(),
                "q_desired": q_desired.tolist(),
                "state_age_s": state_age,
                "desired_age_s": desired_age,
            }
        self._sequence += 1
        return {
            "schema": REQUEST_SCHEMA,
            "sequence": self._sequence,
            "stamp": time.time(),
            "active_sides": list(self._active_sides),
            "arms": arms,
            "limits": {"max_joint_step_rad": self._max_joint_step_rad},
        }

    def _tick(self) -> None:
        request = self._build_request()
        if request is None:
            return

        start = time.perf_counter()
        try:
            self._socket.send_json(request)
            if not self._socket.poll(int(self._response_timeout_s * 1000.0)):
                self._connect_socket()
                self._publish_status("timeout", f"no Isaac response within {self._response_timeout_s:.3f}s")
                return
            response = self._socket.recv_json()
        except Exception as exc:
            self._connect_socket()
            self._publish_status("zmq_error", str(exc))
            return

        roundtrip_ms = (time.perf_counter() - start) * 1000.0
        self._roundtrip_pub.publish(_float_msg(roundtrip_ms))
        try:
            safe_targets, mode, reason = self._validate_response(request, response)
        except ValueError as exc:
            self._publish_status("bad_response", str(exc))
            return

        if not self._publish_safe_targets:
            self._publish_status("shadow", f"{mode}: {reason}")
            return

        for side, q_safe in safe_targets.items():
            self._safe_pub[side].publish(_joint_msg(q_safe))
            q_desired = np.asarray(request["arms"][side]["q_desired"], dtype=np.float64)
            self._safe_error_pub[side].publish(_float_msg(float(np.linalg.norm(q_safe - q_desired))))
        self._publish_status(mode, reason)

    def _validate_response(self, request: dict, response: dict) -> tuple[Dict[str, np.ndarray], str, str]:
        if response.get("schema") != RESPONSE_SCHEMA:
            raise ValueError(f"schema mismatch: {response.get('schema')}")
        if int(response.get("sequence", -1)) != int(request["sequence"]):
            raise ValueError(f"sequence mismatch: {response.get('sequence')} != {request['sequence']}")
        if not bool(response.get("ok", False)):
            raise ValueError(str(response.get("reason", "Isaac response ok=false")))
        mode = str(response.get("mode", ""))
        if mode not in ("pass_through", "filtered"):
            raise ValueError(f"refusing response mode {mode}")
        arms = response.get("arms", {})
        safe_targets: Dict[str, np.ndarray] = {}
        for side in self._active_sides:
            if side not in arms:
                raise ValueError(f"missing {side} response")
            q_safe = np.asarray(arms[side].get("q_safe", []), dtype=np.float64)
            if q_safe.shape != (6,) or not np.all(np.isfinite(q_safe)):
                raise ValueError(f"{side} q_safe must be six finite numbers")
            q_current = np.asarray(request["arms"][side]["q_current"], dtype=np.float64)
            if np.max(np.abs(q_safe - q_current)) > max(self._max_joint_step_rad, 1e-6) + 1e-6:
                raise ValueError(f"{side} q_safe exceeds max_joint_step_rad from current")
            safe_targets[side] = q_safe
        return safe_targets, mode, str(response.get("reason", ""))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacRmpflowZmqBridge()
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
