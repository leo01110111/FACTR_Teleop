# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------
#
# UR7e ROLLOUT / INFERENCE DRIVE NODE.
#
# During teleoperation, `factr_teleop_ur7e.py` reads the GELLO leader and drives
# the follower UR7e: it PUBLISHES `/factr_teleop/<name>/cmd_ur_pos` (6 joints) and
# `/factr_teleop/<name>/cmd_gripper_pos` (1 Robotiq 0..255 value) purely for
# logging, while the SERVO happens inside that same node from the leader pose.
#
# At policy-rollout time there is no GELLO leader: `bc/policy_rollout.py` publishes
# the SAME two command topics from the trained policy. Nothing, however, subscribes
# to them to actually move the arm (the Franka pipeline had `bc/franka_bridge.py`
# for this; the UR7e pipeline had no equivalent). This node fills that gap.
#
# It reuses the EXACT servo/command mechanism from factr_teleop_ur7e.py -- the
# ur_rtde directTorque joint-space PD loop (`pd_joint_torque` + `directTorque`) and
# the Robotiq URCap socket client (`RobotiqGripper`) -- but is driven by the two
# command topics instead of the Dynamixel leader. It also republishes the same
# observation topics the policy consumes (`obs_ur_state`, `obs_ur_wrench`,
# `obs_gripper`) so the rollout node sees observations without the teleop node.
#
# It deliberately does NOT import/instantiate the Dynamixel / pinocchio leader
# stack: only the follower half of the teleop node is needed here.

import os
import threading
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

# Reuse the follower servo + gripper primitives verbatim from the teleop node so
# the arm is commanded through exactly the same path it was during data collection.
from factr_teleop.factr_teleop_ur7e import (
    RobotiqGripper,
    create_array_msg,
    pd_joint_torque,
)
from python_utils.global_configs import (
    ur_left_real_addresses,
    ur_right_real_addresses,
)


def _workspace_root():
    """Locate the colcon workspace root that holds src/factr_teleop/.../configs."""
    colcon_prefix = os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep)[0]
    if colcon_prefix:
        root = Path(colcon_prefix).resolve().parent
        if (root / "src/factr_teleop/factr_teleop/configs").exists():
            return root
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/factr_teleop/factr_teleop/configs").exists():
            return parent
    return Path.cwd()


def _load_config(config_file):
    config_path = Path(config_file)
    if not config_path.is_file():
        config_path = (
            _workspace_root()
            / "src/factr_teleop/factr_teleop/configs"
            / config_file
        )
    if not config_path.is_file():
        raise FileNotFoundError(f"Could not find config file: {config_file}")
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


class FACTRTeleopUR7eRollout(Node):
    """
    Drives the follower UR7e from policy command topics during rollout.

    Subscribes:
        /factr_teleop/<name>/cmd_ur_pos       (JointState, 6 joint positions [rad])
        /factr_teleop/<name>/cmd_gripper_pos  (JointState, 1 Robotiq position 0..255)

    Publishes (same as the teleop node, so the policy sees observations):
        /ur/<name>/obs_ur_state    (JointState, 6 actual joint positions [rad])
        /ur/<name>/obs_ur_wrench   (JointState, 6-dim actual TCP wrench)
        /ur/<name>/obs_gripper     (JointState, [pos255, current255] Robotiq 0..255)
    """

    def __init__(self):
        super().__init__("factr_teleop_ur7e_rollout")

        config_file = self.declare_parameter(
            "config_file", "ur7e_leader_left.yaml"
        ).get_parameter_value().string_value
        self.config = _load_config(config_file)
        self.name = self.config["name"]

        self.num_arm_joints = int(self.config["arm_teleop"]["num_arm_joints"])

        # ---- follower servo gains (direct-torque PD) -- identical source to teleop
        torque_cfg = self.config["controller"]["torque"]

        def _required_gain(key):
            gain = np.asarray(torque_cfg[key], dtype=np.float64)
            if gain.shape != (self.num_arm_joints,):
                raise ValueError(
                    f"controller.torque.{key} must have {self.num_arm_joints} "
                    f"values, got {gain.shape[0]}."
                )
            return gain

        self.pd_kp = _required_gain("kp")
        self.pd_kd = _required_gain("kd")
        self.pd_tau_max = _required_gain("tau_max")
        self.pd_friction_comp = bool(torque_cfg.get("friction_comp", True))

        frequency = float(self.config["controller"]["frequency"])
        servo_cfg = self.config["arm_teleop"].get("servo", {})
        self.servo_command_hz = float(servo_cfg.get("command_hz", frequency))
        self.observation_hz = float(
            servo_cfg.get("observation_hz", self.servo_command_hz)
        )
        self.watchdog_min_frequency = float(
            servo_cfg.get("watchdog_min_frequency", 20.0)
        )
        self.max_joint_speed = float(servo_cfg.get("max_joint_speed", 0.5))
        if self.max_joint_speed <= 0.0:
            raise ValueError(
                f"arm_teleop.servo.max_joint_speed must be > 0 rad/s, "
                f"got {self.max_joint_speed}."
            )

        self.initial_match_joint_pos = np.array(
            self.config["arm_teleop"]["initialization"]["initial_match_joint_pos"],
            dtype=np.float64,
        )[: self.num_arm_joints]

        # Hold the follower's pose until a fresh policy command has been received;
        # never move toward a stale/absent target. `command_timeout` guards how old
        # a cmd_ur_pos may be before we stop tracking it and just hold position.
        self.command_timeout = float(
            self.declare_parameter("command_timeout", 0.5)
            .get_parameter_value().double_value
        )

        # ---- addresses
        if self.name == "left":
            addr = ur_left_real_addresses
        elif self.name == "right":
            addr = ur_right_real_addresses
        else:
            raise ValueError(
                f"Invalid robot name '{self.name}'. Expected 'left' or 'right'."
            )
        self.robot_ip = addr["ip"]
        self.ur_cap_port = addr["ur_cap_port"]
        self.gripper_port = addr["gripper_port"]

        # ---- shared state
        self._servo_lock = threading.Lock()
        self._target_q = None            # latest commanded joint target (6,)
        self._target_q_t = 0.0           # monotonic time the target was received
        self._servo_target_q = None      # rate-limited PD target (6,)
        self._latest_ur_q = None         # cached actual q (for obs)
        self._latest_wrench = np.zeros(6, dtype=np.float64)
        self._first_cmd_logged = False
        self._last_hold_log_t = 0.0

        self._connect_rtde(frequency)
        self._safety_start_check()

        # ---- ROS I/O
        self.obs_ur_state_pub = self.create_publisher(
            JointState, f"/ur/{self.name}/obs_ur_state", 10
        )
        self.obs_ur_wrench_pub = self.create_publisher(
            JointState, f"/ur/{self.name}/obs_ur_wrench", 10
        )
        self.obs_gripper_pub = self.create_publisher(
            JointState, f"/ur/{self.name}/obs_gripper", 10
        )
        self.servo_hz_pub = self.create_publisher(
            Float32, f"/factr_teleop/{self.name}/servo_hz", 10
        )
        self.create_subscription(
            JointState,
            f"/factr_teleop/{self.name}/cmd_ur_pos",
            self._cmd_ur_pos_cb,
            10,
        )
        self.create_subscription(
            JointState,
            f"/factr_teleop/{self.name}/cmd_gripper_pos",
            self._cmd_gripper_pos_cb,
            10,
        )

        # ---- gripper
        gripper_cfg = self.config["gripper_teleop"]
        self.enable_follower_gripper = gripper_cfg.get("enable_follower_gripper", True)
        self.gripper = None
        self._gripper_thread_running = False
        self._desired_gripper_255 = 0
        self._follower_gripper_pos_255 = 0
        self._follower_gripper_current_255 = 0
        if self.enable_follower_gripper:
            try:
                self.gripper = RobotiqGripper(self.robot_ip, self.gripper_port)
                self.gripper.activate(
                    speed=gripper_cfg.get("speed", 255),
                    force=gripper_cfg.get("force", 128),
                )
                # Seed the desired position with the gripper's current opening so
                # the first io cycle does not command a jump before the policy has
                # published a gripper target.
                self._desired_gripper_255 = self.gripper.get_position()
                self._gripper_thread_running = True
                self._gripper_thread = threading.Thread(
                    target=self._gripper_io_loop, daemon=True
                )
                self._gripper_thread.start()
                self.get_logger().info(
                    f"FACTR UR7e rollout {self.name}: Robotiq 2F-85 connected."
                )
            except Exception as e:
                self.get_logger().warn(
                    f"FACTR UR7e rollout {self.name}: gripper init failed ({e}). "
                    f"Continuing without follower gripper."
                )
                self.gripper = None

        # ---- start servo + observation threads
        q0, wrench0 = self._read_robot_observation()
        with self._servo_lock:
            self._latest_ur_q = q0.copy()
            self._latest_wrench = wrench0.copy()
            self._servo_target_q = q0.copy()

        self._servo_thread_running = True
        self._servo_thread = threading.Thread(target=self._servo_loop, daemon=True)
        self._servo_thread.start()
        self.get_logger().info(
            f"FACTR UR7e rollout {self.name}: direct-torque servo thread running at "
            f"{self.servo_command_hz:.1f} Hz with max_joint_speed="
            f"{self.max_joint_speed:.3f} rad/s. Holding pose until first cmd_ur_pos."
        )

        self._obs_thread_running = True
        self._obs_thread = threading.Thread(target=self._observation_loop, daemon=True)
        self._obs_thread.start()

    # ------------------------------------------------------------------ RTDE
    def _connect_rtde(self, frequency):
        self.get_logger().info(
            f"FACTR UR7e rollout {self.name}: connecting RTDE to {self.robot_ip} "
            f"(ur_cap_port={self.ur_cap_port}). Press PLAY on ExternalControl."
        )
        self.rtde_r = RTDEReceiveInterface(self.robot_ip)
        self.rtde_c = RTDEControlInterface(
            self.robot_ip, frequency,
            RTDEControlInterface.FLAG_USE_EXT_UR_CAP, self.ur_cap_port,
        )
        self.rtde_c.setWatchdog(self.watchdog_min_frequency)
        self.rtde_c.zeroFtSensor()
        self.get_logger().info(
            f"FACTR UR7e rollout {self.name}: RTDE connected "
            f"({self.rtde_c.isConnected()})."
        )

    def _safety_start_check(self):
        # Same fail-safe as the teleop node: refuse to start if the UR is not near
        # the shared start reference, so the first directTorque command cannot make
        # the arm jump. Jog the UR with return_ur_to_initial_match.py first.
        follower_q = np.array(self.rtde_r.getActualQ(), dtype=np.float64)[
            : self.num_arm_joints
        ]
        per_joint_err = np.abs(follower_q - self.initial_match_joint_pos)
        if np.any(per_joint_err > 0.5):
            try:
                self.rtde_c.stopScript()
                self.rtde_c.disconnect()
                self.rtde_r.disconnect()
            except Exception:
                pass
            raise RuntimeError(
                f"FACTR UR7e rollout {self.name}: follower start config differs from "
                f"initial_match_joint_pos per-joint by "
                f"{[round(float(e), 3) for e in per_joint_err]} rad (limit 0.5). "
                f"Refusing to start to avoid a jump. Jog the UR to "
                f"initial_match_joint_pos = "
                f"{[round(float(x), 4) for x in self.initial_match_joint_pos]} first "
                f"(e.g. run return_ur_to_initial_match)."
            )

    def _read_robot_observation(self):
        q = np.array(self.rtde_r.getActualQ(), dtype=np.float64)[: self.num_arm_joints]
        wrench = np.array(self.rtde_r.getActualTCPForce(), dtype=np.float64)
        return q, wrench

    # -------------------------------------------------------------- callbacks
    def _cmd_ur_pos_cb(self, msg):
        if len(msg.position) < self.num_arm_joints:
            self.get_logger().warn(
                f"Ignoring cmd_ur_pos with {len(msg.position)} positions; "
                f"expected {self.num_arm_joints}."
            )
            return
        with self._servo_lock:
            self._target_q = np.asarray(
                msg.position[: self.num_arm_joints], dtype=np.float64
            )
            self._target_q_t = time.monotonic()
        if not self._first_cmd_logged:
            self._first_cmd_logged = True
            self.get_logger().info(
                f"FACTR UR7e rollout {self.name}: first cmd_ur_pos received; "
                f"tracking policy."
            )

    def _cmd_gripper_pos_cb(self, msg):
        if len(msg.position) < 1:
            return
        # The policy commands a single raw Robotiq position (0 = open, 255 = closed),
        # matching how cmd_gripper_pos / obs_gripper[0] were logged during teleop.
        self._desired_gripper_255 = int(np.clip(round(msg.position[0]), 0, 255))

    # ------------------------------------------------------------- servo loop
    def _servo_loop(self):
        period = 1.0 / max(self.servo_command_hz, 1.0)
        next_tick = time.perf_counter()
        count = 0
        last_status_t = time.monotonic()
        while self._servo_thread_running:
            now_mono = time.monotonic()
            with self._servo_lock:
                fresh = (
                    self._target_q is not None
                    and now_mono - self._target_q_t <= self.command_timeout
                )
                if fresh:
                    target_q = self._target_q.copy()
                elif self._latest_ur_q is not None:
                    # No fresh policy command: hold the current pose. directTorque is
                    # still issued every cycle (UR watchdog heartbeat), so the arm
                    # stays actively held rather than going limp or drifting.
                    target_q = self._latest_ur_q.copy()
                else:
                    target_q = None

            should_log_hold = (
                not fresh
                and self._first_cmd_logged
                and now_mono - self._last_hold_log_t >= 1.0
            )
            if should_log_hold:
                self.get_logger().warn(
                    f"FACTR UR7e rollout {self.name}: cmd_ur_pos stale (> "
                    f"{self.command_timeout:.2f}s); holding pose."
                )
                self._last_hold_log_t = now_mono

            if target_q is not None:
                try:
                    q = np.asarray(self.rtde_r.getActualQ(), dtype=np.float64)[
                        : self.num_arm_joints
                    ]
                    dq = np.asarray(self.rtde_r.getActualQd(), dtype=np.float64)[
                        : self.num_arm_joints
                    ]
                    if fresh:
                        max_step = self.max_joint_speed * period
                        with self._servo_lock:
                            if self._servo_target_q is None:
                                self._servo_target_q = q.copy()
                            step = np.clip(
                                target_q - self._servo_target_q,
                                -max_step,
                                max_step,
                            )
                            self._servo_target_q = self._servo_target_q + step
                            servo_target_q = self._servo_target_q.copy()
                    else:
                        # No fresh policy command: anchor the PD target at the
                        # measured pose so a stale setpoint cannot keep pulling.
                        with self._servo_lock:
                            self._servo_target_q = q.copy()
                            servo_target_q = self._servo_target_q.copy()

                    tau = pd_joint_torque(
                        servo_target_q, q, dq, self.pd_kp, self.pd_kd,
                        self.pd_tau_max
                    )
                    self.rtde_c.directTorque(
                        tau.tolist(), friction_comp=self.pd_friction_comp
                    )
                    with self._servo_lock:
                        self._latest_ur_q = q.copy()
                    count += 1
                    status_dt = time.monotonic() - last_status_t
                    if status_dt >= 1.0:
                        msg = Float32()
                        msg.data = float(count / status_dt)
                        self.servo_hz_pub.publish(msg)
                        count = 0
                        last_status_t = time.monotonic()
                except Exception as exc:
                    if self._servo_thread_running:
                        self.get_logger().warn(f"servo thread error: {exc}")
                    time.sleep(period)

            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

    # ------------------------------------------------------- observation loop
    def _observation_loop(self):
        period = 1.0 / max(self.observation_hz, 1.0)
        next_tick = time.perf_counter()
        while self._obs_thread_running:
            try:
                q, wrench = self._read_robot_observation()
                with self._servo_lock:
                    self._latest_wrench = wrench.copy()
                self.obs_ur_state_pub.publish(create_array_msg(q))
                self.obs_ur_wrench_pub.publish(create_array_msg(wrench.tolist()))
                # 2-dim [position, current] in raw Robotiq 0..255 units, matching
                # the obs_gripper layout recorded during data collection.
                self.obs_gripper_pub.publish(create_array_msg(
                    [self._follower_gripper_pos_255,
                     self._follower_gripper_current_255]
                ))
            except Exception as exc:
                if self._obs_thread_running:
                    self.get_logger().warn(f"observation thread error: {exc}")
                time.sleep(period)

            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

    # ---------------------------------------------------------- gripper loop
    def _gripper_io_loop(self):
        period = 0.04  # ~25 Hz; the Robotiq URCap socket is slow
        while self._gripper_thread_running:
            try:
                self.gripper.set_position(self._desired_gripper_255)
                self._follower_gripper_pos_255 = self.gripper.get_position()
                self._follower_gripper_current_255 = self.gripper.get_current()
            except Exception:
                pass
            time.sleep(period)

    # ------------------------------------------------------------------ shut
    def shut_down(self):
        self._servo_thread_running = False
        if getattr(self, "_servo_thread", None) is not None:
            self._servo_thread.join(timeout=1.0)
        self._obs_thread_running = False
        if getattr(self, "_obs_thread", None) is not None:
            self._obs_thread.join(timeout=1.0)
        self._gripper_thread_running = False
        if getattr(self, "_gripper_thread", None) is not None:
            self._gripper_thread.join(timeout=1.0)
        # Zero the direct torque to hand control back cleanly, then release RTDE.
        try:
            self.rtde_c.directTorque(
                [0.0] * self.num_arm_joints, friction_comp=self.pd_friction_comp
            )
        except Exception:
            pass
        try:
            self.rtde_c.stopScript()
        except Exception:
            pass
        try:
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()
        except Exception:
            pass
        if self.gripper is not None:
            self.gripper.close()


def main(args=None):
    rclpy.init(args=args)
    node = FACTRTeleopUR7eRollout()
    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down...")
    finally:
        node.shut_down()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
