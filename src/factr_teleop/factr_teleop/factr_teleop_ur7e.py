# UR7e follower implementation using UR RTDE (ur_rtde).

import socket
import threading
import time

import numpy as np
import pinocchio as pin

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

from factr_teleop.factr_teleop import FACTRTeleop
from python_utils.global_configs import ur_left_real_addresses, ur_right_real_addresses


def create_array_msg(data):
    msg = JointState()
    msg.position = list(map(float, data))
    return msg


class RobotiqGripper:
    """
    Minimal client for the Robotiq 2F-85 socket exposed by the Robotiq URCap.

    When the Robotiq URCap is installed on the UR controller, it opens a plain
    ASCII command socket (default port 63352) on the robot. Commands are of the
    form "SET <VAR> <val>" / "GET <VAR>"; positions are 0..255 (0 = open,
    255 = closed for the 2F-85).
    """

    def __init__(self, ip, port=63352, timeout=2.0):
        self._sock = socket.create_connection((ip, port), timeout=timeout)
        self._lock = threading.Lock()

    def _cmd(self, cmd):
        with self._lock:
            self._sock.sendall((cmd + "\n").encode())
            return self._sock.recv(1024).decode().strip()

    def _get_var(self, name):
        # response looks like "POS 12"
        resp = self._cmd(f"GET {name}")
        return int(resp.split()[1])

    def activate(self, speed=255, force=128, wait_timeout=5.0):
        self._cmd("SET ACT 1")
        self._cmd(f"SET SPE {int(np.clip(speed, 0, 255))}")
        self._cmd(f"SET FOR {int(np.clip(force, 0, 255))}")
        self._cmd("SET GTO 1")
        t0 = time.time()
        # STA == 3 -> activation complete
        while self._get_var("STA") != 3 and time.time() - t0 < wait_timeout:
            time.sleep(0.1)

    def set_position(self, pos):
        self._cmd(f"SET POS {int(np.clip(pos, 0, 255))}")

    def get_position(self):
        return self._get_var("POS")  # 0 open .. 255 closed

    def get_current(self):
        return self._get_var("COU")  # motor current 0..255 (grasp-force proxy)

    def get_object(self):
        # object-detection status: 0 moving, 1 stopped-on-contact while opening,
        # 2 stopped-on-contact while closing (a grasp), 3 reached requested position.
        return self._get_var("OBJ")

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass


class FACTRTeleopUR7e(FACTRTeleop):
    """
    Implements the FACTRTeleop communication layer for a UR7e follower arm using
    UR RTDE (ur_rtde).

    Unlike the Franka example, there is no separate follower process and no ZMQ
    hop: the UR control box is itself the follower controller. We stream joint
    position targets to it with servoJ (the controller does the real-time
    interpolation/tracking internally), read the external TCP wrench for joint-
    space force-feedback, and drive a Robotiq 2F-85 gripper over its URCap socket.
    """

    def __init__(self):
        self.gripper_feedback_magnitude = 0.2
        self.gripper_torque_ema_beta = 0.9
        self.gripper_current_limit_ma = 250.0
        self._last_servo_t = None
        super().__init__()
        # Only needed if force-feedback to the leader gripper is enabled. Use
        # .get() so the demo/teleop configs that disable it need not carry gains.
        gf = self.config["controller"]["gripper_feedback"]
        # On/off grasp-feedback torque magnitude (before the current cap below).
        self.gripper_feedback_magnitude = gf.get("magnitude", 0.2)
        self.gripper_torque_ema_beta = gf.get("ema_beta", 0.9)
        # Hard cap on the trigger (XL330-M077) motor current, in mA. Its continuous
        # (rated) torque is only ~20% of stall (~250 mA); sustained current above
        # that overloads the motor since the trigger must push continuously against
        # the operator's finger. Capped on the FACTR side (see gripper_feedback).
        self.gripper_current_limit_ma = gf.get("current_limit_ma", 250.0)
        self._ensure_collision_safety_state()

    def _ensure_collision_safety_state(self):
        if hasattr(self, "enable_collision_safety"):
            return
        self.enable_collision_safety = self.declare_parameter(
            "collision_safety", False
        ).get_parameter_value().bool_value
        self.safe_target_timeout = self.declare_parameter(
            "safe_target_timeout", 0.25
        ).get_parameter_value().double_value
        self._latest_safe_ur_pos = None
        self._latest_safe_ur_pos_t = 0.0

    # ------------------------------------------------------------------ setup
    def set_up_communication(self):
        """
        Sets up the communication with the integrated UR arm controllers
        Sets up basic settings
        Makes the follower arm match the leader arm and throws an exception if the error is too large
        Starts the ROS nodes
        Starts the gripper io loop
        """
        self._ensure_collision_safety_state()
        if self.name == "left":
            addr = ur_left_real_addresses
        elif self.name == "right":
            addr = ur_right_real_addresses
        else:
            raise ValueError(f"Invalid robot name '{self.name}'. Expected 'left' or 'right'.")

        self.robot_ip = addr["ip"]
        self.ur_cap_port = addr["ur_cap_port"]
        self.gripper_port = addr["gripper_port"]

        frequency = float(self.config["controller"]["frequency"])

        # gain chases. ~0.1 / ~300 is a good starting point for policy-rate teleop.
        servo_cfg = self.config["arm_teleop"].get("servo", {})
        # lookahead time is in seconds and it asks how smoother
        #servoJ has an internal smoother where it sets accel such that the newest target gets there at lookahead_time
            #gain decides how stiffly the arm follows the plan created.
        self.servo_lookahead_time = servo_cfg.get("lookahead_time", 0.1)
        self.servo_gain = servo_cfg.get("gain", 300.0)
        self.servo_command_hz = float(servo_cfg.get("command_hz", frequency))
        self.observation_hz = float(servo_cfg.get("observation_hz", self.servo_command_hz))
        self.enable_fast_servo_thread = self.enable_collision_safety and bool(
            servo_cfg.get("fast_servo_thread", True)
        )
        self.enable_observation_thread = bool(servo_cfg.get("observation_thread", True))
        self._servo_lock = threading.Lock()
        self._servo_thread_running = False
        self._servo_thread = None
        self._servo_last_cmd_q = None
        self._servo_last_cmd_t = 0.0
        self._servo_count_since_status = 0
        self._servo_last_status_t = time.monotonic()
        self._last_safe_wait_log_t = 0.0
        self._safe_target_received_logged = False
        self._latest_ur_q = None
        self._latest_tcp_wrench = np.zeros(6, dtype=np.float64)
        self._obs_lock = threading.Lock()
        self._obs_thread_running = False
        self._obs_thread = None
        self._obs_count_since_status = 0
        self._obs_last_status_t = time.monotonic()

        #Promise that we'll send commands at this freq. If not, kill control.
        self.watchdog_min_frequency = servo_cfg.get("watchdog_min_frequency", 20.0)

        # Connect RTDE. The control interface uses the ExternalControl URCap, so
        # the ExternalControl program must be running (Play pressed) on the
        # pendant, with Host IP = this PC and Custom port = self.ur_cap_port.
        self.get_logger().info(
            f"FACTR UR7e {self.name}: connecting RTDE to {self.robot_ip} "
            f"(ur_cap_port={self.ur_cap_port}). Press PLAY on the ExternalControl program."
        )
        self.rtde_r = RTDEReceiveInterface(self.robot_ip)
        self.rtde_c = RTDEControlInterface(
            self.robot_ip, frequency,
            RTDEControlInterface.FLAG_USE_EXT_UR_CAP, self.ur_cap_port,
        )
        # Safety net: if commands stop arriving faster than this, the controller
        # stops the arm. servoJ in the control loop is the heartbeat that kicks it.
        self.rtde_c.setWatchdog(self.watchdog_min_frequency)
        self.get_logger().info(f"FACTR UR7e {self.name}: RTDE connected ({self.rtde_c.isConnected()}).")

        # Zero the wrist 6-axis F/T sensor so getActualTCPForce() reads ~0 at rest.
        # Payload mass/CoG must already be configured on the controller (they are);
        # zeroing removes residual bias/drift. The arm must be static here.
        self.rtde_c.zeroFtSensor()

        # SAFETY: the first servoJ target will be the leader's matched pose
        # (initial_match_joint_pos, the shared start reference). If the follower
        # UR is not already near it, the arm will jump on the first command.
        # Refuse to start (fail-safe) and tear down the RTDE session. The fix is
        # to jog the UR to initial_match_joint_pos before launching -- do NOT
        # change the config to the UR's current pose, that would desync the
        # leader matching / calibration.
        follower_q = np.array(self.rtde_r.getActualQ())
        match_q = self.initial_match_joint_pos[0:self.num_arm_joints]
        # Per-joint absolute error across all arm joints.
        per_joint_err = np.abs(follower_q[:self.num_arm_joints] - match_q[:self.num_arm_joints])
        if np.any(per_joint_err > 0.5):
            try:
                self.rtde_c.servoStop()
                self.rtde_c.stopScript()
                self.rtde_c.disconnect()
                self.rtde_r.disconnect()
            except Exception:
                pass
            raise RuntimeError(
                f"FACTR UR7e {self.name}: follower start config differs from "
                f"initial_match_joint_pos per-joint by "
                f"{[round(float(e), 3) for e in per_joint_err]} rad (limit 0.5). "
                f"Refusing to start to avoid a servoJ jump. "
                f"Jog the UR to initial_match_joint_pos = {[round(x, 4) for x in match_q]} "
                f"before launching (UR is currently at "
                f"{[round(x, 4) for x in follower_q.tolist()]})."
            )

        # End-effector frame in the leader URDF, used to build the task Jacobian
        # with pinocchio for haptic force feedback (see get_leader_arm_external_
        # joint_torque). self.pin_model is built by the base class from the leader
        # URDF in _prepare_inverse_dynamics(), which runs before this method.
        ee_frame = self.config["arm_teleop"].get("ee_frame", "handle")
        self.ee_frame_id = self.pin_model.getFrameId(ee_frame)

        # ROS publishers for logging / behavior-cloning data collection.
        self.obs_ur_state_pub = self.create_publisher(JointState, f'/ur/{self.name}/obs_ur_state', 10)
        self.desired_ur_pos_pub = self.create_publisher(JointState, f'/factr_teleop/{self.name}/desired_ur_pos', 10)
        self.cmd_ur_pos_pub = self.create_publisher(JointState, f'/factr_teleop/{self.name}/cmd_ur_pos', 10)
        self.cmd_gripper_pos_pub = self.create_publisher(JointState, f'/factr_teleop/{self.name}/cmd_gripper_pos', 10)
        self.servo_hz_pub = self.create_publisher(Float32, f'/factr_teleop/{self.name}/servo_hz', 10)
        self.observation_hz_pub = self.create_publisher(Float32, f'/factr_teleop/{self.name}/observation_hz', 10)
        # Raw TCP wrench is logged unconditionally as an observation (cheap
        # streamed read) -- force data must not depend on whether haptic feedback
        # happens to be enabled.
        self.obs_ur_wrench_pub = self.create_publisher(JointState, f'/ur/{self.name}/obs_ur_wrench', 10)
        # Follower gripper state ([position, current] in raw Robotiq 0..255 units,
        # matching the command logging) -- always-on observation for training.
        self.obs_gripper_pub = self.create_publisher(JointState, f'/ur/{self.name}/obs_gripper', 10)
        if self.enable_collision_safety:
            self.create_subscription(
                JointState,
                f"/factr_teleop/{self.name}/safe_ur_pos",
                self._safe_ur_pos_cb,
                10,
            )
            self.get_logger().info(
                f"FACTR UR7e {self.name}: using safe_ur_pos safety filter topic."
            )

        # ---- Robotiq 2F-85 gripper (optional, runs in its own slow thread) ----
        gripper_cfg = self.config["gripper_teleop"]
        self.enable_follower_gripper = gripper_cfg.get("enable_follower_gripper", True)
        self.gripper_invert = gripper_cfg.get("invert", False)
        self.gripper = None
        self._gripper_thread_running = False
        self._desired_gripper_255 = 0           # written by control loop, read by thread
        self._follower_gripper_pos_255 = 0      # raw Robotiq 0..255 (cached for logging)
        self._follower_gripper_current_255 = 0  # raw Robotiq 0..255 (cached for logging)
        self._follower_gripper_current = 0.0    # normalized 0..1 EMA (cached for feedback)
        self._follower_gripper_obj = 3          # Robotiq OBJ status (3 = no object held)
        self._grasp_torque = 0.0                # latched gripper-feedback torque
        self._grasp_release_count = 0           # consecutive "no object" cycles seen
        if self.enable_follower_gripper:
            try:
                self.gripper = RobotiqGripper(self.robot_ip, self.gripper_port)
                self.gripper.activate(
                    speed=gripper_cfg.get("speed", 255),
                    force=gripper_cfg.get("force", 128),
                )
                self._gripper_thread_running = True
                self._gripper_thread = threading.Thread(target=self._gripper_io_loop, daemon=True)
                self._gripper_thread.start()
                self.get_logger().info(f"FACTR UR7e {self.name}: Robotiq 2F-85 connected and activated.")
            except Exception as e:
                self.get_logger().warn(
                    f"FACTR UR7e {self.name}: Robotiq gripper init failed ({e}). "
                    f"Continuing without follower gripper."
                )
                self.gripper = None

    def _post_match_start(self): 
        # After leader matches the initial pos, we initialize UR states (wrench, joint positions)
        # and start the fast servo command thread. Compared to the main leader loop, the fast servo
        # thread allows us to be able to continuously send the same joint position command while the
        # loop runs behind it
        q_hold, wrench_hold = self._read_robot_observation()
        with self._servo_lock:
            self._latest_ur_q = q_hold.copy()
            self._servo_last_cmd_q = q_hold.copy()
            self._servo_last_cmd_t = time.monotonic()
        with self._obs_lock:
            self._latest_tcp_wrench = wrench_hold.copy()

        if self.enable_observation_thread:
            self._obs_thread_running = True
            self._obs_thread = threading.Thread(target=self._observation_loop, daemon=True)
            self._obs_thread.start()
            self.get_logger().info(
                f"FACTR UR7e {self.name}: RTDE observation thread running at "
                f"{self.observation_hz:.1f} Hz."
            )

        if not self.enable_fast_servo_thread:
            return

        self._servo_thread_running = True
        self._servo_thread = threading.Thread(target=self._servo_loop, daemon=True)
        self._servo_thread.start()
        self.get_logger().info(
            f"FACTR UR7e {self.name}: fast servoJ thread running at "
            f"{self.servo_command_hz:.1f} Hz."
        )

    def _servo_loop(self):
        period = 1.0 / max(self.servo_command_hz, 1.0)
        next_tick = time.perf_counter()
        while self._servo_thread_running:
            now_mono = time.monotonic()
            with self._servo_lock:
                safe_fresh = (
                    self._latest_safe_ur_pos is not None
                    and now_mono - self._latest_safe_ur_pos_t <= self.safe_target_timeout
                )
                if safe_fresh:
                    target_q = self._latest_safe_ur_pos.copy()
                elif self._latest_ur_q is not None:
                    target_q = self._latest_ur_q.copy()
                elif self._servo_last_cmd_q is not None:
                    target_q = self._servo_last_cmd_q.copy()
                else:
                    target_q = None
                safe_age = None if self._latest_safe_ur_pos is None else now_mono - self._latest_safe_ur_pos_t

            if self.enable_collision_safety and not safe_fresh and now_mono - self._last_safe_wait_log_t >= 1.0:
                if safe_age is None:
                    reason = "no safe_ur_pos has been received"
                else:
                    reason = f"safe_ur_pos is stale ({safe_age:.3f}s old)"
                self.get_logger().warn(
                    f"FACTR UR7e {self.name}: holding current pose because {reason}."
                )
                self._last_safe_wait_log_t = now_mono

            if target_q is not None:
                try:
                    # After we send a servoJ command, we then update our previous commanded joint positions
                    # to operate as a fallback (so we hold the last commanded joint positions). By using 
                    # _servo_lock, we ensure that no other variable can overwrite the following variables 
                    # in the with ___ statement
                    self.rtde_c.servoJ(
                        list(map(float, target_q)),
                        0.0,
                        0.0,
                        period,
                        self.servo_lookahead_time,
                        self.servo_gain,
                    )
                    with self._servo_lock:
                        self._servo_last_cmd_q = target_q.copy()
                        self._servo_last_cmd_t = time.monotonic()
                        self._servo_count_since_status += 1
                        status_dt = self._servo_last_cmd_t - self._servo_last_status_t
                        if status_dt >= 1.0:
                            msg = Float32()
                            msg.data = float(self._servo_count_since_status / status_dt)
                            self.servo_hz_pub.publish(msg)
                            self._servo_count_since_status = 0
                            self._servo_last_status_t = self._servo_last_cmd_t
                except Exception as exc:
                    if self._servo_thread_running:
                        self.get_logger().warn(f"fast servoJ thread error: {exc}")
                    time.sleep(period)

            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

    def _read_robot_observation(self):
        q = np.array(self.rtde_r.getActualQ(), dtype=np.float64)[:self.num_arm_joints]
        wrench = np.array(self.rtde_r.getActualTCPForce(), dtype=np.float64)
        return q, wrench

    def _cache_robot_observation(self, q, wrench):
        q = np.asarray(q, dtype=np.float64)[:self.num_arm_joints]
        wrench = np.asarray(wrench, dtype=np.float64)
        with self._servo_lock:
            self._latest_ur_q = q.copy()
        with self._obs_lock:
            self._latest_tcp_wrench = wrench.copy()

    def _get_cached_robot_observation(self):
        with self._servo_lock:
            q = None if self._latest_ur_q is None else self._latest_ur_q.copy()
        with self._obs_lock:
            wrench = self._latest_tcp_wrench.copy()
        return q, wrench

    def _publish_observation_hz(self):
        self._obs_count_since_status += 1
        now_mono = time.monotonic()
        status_dt = now_mono - self._obs_last_status_t
        if status_dt < 1.0:
            return
        msg = Float32()
        msg.data = float(self._obs_count_since_status / status_dt)
        self.observation_hz_pub.publish(msg)
        self._obs_count_since_status = 0
        self._obs_last_status_t = now_mono

    def _observation_loop(self):
        period = 1.0 / max(self.observation_hz, 1.0)
        next_tick = time.perf_counter()
        while self._obs_thread_running:
            try:
                q, wrench = self._read_robot_observation()
                self._cache_robot_observation(q, wrench)
                self.obs_ur_state_pub.publish(create_array_msg(q))
                self.obs_ur_wrench_pub.publish(create_array_msg(wrench.tolist()))
                self._publish_observation_hz()
            except Exception as exc:
                if self._obs_thread_running:
                    self.get_logger().warn(f"RTDE observation thread error: {exc}")
                time.sleep(period)

            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

    def _gripper_io_loop(self):
        """
        The Robotiq URCap socket is slow (~ms per command), so all gripper I/O
        is decoupled from the 500 Hz arm control loop and runs here instead.
        """
        period = 0.04  # ~25 Hz
        while self._gripper_thread_running:
            try:
                self.gripper.set_position(self._desired_gripper_255)
                # Always read position AND current (raw 0..255) for logging --
                # decoupled from feedback so demos capture grasp force regardless.
                self._follower_gripper_pos_255 = self.gripper.get_position()
                cur255 = self.gripper.get_current()
                self._follower_gripper_current_255 = cur255
                # Object-detection status, used to gate feedback to real grasps only.
                self._follower_gripper_obj = self.gripper.get_object()
                # EMA on normalized current, consumed only by the haptic feedback path.
                self._follower_gripper_current = self.gripper_torque_ema_beta * (self._follower_gripper_current) \
                    + (1.0 - self.gripper_torque_ema_beta) * (cur255 / 255.0)
            except Exception:
                pass
            time.sleep(period)

    def _safe_ur_pos_cb(self, msg):
        if len(msg.position) < self.num_arm_joints:
            self.get_logger().warn(
                f"Ignoring safe target with {len(msg.position)} positions; "
                f"expected {self.num_arm_joints}."
            )
            return
        with self._servo_lock:
            self._latest_safe_ur_pos = np.asarray(
                msg.position[:self.num_arm_joints], dtype=np.float64
            )
            self._latest_safe_ur_pos_t = time.monotonic()
            first_safe_target = not self._safe_target_received_logged
            self._safe_target_received_logged = True
        if first_safe_target:
            self.get_logger().info(
                f"FACTR UR7e {self.name}: first safe_ur_pos received."
            )

    # ----------------------------------------------------------- force feedback
    def get_leader_arm_external_joint_torque(self):
        """
        Joint torques for HAPTIC feedback to the leader: the external TCP wrench
        mapped to joint space via the task Jacobian transpose, tau = J(q)^T @ wrench.

        The leader's Dynamixel motors are joint-space actuators, so a Cartesian
        wrench can only be rendered on them through J^T -- this mapping is required
        for arm force feedback (it is NOT used for the training data, which logs
        the raw wrench in update_communication instead).

        J is computed locally with pinocchio from the leader URDF (a kinematic
        replica of the UR), avoiding the per-cycle getJacobian RPC. It is built in
        LOCAL_WORLD_ALIGNED frame so its rows [linear; angular] match the base-
        frame wrench [force; torque] from getActualTCPForce(). q is the follower's
        actual configuration -- the pose at which the wrench was measured.
        """
        q, wrench = self._get_cached_robot_observation()
        if q is None:
            q, wrench = self._read_robot_observation()
            self._cache_robot_observation(q, wrench)
        # The cached wrench is already negated for haptic feedback. With the raw
        # sign, J^T @ wrench
        # drives the leader in the same direction as an external push on the
        # follower (assistive -- the arm "runs away" toward the pusher). Haptic
        # feedback must *oppose* the push so the operator feels resistance.
        J = pin.computeFrameJacobian(
            self.pin_model, self.pin_data, q,
            self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )                                                          # (6, n) base-aligned
        external_torque = J.T @ wrench                             # (n,) joint torques
        return external_torque

    # --------------------------------------------------------- gripper feedback
    def get_leader_gripper_feedback(self):
        # Follower grasp-force proxy (Robotiq motor current, normalized 0..1),
        # cached by the gripper I/O thread.
        return self._follower_gripper_current

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        # Only render force when the follower has stopped ON AN OBJECT (Robotiq OBJ
        # status). Trigger convention: 0 = closed, ~0.9 = open. Empirically a
        # POSITIVE motor torque drives the trigger toward 0 (closed), so the
        # "resist closing" direction we want on a grasp -- pushing the trigger
        # toward OPEN -- is NEGATIVE.
        #   OBJ == 2 -> stopped while CLOSING on an object (grasp): resist further
        #               closing -> push trigger toward OPEN -> negative.
        #   OBJ == 1 -> stopped while OPENING on an object: push toward CLOSED ->
        #               positive.
        #   OBJ 0/3  -> moving / reached commanded position, no object: no feedback.
        magnitude = self.gripper_feedback_magnitude
        RELEASE_DEBOUNCE = 40  # OBJ must read 0/3 this many cycles before releasing
        obj = self._follower_gripper_obj
        if obj == 2:                       # grasp while closing
            self._grasp_torque = magnitude
            self._grasp_release_count = 0
        elif obj == 1:                     # grasp while opening
            self._grasp_torque = -magnitude
            self._grasp_release_count = 0
        elif self._grasp_torque != 0.0:    # OBJ 0/3 while a grasp is latched: debounce
            self._grasp_release_count += 1
            if self._grasp_release_count >= RELEASE_DEBOUNCE:
                self._grasp_torque = 0.0
                self._grasp_release_count = 0
        # Cap the trigger current (FACTR side): convert the configured mA limit to a
        # torque cap via the gripper motor's torque->current map (mA per torque unit)
        # and clamp. This bounds the steady current so the motor cannot overload.
        torque_cap = self.gripper_current_limit_ma / self.driver.torque_to_current_map[-1]
        self._grasp_torque = float(np.clip(self._grasp_torque, -torque_cap, torque_cap))
        return self._grasp_torque

    # ----------------------------------------------------------- command stream
    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        # servoJ's `time` arg must match the ACTUAL interval between calls, not the
        # nominal 1/frequency. The loop is Dynamixel/feedback-bound and runs well
        # below the configured rate (~130 Hz vs 500 Hz) with jitter, so passing
        # self.dt (=0.002) makes each target expire before the next arrives and the
        # arm barely moves. Measure the real period and feed that instead.
        now = time.perf_counter()
        servo_dt = self.dt if self._last_servo_t is None else now - self._last_servo_t
        self._last_servo_t = now
        print(f"hz of the servo: {1/servo_dt}")
        servo_dt = float(np.clip(servo_dt, 0.002, 0.05))
        # v and a are ignored by servoJ; time/lookahead/gain shape the tracking.
        current_q, obs_wrench = self._get_cached_robot_observation()
        if current_q is None:
            current_q, obs_wrench = self._read_robot_observation()
            self._cache_robot_observation(current_q, obs_wrench)

        if self.enable_collision_safety:
            desired_q = np.array(leader_arm_pos, dtype=np.float64)
            self.desired_ur_pos_pub.publish(create_array_msg(desired_q))
            with self._servo_lock:
                self._latest_ur_q = current_q.copy()
                safe_fresh = (
                    self._latest_safe_ur_pos is not None
                    and time.monotonic() - self._latest_safe_ur_pos_t <= self.safe_target_timeout
                )
                target_q = self._latest_safe_ur_pos.copy() if safe_fresh else current_q
                logged_cmd_q = self._servo_last_cmd_q.copy() if self._servo_last_cmd_q is not None else target_q
        else:
            target_q = leader_arm_pos
            logged_cmd_q = target_q

        if not self.enable_fast_servo_thread:
            self.rtde_c.servoJ(
                list(map(float, target_q)),
                0.0, 0.0, servo_dt, self.servo_lookahead_time, self.servo_gain,
            )
            logged_cmd_q = target_q

        # Map the leader trigger angle to the Robotiq command space (0..255).
        norm = float(np.clip(leader_gripper_pos / self.gripper_limit_max, 0.0, 1.0))
        if self.gripper_invert:
            norm = 1.0 - norm
        gripper_cmd_255 = int(norm * 255)
        # Hand the desired gripper opening to the slow gripper thread.
        if self.gripper is not None:
            self._desired_gripper_255 = gripper_cmd_255

        # ROS logging for data collection. The gripper action is logged in
        # follower (Robotiq, 0..255) units -- the space the policy commands at
        # deployment -- not the leader's trigger angle.
        self.cmd_ur_pos_pub.publish(create_array_msg(logged_cmd_q))
        self.cmd_gripper_pos_pub.publish(create_array_msg([gripper_cmd_255]))
        if not self.enable_observation_thread:
            self.obs_ur_state_pub.publish(create_array_msg(current_q))
            self.obs_ur_wrench_pub.publish(create_array_msg(obs_wrench.tolist()))
        # Follower gripper [position, current] in raw Robotiq 0..255 units, cached
        # by the gripper thread (values stay at 0 if no gripper is connected).
        self.obs_gripper_pub.publish(create_array_msg(
            [self._follower_gripper_pos_255, self._follower_gripper_current_255]
        ))

    # ------------------------------------------------------------------ cleanup
    def shut_down(self):
        self._servo_thread_running = False
        if getattr(self, "_servo_thread", None) is not None:
            self._servo_thread.join(timeout=1.0)
        self._obs_thread_running = False
        if getattr(self, "_obs_thread", None) is not None:
            self._obs_thread.join(timeout=1.0)
        # Stop the gripper thread first so it stops touching the socket.
        self._gripper_thread_running = False
        if getattr(self, "_gripper_thread", None) is not None:
            self._gripper_thread.join(timeout=1.0)
        # Land the arm: stop servoing, release the control script, disconnect.
        try:
            self.rtde_c.servoStop()
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
        # Base class: zero leader torque and disable Dynamixel torque.
        super().shut_down()


def main(args=None):
    rclpy.init(args=args)
    factr_teleop_ur7e = FACTRTeleopUR7e()
    if getattr(factr_teleop_ur7e, "leader_match_only", False):
        factr_teleop_ur7e.destroy_node()
        rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_ur7e)
    except KeyboardInterrupt:
        # print() rather than get_logger(): rclpy's context is already shutting
        # down here, so a rosout publish would just warn "publisher's context is invalid".
        print("Keyboard interrupt received. Shutting down...")
    finally:
        # Always run shut_down: under `ros2 launch`, rclpy's SIGINT handler
        # consumes Ctrl+C and makes spin() return normally (no KeyboardInterrupt),
        # so putting shut_down only in the except branch leaves Dynamixel torque
        # enabled and the leader arm latched holding gravity comp.
        factr_teleop_ur7e.shut_down()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
