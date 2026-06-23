# UR7e follower implementation using UR RTDE (ur_rtde).

import socket
import threading
import time

import numpy as np
import pinocchio as pin

import rclpy
from sensor_msgs.msg import JointState

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
        super().__init__()
        # Only needed if force-feedback to the leader gripper is enabled. Use
        # .get() so the demo/teleop configs that disable it need not carry gains.
        gf = self.config["controller"]["gripper_feedback"]
        self.gripper_feedback_gain = gf.get("gain", 1.0) #.get is used to prevent key error if they aren't configured.
        self.gripper_torque_ema_beta = gf.get("ema_beta", 0.9)
        # Timestamp of the previous servoJ call, used to measure the real control
        # period for servoJ's `time` argument (see update_communication).
        self._last_servo_t = None
        self._last_watchdog_wait_warn_t = 0.0

    # ------------------------------------------------------------------ setup
    def set_up_communication(self):
        """
        Sets up the communication with the integrated UR arm controllers
        Sets up basic settings
        Makes the follower arm match the leader arm and throws an exception if the error is too large
        Starts the ROS nodes
        Starts the gripper io loop
        """
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
        self.cmd_ur_pos_pub = self.create_publisher(JointState, f'/factr_teleop/{self.name}/cmd_ur_pos', 10)
        self.cmd_gripper_pos_pub = self.create_publisher(JointState, f'/factr_teleop/{self.name}/cmd_gripper_pos', 10)
        # Raw TCP wrench is logged unconditionally as an observation (cheap
        # streamed read) -- force data must not depend on whether haptic feedback
        # happens to be enabled.
        self.obs_ur_wrench_pub = self.create_publisher(JointState, f'/ur/{self.name}/obs_ur_wrench', 10)
        # Follower gripper state ([position, current] in raw Robotiq 0..255 units,
        # matching the command logging) -- always-on observation for training.
        self.obs_gripper_pub = self.create_publisher(JointState, f'/ur/{self.name}/obs_gripper', 10)

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

    def _while_waiting_for_start_pos(self):
        try:
            self.rtde_c.kickWatchdog()
        except Exception as e:
            now = time.monotonic()
            if now - self._last_watchdog_wait_warn_t > 2.0:
                self._last_watchdog_wait_warn_t = now
                self.get_logger().warn(
                    f"FACTR UR7e {self.name}: RTDE watchdog kick failed while waiting "
                    f"for leader match ({e}). The ExternalControl program may need "
                    f"to be replayed."
                )

    def _on_start_pos_matched(self):
        try:
            self.rtde_c.kickWatchdog()
        except Exception:
            pass

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
                # EMA on normalized current, consumed only by the haptic feedback path.
                self._follower_gripper_current = self.gripper_torque_ema_beta * (self._follower_gripper_current) \
                    + (1.0 - self.gripper_torque_ema_beta) * (cur255 / 255.0)
            except Exception:
                pass
            time.sleep(period)

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
        q = np.array(self.rtde_r.getActualQ())
        wrench = np.array(self.rtde_r.getActualTCPForce())          # (6,) base frame [F; T]
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
        # Resist the leader trigger in proportion to the follower's grasp force.
        torque_gripper = -1.0 * gripper_feedback / self.gripper_feedback_gain
        return torque_gripper

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
        servo_dt = float(np.clip(servo_dt, 0.002, 0.05))
        # v and a are ignored by servoJ; time/lookahead/gain shape the tracking.
        self.rtde_c.servoJ(
            list(map(float, leader_arm_pos)),
            0.0, 0.0, servo_dt, self.servo_lookahead_time, self.servo_gain,
        )

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
        self.cmd_ur_pos_pub.publish(create_array_msg(leader_arm_pos))
        self.cmd_gripper_pos_pub.publish(create_array_msg([gripper_cmd_255]))
        self.obs_ur_state_pub.publish(create_array_msg(self.rtde_r.getActualQ()))
        # Raw TCP wrench (base frame [Fx,Fy,Fz,Tx,Ty,Tz]) as an always-on force
        # observation -- cheap streamed read, no Jacobian, no dependence on the
        # feedback flag. This is the force signal to train policies on.
        self.obs_ur_wrench_pub.publish(create_array_msg(self.rtde_r.getActualTCPForce()))
        # Follower gripper [position, current] in raw Robotiq 0..255 units, cached
        # by the gripper thread (values stay at 0 if no gripper is connected).
        self.obs_gripper_pub.publish(create_array_msg(
            [self._follower_gripper_pos_255, self._follower_gripper_current_255]
        ))

    # ------------------------------------------------------------------ cleanup
    def shut_down(self):
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

    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_ur7e)
    except KeyboardInterrupt:
        factr_teleop_ur7e.get_logger().info("Keyboard interrupt received. Shutting down...")
        factr_teleop_ur7e.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
