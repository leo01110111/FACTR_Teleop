# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

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

import os
import sys
import time
import yaml
import subprocess
import numpy as np
import pinocchio as pin
from abc import ABC, abstractmethod

from rclpy.node import Node
from std_msgs.msg import Float32
from python_utils.utils import get_workspace_root
from factr_teleop.dynamixel.driver import DynamixelDriver


def find_ttyusb(port_name):
    """
    This function is used to locate the underlying ttyUSB device.
    """
    base_path = "/dev/serial/by-id/"
    full_path = os.path.join(base_path, port_name)
    if not os.path.exists(full_path):
        raise Exception(f"Port '{port_name}' does not exist in {base_path}.")
    try:
        resolved_path = os.readlink(full_path)
        actual_device = os.path.basename(resolved_path)
        if actual_device.startswith("ttyUSB"):
            return actual_device
        else:
            raise Exception(
                f"The port '{port_name}' does not correspond to a ttyUSB device. It links to {resolved_path}."
            )
    except Exception as e:
        raise Exception(f"Unable to resolve the symbolic link for '{port_name}'. {e}")


class FACTRTeleop(Node, ABC):
    """
    Base class for implementing the FACTR low-cost force-feedback teleoperation system 
    for a follower arm.

    This class implements the control loop for the leader teleoperation arm, including
    features such as gravity compensation, null-space regulation, friction compensation,
    and force-feedback.

    Note that this class should be used as a parent class, where the defined abstract 
    methods must be implemented by subclasses for handling communication between the 
    leader and follower arms, as well as force-feedback for the leader gripper.
    """
    def __init__(self):
        super().__init__('factr_teleop')

        config_file_name = self.declare_parameter('config_file', '').get_parameter_value().string_value
        self.leader_match_only = self.declare_parameter(
            'leader_match_only', False
        ).get_parameter_value().bool_value
        config_path = os.path.join(get_workspace_root(), f"src/factr_teleop/factr_teleop/configs/{config_file_name}")
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.name = self.config["name"]
        self.dt = 1 / self.config["controller"]["frequency"]
        self._leader_loop_count_since_status = 0
        self._leader_loop_last_status_t = time.monotonic()
        self.leader_hz_pub = self.create_publisher(Float32, f"/factr_teleop/{self.name}/leader_hz", 10)
        
        self._prepare_dynamixel()
        self._prepare_inverse_dynamics()

        # leader arm parameters
        self.num_arm_joints = self.config["arm_teleop"]["num_arm_joints"]
        self.safety_margin = self.config["arm_teleop"]["arm_joint_limits_safety_margin"]
        self.arm_joint_limits_max = np.array(self.config["arm_teleop"]["arm_joint_limits_max"]) - self.safety_margin
        self.arm_joint_limits_min = np.array(self.config["arm_teleop"]["arm_joint_limits_min"]) + self.safety_margin
        self.calibration_joint_pos = np.array(self.config["arm_teleop"]["initialization"]["calibration_joint_pos"])
        self.initial_match_joint_pos = np.array(self.config["arm_teleop"]["initialization"]["initial_match_joint_pos"])
        # Pose the leader arm is jogged to on shutdown; defaults to the calibration pose.
        self.end_pose = np.array(
            self.config["arm_teleop"]["initialization"].get("end_pose", self.calibration_joint_pos)
        )
        self.normalize_leader_joint_angles = bool(
            self.config["arm_teleop"]["initialization"].get("normalize_joint_angles", False)
        )
        self._leader_arm_pos_reference = None
        if self.normalize_leader_joint_angles:
            self.get_logger().info("Leader joint angle normalization enabled.")
        assert self.num_arm_joints == len(self.arm_joint_limits_max) == len(self.arm_joint_limits_min), \
            "num_arm_joints and the length of arm joint limits must be the same"
        assert self.num_arm_joints == len(self.calibration_joint_pos) == len(self.initial_match_joint_pos) == len(self.end_pose), \
            "num_arm_joints and the length of calibration_joint_pos, initial_match_joint_pos and end_pose must be the same"
        
        # leader gripper parameters
        self.gripper_limit_min = 0.0
        self.gripper_limit_max = self.config["gripper_teleop"]["actuation_range"]
        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0

        # gravity comp
        self.enable_gravity_comp = self.config["controller"]["gravity_comp"]["enable"]
        self.gravity_comp_modifier = np.asarray(self.config["controller"]["gravity_comp"]["gain"], dtype=float)
        self.tau_g = np.zeros(self.num_arm_joints)
        # friction comp
        self.stiction_comp_enable_speed = self.config["controller"]["static_friction_comp"]["enable_speed"]
        self.stiction_comp_gain = self.config["controller"]["static_friction_comp"]["gain"]
        self.stiction_dither_flag = np.ones((self.num_arm_joints), dtype=bool)
        # joint limit barrier:
        self.joint_limit_kp = self.config["controller"]["joint_limit_barrier"]["kp"]
        self.joint_limit_kd = self.config["controller"]["joint_limit_barrier"]["kd"]
        # null space regulation
        self.null_space_joint_target = np.array(self.config["controller"]["null_space_regulation"]["null_space_joint_target"])
        self.null_space_kp = self.config["controller"]["null_space_regulation"]["kp"]
        self.null_space_kd = self.config["controller"]["null_space_regulation"]["kd"]
        # torque feedback
        self.enable_torque_feedback = self.config["controller"]["torque_feedback"]["enable"]
        self.torque_feedback_gain = self.config["controller"]["torque_feedback"]["gain"]
        self.torque_feedback_motor_scalar = self.config["controller"]["torque_feedback"]["motor_scalar"]
        self.torque_feedback_damping = self.config["controller"]["torque_feedback"]["damping"]
        # gripper feedback
        self.enable_gripper_feedback = self.config["controller"]["gripper_feedback"]["enable"]
        
        if self.leader_match_only:
            self.get_logger().info("Leader match only: skipping follower communication.")
            self._get_dynamixel_offsets()
            self._match_start_pos()
            self.get_logger().info("Leader match only complete; exiting.")
            self.set_leader_joint_torque(np.zeros(self.num_arm_joints), 0.0)
            self.driver.set_torque_mode(False)
            self.timer = None
            return

        # needs to be implemented to establish communication between the leader and the follower
        self.set_up_communication()
        # calibrate the leader arm joints before starting
        self._get_dynamixel_offsets()
        # ensure the leader and the follower arms have the same joint positions before starting
        self._match_start_pos()
        self._post_match_start()
        # start the control loop
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

    def _post_match_start(self):
        """Optional subclass hook after leader/follower matching, before the timer starts."""
        pass


    def _prepare_dynamixel(self):
        """
        Instantiates driver for interfacing with Dynamixel servos.
        """
        self.servo_types = self.config["dynamixel"]["servo_types"]
        self.num_motors = len(self.servo_types)
        self.joint_signs = np.array(self.config["dynamixel"]["joint_signs"], dtype=float)
        assert self.num_motors == len(self.joint_signs), \
            "The number of motors and the number of joint signs must be the same"
        self.dynamixel_port = "/dev/serial/by-id/" + self.config["dynamixel"]["dynamixel_port"]

        # checks of the latency timer on ttyUSB of the corresponding port is 1
        # if it is not 1, the control loop cannot run at above 200 Hz, which will 
        # cause extremely undesirable behaviour for the leader arm. If the latency 
        # timer is not 1, one can set it to 1 as follows:
        # echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB{NUM}/latency_timer
        ttyUSBx = find_ttyusb(self.dynamixel_port)
        command = f"cat /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"        
        result = subprocess.run(command, shell=True, capture_output=True, text=True, check=True)
        ttyUSB_latency_timer = int(result.stdout)
        if ttyUSB_latency_timer != 1:
            raise Exception(
                f"Please ensure the latency timer of {ttyUSBx} is 1. Run: \n \
                echo 1 | sudo tee /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"
            )

        joint_ids = np.arange(self.num_motors) + 1
        try:
            self.driver = DynamixelDriver(
                joint_ids, self.servo_types, self.dynamixel_port
            )
        except FileNotFoundError:
            self.get_logger().info(f"Port {self.dynamixel_port} not found. Please check the connection.")
            return
        self.driver.set_torque_mode(False)
        # set operating mode to current mode
        self.driver.set_operating_mode(0)
        # enable torque
        self.driver.set_torque_mode(True)

    def _prepare_inverse_dynamics(self):
        """
        Creates a model of the leader arm given the its URDF for kinematic and dynamic
        computations used in gravity compensation and null-space regulation calculations.
        """
        self.leader_urdf = os.path.join(
            'src/factr_teleop/factr_teleop/urdf/',
            self.config["arm_teleop"]["leader_urdf"]
        )
        workspace_root = get_workspace_root()
        urdf_model_path = os.path.join(workspace_root, self.leader_urdf)
        self.pin_model = pin.buildModelFromUrdf(urdf_model_path)
        self.pin_data = self.pin_model.createData()

        # pin_model's kinematics/dynamics are defined in the URDF's own joint
        # convention (zero pose and axis directions from onshape-to-robot),
        # which has no reason to agree with dynamixel.joint_signs / joint_offsets
        # above (those align the leader to the follower UR's calibration
        # stance). Gravity comp and null-space regulation need raw motor
        # readings mapped into the URDF's convention instead, via a
        # separately-calibrated sim_offsets_<leader_name>.yaml (see
        # leader_sim_offset_calibrate.py / visualize_leader_urdf.py).
        num_arm_joints = self.config["arm_teleop"]["num_arm_joints"]
        sim_offset_filename = f"sim_offsets_{self.config['dynamixel']['leader_name']}.yaml"
        sim_offset_path = os.path.join(
            workspace_root, "src/factr_teleop/factr_teleop/configs", sim_offset_filename
        )
        if not os.path.isfile(sim_offset_path):
            raise FileNotFoundError(
                f"No sim offsets file found at {sim_offset_path}. Run "
                "leader_sim_offset_calibrate.py first to generate it -- gravity "
                "compensation and null-space regulation need it to map raw motor "
                "readings into the URDF's own joint convention."
            )
        with open(sim_offset_path, 'r') as f:
            sim_config = yaml.safe_load(f)
        self.sim_joint_offsets = np.array(sim_config["offset"], dtype=float)
        self.sim_joint_signs = np.array(sim_config["joint_signs"], dtype=float)[0:num_arm_joints]

        # Elementwise sign flip to convert a torque conjugate to the sim/URDF
        # joint convention into one conjugate to the follower joint convention
        # (both parametrize the same physical joint, just with different
        # signs/offsets) -- see get_leader_joint_states() and
        # gravity_compensation()/null_space_regulation().
        self.sim_to_follower_torque_sign = (
            self.sim_joint_signs * self.joint_signs[0:num_arm_joints]
        )

    def _get_dynamixel_offsets(self, verbose=True):
        """
        Calibrates the Dynamixel servos with respect to the Franka arm to ensure the joint
        position readings of the leader arm correspond to those of the follower arm.

        Before launching this program, the leader arm should be manually placed in a 
        configuration roughly corresponding to the follower's calibration position 
        described in self.calibration_joint_pos (within ±90 degrees per joint).
        """
        import json
        offset_filename = f"offsets_{self.config['dynamixel']['leader_name']}.json"
        offset_file = os.path.join(get_workspace_root(), "src/factr_teleop/factr_teleop/configs", offset_filename)

        if os.path.exists(offset_file):
            if verbose:
                self.get_logger().info(f"FACTR TELEOP: Loading permanent offsets from {offset_filename}")
            with open(offset_file, 'r') as f:
                self.joint_offsets = np.array(json.load(f))
            return

        if verbose:
            self.get_logger().info(f"FACTR TELEOP: Calibrating offsets and saving to {offset_filename}")

        # warm up
        for _ in range(10):
            self.driver.get_positions_and_velocities()
        
        def _get_error(calibration_joint_pos, offset, index, joint_state):
            joint_sign_i = self.joint_signs[index]
            joint_i = joint_sign_i * (joint_state[index] - offset)
            start_i = calibration_joint_pos[index]
            return np.abs(joint_i - start_i)

        # get arm offsets
        self.joint_offsets = []
        curr_joints, _ = self.driver.get_positions_and_velocities()
        for i in range(self.num_arm_joints):
            best_offset = 0
            best_error = 1e9
            # intervals of pi/20 for the first 2 motors and pi/2 for the rest
            if i < 2:
                for offset in np.linspace(-20 * np.pi, 20 * np.pi, 20 * 20 + 1):
                    error = _get_error(self.calibration_joint_pos, offset, i, curr_joints)
                    if error < best_error:
                        best_error = error
                        best_offset = offset
            else:
                for offset in np.linspace(-20 * np.pi, 20 * np.pi, 20 * 4 + 1):
                    error = _get_error(self.calibration_joint_pos, offset, i, curr_joints)
                    if error < best_error:
                        best_error = error
                        best_offset = offset
            self.joint_offsets.append(best_offset)

        # get gripper offset:
        curr_gripper_joint = curr_joints[-1]
        self.joint_offsets.append(curr_gripper_joint)

        self.joint_offsets = np.asarray(self.joint_offsets)
        if verbose:
            print(self.joint_offsets)
            print("best offsets               : ", [f"{x:.3f}" for x in self.joint_offsets])
            print(
                "best offsets function of pi: ["
                + ", ".join([f"{int(np.round(x/(np.pi/2)))}*np.pi/2" for x in self.joint_offsets])
                + " ]",
            )
        
        # save offsets
        with open(offset_file, 'w') as f:
            json.dump(self.joint_offsets.tolist(), f)
    
    def _match_start_pos(self):
        """
        Waits until the leader arm is manually moved to roughly the same configuration as the 
        follower arm before the follower arm starts mirroring the leader arm. 
        """
        curr_pos, _, curr_gripper_pos, _, _, _ = self.get_leader_joint_states()
        printed_match_status = False
        printed_match_lines = 0
        while True:
            joint_pos_error = curr_pos - self.initial_match_joint_pos[0:self.num_arm_joints]
            current_joint_error = np.linalg.norm(joint_pos_error)
            if current_joint_error <= 0.6:
                break

            per_joint_err = np.abs(joint_pos_error)
            match_str = np.array2string(
                self.initial_match_joint_pos[0:self.num_arm_joints],
                precision=2, floatmode="fixed", separator=", ", max_line_width=1000,
            )
            curr_str = np.array2string(
                curr_pos, precision=2, floatmode="fixed", separator=", ", max_line_width=1000,
            )
            err_str = np.array2string(
                per_joint_err, precision=2, floatmode="fixed", separator=", ", max_line_width=1000,
            )
            status_lines = [
                f"Match error: {current_joint_error:.2f}",
                f"Per-joint:   {err_str}",
                f"Target q:    {match_str}",
                f"Current q:   {curr_str}",
                f"Gripper q:   {curr_gripper_pos:.2f}",
            ]
            if printed_match_status:
                sys.stdout.write(f"\x1b[{printed_match_lines}F")
                for _ in range(printed_match_lines):
                    sys.stdout.write("\x1b[2K\n")
                sys.stdout.write(f"\x1b[{printed_match_lines}F")
            for line in status_lines:
                sys.stdout.write(f"\x1b[2K{line}\n")
            sys.stdout.flush()
            printed_match_status = True
            printed_match_lines = len(status_lines)
            curr_pos, _, curr_gripper_pos, _, _, _ = self.get_leader_joint_states()
            time.sleep(0.25)
        if printed_match_status:
            print()
        self.get_logger().info(f"FACTR TELEOP {self.name}: Initial joint position matched.")

    def _jog_to_end_pose(self):
        """
        Slowly drives the leader arm to self.end_pose (in the follower joint
        convention) to park it on shutdown, one joint-group at a time.

        The groups are moved in an order that keeps each motor's torque demand low
        (the weak leader motors saturate if the whole arm moves at once):
          1. base (id 1)              -- rotate while the arm is still tucked from the
                                         match pose, i.e. at low inertia
          2. wrists (ids 4, 5, 6)     -- light distal joints
          3. long links (ids 2, 3)    -- the heavy shoulder/elbow, moved last and
                                         alone so they get the full torque budget for
                                         the (high-gravity) extension
        Every other joint is actively held (gravity comp + PD) during each stage so
        nothing sags.
        """
        # (label, arm-joint indices [= Dynamixel id - 1], per-stage timeout seconds)
        stages = [
            ("base", [0], 4.0),
            ("wrists", [3, 4, 5], 5.0),
            ("long links", [1, 2], 6.0),
        ]
        # Shutdown runs after rclpy tears its context down, so get_logger() can no
        # longer publish to rosout; use print() so these messages still show up.
        for label, idx, stage_timeout in stages:
            idx = [i for i in idx if i < self.num_arm_joints]
            if not idx:
                continue
            ids = [i + 1 for i in idx]
            print(f"FACTR TELEOP {self.name}: homing {label} (Dynamixel {ids}).")
            reached = self._drive_joint_group(idx, stage_timeout)
            if not reached:
                print(f"FACTR TELEOP {self.name}: {label} did not fully reach target.")
        print(f"FACTR TELEOP {self.name}: return-home sequence complete.")

    def _return_home_gain(self, jpc, key, default):
        """
        Reads a return-home gain from the config as a per-joint array. The value may
        be a scalar (broadcast to every arm joint) or a list of num_arm_joints
        entries, so a single joint whose motor differs from the rest can be tuned
        without changing the others.
        """
        value = np.asarray(jpc.get(key, default), dtype=float)
        if value.ndim == 0:
            return np.full(self.num_arm_joints, float(value))
        assert value.shape == (self.num_arm_joints,), (
            f"{key} must be a scalar or a list of {self.num_arm_joints} values, "
            f"got shape {value.shape}"
        )
        return value

    def _drive_joint_group(self, active_idx, timeout, tol=0.08):
        """
        Drives the joints in `active_idx` to their self.end_pose targets while
        holding all other arm joints at their current position. A virtual per-joint
        setpoint advances the active joints toward the goal at return_home_speed
        (rad/s); every joint (moving or held) is PD-controlled with gravity
        compensation, using the stronger return_home_kp/kd position gains so the
        motion actually reaches the target rather than stalling short like the weak
        teleop gains would. Returns True if the active joints reached `tol` rad of
        their targets, False on timeout.
        """
        jpc = self.config["controller"]["joint_position_control"]
        # Homing is a PI controller (+ gravity comp), NOT a stiff PD: the shutdown
        # loop runs at ~110 Hz, well under the ~200 Hz a strong PD needs to stay
        # stable, so a high kp oscillates. Instead we keep kp low (stable) and let a
        # slow integral term ramp the torque until the joint reaches the goal,
        # overcoming friction without a high proportional gain. kd defaults to 0
        # because the Dynamixels' laggy velocity makes damping destabilizing here.
        #
        # Each gain may be a scalar (same value for every joint) or a list of
        # num_arm_joints values. Per-joint values exist because the arms do not use
        # identical Dynamixels: e.g. the right arm's shoulder is an XM430-W350-T
        # where the left's is an XM430-W210-T, so the same commanded Nm maps to a
        # different current (see TORQUE_TO_CURRENT_MAPPING) and the same ki/i_clamp
        # winds up to a much larger torque before stiction breaks -- which shows up
        # as the shoulder jerking rather than gliding to the goal.
        kp = self._return_home_gain(jpc, "return_home_kp", 0.5)
        kd = self._return_home_gain(jpc, "return_home_kd", 0.0)
        ki = self._return_home_gain(jpc, "return_home_ki", 1.0)
        # anti-windup: max integral torque
        i_clamp = self._return_home_gain(jpc, "return_home_i_clamp", 2.0)
        speed = self._return_home_gain(jpc, "return_home_speed", 0.5)
        goal = self.end_pose[0:self.num_arm_joints]

        active = np.zeros(self.num_arm_joints, dtype=bool)
        active[active_idx] = True

        curr_pos, curr_vel, _, _, curr_pos_sim, curr_vel_sim = self.get_leader_joint_states()
        setpoint = curr_pos.copy()  # held joints stay here; active ones march to goal
        integral = np.zeros(self.num_arm_joints)
        start_t = time.monotonic()
        prev_t = start_t
        n_loops = 0
        # No time.sleep(): the serial round-trips already pace the loop, so we run as
        # fast as the hardware allows to keep the control rate as high as possible.
        while time.monotonic() - start_t < timeout:
            now = time.monotonic()
            dt_real = now - prev_t
            prev_t = now
            n_loops += 1

            # Advance only the active joints' setpoints toward the goal, by at most
            # `speed * dt` this cycle, so they travel at `speed` rad/s in real time.
            to_goal = (goal - setpoint) * active
            max_step = speed * dt_real
            setpoint = setpoint + np.clip(to_goal, -max_step, max_step)

            err = setpoint - curr_pos
            # Integrate only the active joints; clamp to bound the wind-up torque.
            integral = np.clip(integral + err * dt_real * active, -i_clamp, i_clamp)
            torque_arm = (kp * err + ki * integral - kd * curr_vel) * active
            if self.enable_gravity_comp:
                torque_arm += self.gravity_compensation(curr_pos_sim, curr_vel_sim)
            self.set_leader_joint_torque(torque_arm, 0.0)

            curr_pos, curr_vel, _, _, curr_pos_sim, curr_vel_sim = self.get_leader_joint_states()
            if np.abs((goal - curr_pos)[active]).max() < tol:
                hz = n_loops / max(time.monotonic() - start_t, 1e-6)
                print(f"FACTR TELEOP {self.name}: stage reached target; loop rate {hz:.0f} Hz.")
                return True
        hz = n_loops / max(time.monotonic() - start_t, 1e-6)
        print(f"FACTR TELEOP {self.name}: stage timed out; loop rate {hz:.0f} Hz.")
        return False

    def shut_down(self):
        """
        Jogs the leader arm to the configured end pose, then disables all torque on
        the leader arm and gripper during node shutdown.

        Best-effort: any hardware/comm error (e.g. a transient Dynamixel syncwrite
        failure) is logged but never allowed to escape, so that disabling the
        Dynamixel torque mode -- the step that makes the arm safe -- always runs.
        """
        # Ctrl-C can raise KeyboardInterrupt inside an in-flight Dynamixel
        # transaction, leaving the port stuck (COMM_PORT_BUSY / stale syncwrite
        # params) so every call below would fail. Recover the port state first.
        # Shutdown runs after rclpy tears its context down, so get_logger() can no
        # longer publish to rosout; use print() so these messages still show up.
        try:
            self.driver.recover_port()
        except Exception as e:
            print(f"FACTR TELEOP {self.name}: failed to recover Dynamixel port ({e}).")
        try:
            self._jog_to_end_pose()
        except Exception as e:
            print(f"FACTR TELEOP {self.name}: failed to jog to end pose ({e}).")

        # The arm is now home: turn gravity compensation off so it no longer
        # actively holds itself up, then zero torque and disable the motors below.
        self.enable_gravity_comp = False
        print(f"FACTR TELEOP {self.name}: gravity compensation disabled.")

        try:
            self.set_leader_joint_torque(np.zeros(self.num_arm_joints), 0.0)
        except Exception as e:
            print(f"FACTR TELEOP {self.name}: failed to zero leader torque ({e}).")
        try:
            self.driver.set_torque_mode(False)
        except Exception as e:
            print(f"FACTR TELEOP {self.name}: failed to disable Dynamixel torque mode ({e}).")

    def _normalize_leader_arm_pos(self, joint_pos_arm):
        if not self.normalize_leader_joint_angles:
            return joint_pos_arm

        if self._leader_arm_pos_reference is None:
            reference = self.initial_match_joint_pos[0:self.num_arm_joints]
        else:
            reference = self._leader_arm_pos_reference

        joint_pos_arm = reference + np.arctan2(
            np.sin(joint_pos_arm - reference),
            np.cos(joint_pos_arm - reference),
        )
        self._leader_arm_pos_reference = joint_pos_arm.copy()
        return joint_pos_arm

    def get_leader_joint_states(self):
        """
        Returns the current joint positions and velocities of the leader arm and gripper,
        aligned with the joint conventions (range and direction) of the follower arm, plus
        the arm joint position/velocity in the URDF/pin_model's own convention (for gravity
        compensation and null-space regulation -- see _prepare_inverse_dynamics()).
        """
        self.gripper_pos_prev = self.gripper_pos
        joint_pos, joint_vel = self.driver.get_positions_and_velocities()
        joint_pos_arm = (
            joint_pos[0:self.num_arm_joints] - self.joint_offsets[0:self.num_arm_joints]
        ) * self.joint_signs[0:self.num_arm_joints]
        joint_pos_arm = self._normalize_leader_arm_pos(joint_pos_arm)
        self.gripper_pos = (joint_pos[-1] - self.joint_offsets[-1]) * self.joint_signs[-1]
        joint_vel_arm = joint_vel[0:self.num_arm_joints] * self.joint_signs[0:self.num_arm_joints]

        gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt

        joint_pos_arm_sim = (
            joint_pos[0:self.num_arm_joints] - self.sim_joint_offsets[0:self.num_arm_joints]
        ) * self.sim_joint_signs
        joint_vel_arm_sim = joint_vel[0:self.num_arm_joints] * self.sim_joint_signs

        return joint_pos_arm, joint_vel_arm, self.gripper_pos, gripper_vel, joint_pos_arm_sim, joint_vel_arm_sim
    
    def set_leader_joint_pos(self, goal_joint_pos, goal_gripper_pos):
        """
        Moves the leader arm and gripper to a specified joint configuration using a PD control loop.
        This method is useful for aligning the leader arm with a desired configuration, such as 
        matching the follower arm's configuration. It interpolates the motion toward the target 
        position and applies torque commands based on a PD controller.

        **Note:** This function is not used by default in the main teleoperation loop. To ensure 
        controller stability, please ensure the latency of Dynamixel servos is minimized such
        that the control loop frequency is at least 200 Hz. Otherwise, the PD controller tuning 
        is unstable for low control frequencies.
        """
        interpolation_step_size = np.ones(7)*self.config["controller"]["interpolation_step_size"]
        kp = self.config["controller"]["joint_position_control"]["kp"]
        kd = self.config["controller"]["joint_position_control"]["kd"]

        curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel, _, _ = self.get_leader_joint_states()
        while (np.linalg.norm(curr_pos - goal_joint_pos) > 0.1):
            next_joint_pos_target = np.where(
                np.abs(curr_pos - goal_joint_pos) > interpolation_step_size, 
                curr_pos + interpolation_step_size*np.sign(goal_joint_pos-curr_pos),
                goal_joint_pos,
            )
            torque = -kp*(curr_pos-next_joint_pos_target)-kd*(curr_vel)
            gripper_torque = -kp*(curr_gripper_pos-goal_gripper_pos)-kd*(curr_gripper_vel)
            self.set_leader_joint_torque(torque, gripper_torque)
            curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel, _, _ = self.get_leader_joint_states()
    
    def set_leader_joint_torque(self, arm_torque, gripper_torque):
        """
        Applies torque to the leader arm and gripper.
        """
        arm_gripper_torque = np.append(arm_torque, gripper_torque)
        self.driver.set_torque(arm_gripper_torque*self.joint_signs)


    def joint_limit_barrier(self, arm_joint_pos, arm_joint_vel, gripper_joint_pos, gripper_joint_vel):
        """
        Computes joint limit repulsive torque to prevent the leader arm and gripper from 
        exceeding the physical joint limits of the follower arm.

        This method implements a simplified control law compared to the one described in 
        Section IX.B of the paper, while achieving the same protective effect. It applies 
        repulsive torques proportional to the distance from the joint limits and the joint 
        velocity when limits are approached or exceeded.
        """
        exceed_max_mask = arm_joint_pos > self.arm_joint_limits_max
        tau_l = (-self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_max) \
            - self.joint_limit_kd * arm_joint_vel) * exceed_max_mask
        exceed_min_mask = arm_joint_pos < self.arm_joint_limits_min
        tau_l += (-self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_min) \
            - self.joint_limit_kd * arm_joint_vel) * exceed_min_mask
        """
        if gripper_joint_pos > self.gripper_limit_max:
            tau_l_gripper = -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_max) \
                - self.joint_limit_kd * gripper_joint_vel
        elif gripper_joint_pos < self.gripper_limit_min:
            tau_l_gripper = -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_min) \
                - self.joint_limit_kd * gripper_joint_vel
        else:
            tau_l_gripper = 0.0
            """
        tau_l_gripper = 0.0
        return tau_l, tau_l_gripper

    def gravity_compensation(self, arm_joint_pos_sim, arm_joint_vel_sim):
        """
        Computes joint torque for gravity compensation using inverse dynamics.
        This method uses the Recursive Newton-Euler Algorithm (RNEA), provided by the
        Pinocchio library, to calculate the torques required to counteract gravity
        at the current joint states. The result is scaled by a modifier to tune the
        compensation strength.

        This implementation corresponds to the gravity compensation strategy
        described in Section III.C of the paper.

        arm_joint_pos_sim/arm_joint_vel_sim must be in the URDF/pin_model's own
        joint convention (see get_leader_joint_states()); the resulting torque is
        converted back to the follower joint convention (via
        sim_to_follower_torque_sign) before being returned, since torque_arm is
        accumulated and applied in that convention by set_leader_joint_torque().
        """
        tau_g_sim = pin.rnea(
            self.pin_model, self.pin_data,
            arm_joint_pos_sim, arm_joint_vel_sim, np.zeros_like(arm_joint_vel_sim)
        )
        self.tau_g = tau_g_sim * self.sim_to_follower_torque_sign * self.gravity_comp_modifier
        return self.tau_g

    def friction_compensation(self, arm_joint_vel):
        """
        Compute joint torques to compensate for static friction during teleoperation.

        This method implements static friction compensation as described in Equation 7,
        Section IX.A of the paper. It omits kinetic friction compensation, which was 
        necessary in earlier hardware versions to achieve smooth teleoperation, but has 
        since become unnecessary due to hardware improvements, such as weight reduction. 
        """
        tau_ss = np.zeros(self.num_arm_joints)
        for i in range(self.num_arm_joints):
            if abs(arm_joint_vel[i]) < self.stiction_comp_enable_speed:
                if self.stiction_dither_flag[i]:
                    tau_ss[i] += self.stiction_comp_gain * abs(self.tau_g[i])
                else:
                    tau_ss[i] -= self.stiction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither_flag[i] = ~self.stiction_dither_flag[i]
        return tau_ss
    
    def null_space_regulation(self, arm_joint_pos_sim, arm_joint_vel_sim):
        """
        Computes joint torques to perform null-space regulation for redundancy resolution
        of the leader arm.

        This method enables the specification of a desired null-space joint configuration
        via `self.null_space_joint_target`. It implements the control strategy described
        in Equation 3 of Section III.B in the paper, projecting a PD control law into
        the null space of the task Jacobian to achieve secondary objectives without
        affecting the primary task.

        arm_joint_pos_sim/arm_joint_vel_sim must be in the URDF/pin_model's own joint
        convention (see get_leader_joint_states() and gravity_compensation()); note
        `null_space_joint_target` must therefore also be specified in that same
        convention. The resulting torque is converted back to the follower joint
        convention before being returned.
        """
        J = pin.computeJointJacobian(
            self.pin_model, self.pin_data, arm_joint_pos_sim, self.num_arm_joints
        )
        J_dagger = np.linalg.pinv(J)
        null_space_projector = np.eye(self.num_arm_joints) - J_dagger @ J
        q_error = arm_joint_pos_sim - self.null_space_joint_target[0:self.num_arm_joints]
        tau_n_sim = null_space_projector @ (
            -self.null_space_kp*q_error - self.null_space_kd*arm_joint_vel_sim
        )
        return tau_n_sim * self.sim_to_follower_torque_sign
    
    def torque_feedback(self, external_torque, arm_joint_vel):
        """
        Computes joint torque for the leader arm to achieve force-feedback based on
        the external joint torque from the follower arm.

        This method implements Equation 1 in Section III.A of the paper.
        """
        tau_ff = -1.0*self.torque_feedback_gain/self.torque_feedback_motor_scalar * external_torque
        tau_ff -= self.torque_feedback_damping*arm_joint_vel
        return tau_ff

    def control_loop_callback(self):
        """
        Runs the main control loop of the leader arm. 

        Note that while the control loop can run at up to 500 Hz, lower frequencies 
        such as 200 Hz can still yield comparable performance, although they may 
        require additional tuning of control parameters. For Dynamixel servos to 
        support a 500 Hz control frequency, ensure that the Baud Rate is set to 4 Mbps 
        and the Return Delay Time is set to 0 using the Dynamixel Wizard software.
        """
        (
            leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel,
            leader_arm_pos_sim, leader_arm_vel_sim,
        ) = self.get_leader_joint_states()

        torque_arm = np.zeros(self.num_arm_joints)
        torque_l, torque_gripper = self.joint_limit_barrier(
            leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel
        )
        torque_arm += torque_l
        if self.null_space_kp != 0.0 or self.null_space_kd != 0.0:
            torque_arm += self.null_space_regulation(leader_arm_pos_sim, leader_arm_vel_sim)

        if self.enable_gravity_comp:
            torque_arm += self.gravity_compensation(leader_arm_pos_sim, leader_arm_vel_sim)
            torque_arm += self.friction_compensation(leader_arm_vel)
        
        if self.enable_torque_feedback:
            external_joint_torque = self.get_leader_arm_external_joint_torque()
            torque_arm += self.torque_feedback(external_joint_torque, leader_arm_vel)
        
        if self.enable_gripper_feedback:
            gripper_feedback = self.get_leader_gripper_feedback()
            torque_gripper += self.gripper_feedback(leader_gripper_pos, leader_gripper_vel, gripper_feedback)
        
        torque_gripper += self.config['gripper_teleop']['spring_constant'] * (1 - leader_gripper_pos)

        self.set_leader_joint_torque(torque_arm, torque_gripper)
        self.update_communication(leader_arm_pos, leader_gripper_pos)
        self._publish_leader_loop_hz()

    def _publish_leader_loop_hz(self):
        self._leader_loop_count_since_status += 1
        now_mono = time.monotonic()
        status_dt = now_mono - self._leader_loop_last_status_t
        if status_dt < 1.0:
            return
        msg = Float32()
        msg.data = float(self._leader_loop_count_since_status / status_dt)
        self.leader_hz_pub.publish(msg)
        self._leader_loop_count_since_status = 0
        self._leader_loop_last_status_t = now_mono


    @abstractmethod
    def set_up_communication(self):
        """
        This method should be implemented to set up communication between the leader arm
        and the follower arm for bilateral teleoperation. This method is called once
        in the __init__ method.
        
        For example, a subscriber can  be set up to receive external joint torque from 
        the leader arm and a publisher can be set up to send joint position target commands 
        to the follower arm. Publishers and subscribers can also be set up to record
        the follower arm's joint states

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def get_leader_arm_external_joint_torque(self):
        """
        This method should retrieve the current external joint torque from the follower arm.
        This is used to compute force-feedback in the leader arm. This method is called at
        every iteration of the control loop if self.enable_torque_feedback is set to True.

        Returns:
            np.ndarray: A NumPy array of shape (num_arm_joints,) containing the external 
            joint torques. 

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def get_leader_gripper_feedback(self):
        """
        This method should retrieve any data from the follower gripper that might be required
        to achieve force-feedback in the leader gripper. For example, this method can be used
        to get the current position of the follower gripper for position-position force-feedback
        or the current force of the follower gripper for position-force force-feedback in the
        leader gripper. This method is called at every iteration of the control loop if 
        self.enable_gripper_feedback is set to True.

        Returns:
            Any: Feedback data required by the leader gripper. This can be a NumPy array, a 
            scalar, or any other data type depending on the implementation.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        """
        Processes feedback data from the follower gripper. This method is intended to compute 
        force-feedback for the leader gripper. This method is called at every iteration of the 
        control loop if self.enable_gripper_feedback is set to True.

        Args:
            leader_gripper_pos (float): Leader gripper position. Can be used to provide force-
            feedback for the gripper.
            leader_gripper_vel (float): Leader gripper velocity. Can be used to provide force-
            feedback for the gripper.
            gripper_feedback (Any): Feedback data from the gripper. The format can vary depending 
            on the implementation, such as a NumPy array, scalar, or custom object.
        
        Returns:
            float: The computed joint torque value to apply force-feedback to the leader gripper.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        """
        This method is intended to be called at every iteration of the control loop to transmit 
        relevant data, such as joint position targets, from the leader to the follower arm.

        Args:
            leader_arm_pos (np.ndarray): A NumPy array containing the joint positions of the leader arm.
            leader_gripper_pos (np.ndarray): A NumPy array containing the position of the leader gripper.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass
