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
# UR7e SIM ROLLOUT DRIVE NODE.
#
# Drop-in replacement for factr_teleop_ur7e_rollout.py + the two RealSense camera
# nodes, so bc/policy_rollout.py can drive the ur_sim MuJoCo world instead of a
# real UR7e. It speaks the exact same topic contract, so policy_rollout.py is
# untouched:
#
#   subscribes  /factr_teleop/left/cmd_ur_pos       (JointState, 6 joints [rad])
#               /factr_teleop/left/cmd_gripper_pos  (JointState, 1 Robotiq 0..255)
#   publishes   /ur/left/obs_ur_state    (JointState, 6 joint positions [rad])
#               /ur/left/obs_gripper     (JointState, [pos255, current255])
#               /ur/left/obs_ur_wrench   (JointState, 6 TCP wrench [N,Nm])
#               /realsense/left/im       (Image, rgb8)   <- sim left wrist cam
#               /realsense/top/im        (Image, rgb8)   <- sim top1 cam
#
# Only the LEFT arm is driven by the policy; the right arm + right gripper are
# held at their reset pose (the checkpoint here is single-arm left). The sim is
# stepped on the ROS timer at the policy control rate (default 30 Hz).

import numpy as np

import mujoco

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from ur_sim.env import SimBimanualUR7eEnv

# Publish sim cameras at the real RealSense color resolution so the policy's own
# 224x224 squash-resize (bc/policy_rollout.process_image) reproduces the exact
# preprocessing the checkpoint was trained on.
CAM_W, CAM_H = 1280, 720

# Inlined from factr_teleop_ur7e.create_array_msg -- importing that module here
# would drag in the pinocchio/Dynamixel leader stack, and pinocchio segfaults
# under this env's NumPy 2.x. The sim bridge needs none of it.
def create_array_msg(data):
    msg = JointState()
    msg.position = list(map(float, data))
    return msg


# Left-arm actuator layout in the sim's 14-vector, base->wrist3 (matches UR RTDE
# order, so cmd_ur_pos maps 1:1 with no reindex). Resolved from actuator names at
# startup rather than hard-coded, so a model change can't silently misalign them.
def _resolve_indices(model):
    names = [model.actuator(i).name for i in range(model.nu)]
    order = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
    left_arm = [names.index(f"left_{j}") for j in order]
    left_grip = next(i for i, n in enumerate(names) if n.startswith("left") and "grip" in n)
    return left_arm, left_grip


def _wrench(model, data, side="left"):
    f = model.sensor(f"{side}_ft_force")
    t = model.sensor(f"{side}_ft_torque")
    force = data.sensordata[f.adr[0]:f.adr[0] + 3]
    torque = data.sensordata[t.adr[0]:t.adr[0] + 3]
    return np.concatenate([force, torque])


class FACTRTeleopUR7eSim(Node):
    def __init__(self):
        super().__init__("factr_teleop_ur7e_sim")

        self.name = self.declare_parameter("name", "left").value
        self.control_hz = float(self.declare_parameter("control_hz", 30.0).value)
        show_viewer = bool(self.declare_parameter("show_viewer", False).value)

        self.env = SimBimanualUR7eEnv(
            normalized_actions=False,
            control_hz=self.control_hz,
            max_episode_steps=10 ** 9,  # rollout is continuous; never self-truncate
            show_viewer=show_viewer,
        )
        self.env.reset()
        self.model, self.data = self.env.model, self.env.data
        self._left_arm, self._left_grip = _resolve_indices(self.model)

        # Hold action starts at the reset ctrl; only the left slots are overwritten
        # by policy commands, so the right arm + right gripper stay put.
        self._action = self.data.ctrl.copy()

        # Let the arm settle at the hold pose, then tare the wrist F/T, mirroring
        # the UR zeroFtSensor() done on real hardware after mounting the tool.
        # Taring at the raw reset instant leaves a ~10 N residual as the PD loop
        # pulls into the pose; settling first zeroes the no-contact reading.
        for _ in range(30):
            self.env.step(self._action)
        self._wrench_bias = _wrench(self.model, self.data).copy()

        # Dedicated 1280x720 renderer (the env's own renderer is square 224x224).
        self._renderer = mujoco.Renderer(self.model, height=CAM_H, width=CAM_W)

        self.obs_ur_state_pub = self.create_publisher(
            JointState, f"/ur/{self.name}/obs_ur_state", 10)
        self.obs_ur_wrench_pub = self.create_publisher(
            JointState, f"/ur/{self.name}/obs_ur_wrench", 10)
        self.obs_gripper_pub = self.create_publisher(
            JointState, f"/ur/{self.name}/obs_gripper", 10)
        self.left_im_pub = self.create_publisher(Image, "/realsense/left/im", 10)
        self.top_im_pub = self.create_publisher(Image, "/realsense/top/im", 10)

        self.create_subscription(
            JointState, f"/factr_teleop/{self.name}/cmd_ur_pos",
            self._cmd_ur_pos_cb, 10)
        self.create_subscription(
            JointState, f"/factr_teleop/{self.name}/cmd_gripper_pos",
            self._cmd_gripper_pos_cb, 10)

        self.create_timer(1.0 / self.control_hz, self._step)
        self.get_logger().info(
            f"ur_sim bridge up: driving '{self.name}' arm at {self.control_hz} Hz "
            f"(left actuators {self._left_arm}, gripper {self._left_grip})")

    def _cmd_ur_pos_cb(self, msg):
        q = np.asarray(msg.position[:6], dtype=np.float64)
        if q.shape[0] == 6:
            self._action[self._left_arm] = q

    def _cmd_gripper_pos_cb(self, msg):
        if len(msg.position) >= 1:
            self._action[self._left_grip] = float(np.clip(msg.position[0], 0.0, 255.0))

    def _step(self):
        self.env.step(self._action)

        proprio = self.env._proprio()
        q_left = proprio[self._left_arm]
        grip_pos_255 = float(self._action[self._left_grip])  # commanded == actual proxy
        wrench = _wrench(self.model, self.data) - self._wrench_bias

        self.obs_ur_state_pub.publish(create_array_msg(q_left.tolist()))
        self.obs_ur_wrench_pub.publish(create_array_msg(wrench.tolist()))
        # 0 current: the sim has no gripper current sensor.
        self.obs_gripper_pub.publish(create_array_msg([grip_pos_255, 0.0]))

        self.left_im_pub.publish(self._image_msg(self._render("left")))
        self.top_im_pub.publish(self._image_msg(self._render("top1")))

    def _render(self, role):
        self._renderer.update_scene(self.data, camera=self.env.cameras[role])
        return self._renderer.render()  # (720, 1280, 3) uint8 RGB

    def _image_msg(self, img):
        # Built directly rather than via cv_bridge, whose numpy type tables break
        # under this env's NumPy 2.x (cv2_to_imgmsg raises KeyError). img is a
        # contiguous (H, W, 3) uint8 RGB frame from the MuJoCo renderer.
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg

    def shut_down(self):
        try:
            self._renderer.close()
        except Exception:
            pass
        try:
            self.env.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FACTRTeleopUR7eSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutting down ur_sim bridge...")
    finally:
        node.shut_down()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
