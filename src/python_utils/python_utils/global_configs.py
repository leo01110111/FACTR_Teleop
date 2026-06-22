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

#For the UR7e
# The UR arms are commanded directly over UR RTDE (ur_rtde): the UR control box
# is itself the follower controller (servoJ does the real-time interpolation/
# tracking internally), so there is no separate follower process and no ZMQ hop.
# We therefore only need each arm's own IP, the ExternalControl URCap port used
# by RTDEControlInterface, and the Robotiq 2F-85 gripper socket port.

ur_right_ip_address = "192.168.2.2"
ur_left_ip_address = "192.168.1.2"

ur_right_real_addresses = {
    "ip": ur_right_ip_address,
    "ur_cap_port": 50003,   # ExternalControl URCap "Custom port" on this arm
    "gripper_port": 63352,  # Robotiq 2F-85 socket exposed by the Robotiq URCap
}

ur_left_real_addresses = {
    "ip": ur_left_ip_address,
    "ur_cap_port": 50002,
    "gripper_port": 63352,
}

#For the Franka (from the original FACTR Teleop code)

sim_desktop_ip_address = "172.16.0.9"
franka_left_ip_address = "172.16.0.1"
franka_right_ip_address = "172.16.0.3"

franka_right_real_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_right_ip_address}:3099",
    "joint_torque_sub": f"tcp://{franka_right_ip_address}:3087",
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:2098",

}

franka_left_real_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_left_ip_address}:5099",
    "joint_torque_sub": f"tcp://{franka_left_ip_address}:5087",
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:4098",

}