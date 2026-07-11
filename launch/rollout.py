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
#
# UR7e policy-rollout launch. Brings up the full on-robot inference stack:
#
#   * two RealSense camera nodes -- left (D455) and top (D435) -- matching the
#     streams recorded during data collection (see record_data_left.sh):
#       realsense_left_cam.yaml -> /realsense/left/im
#       top_cam.yaml            -> /realsense/top/im
#   * the UR7e rollout-drive node, which SUBSCRIBES to the policy command topics
#     /factr_teleop/left/cmd_ur_pos (6 joints) and /factr_teleop/left/cmd_gripper_pos
#     (1 Robotiq 0..255 value) and servos the follower UR7e over ur_rtde
#     (directTorque PD + Robotiq URCap socket), while republishing the obs topics
#     the policy consumes. This is the rollout-time replacement for the GELLO
#     leader that drove the arm during teleoperation.
#   * the policy_rollout node, which loads the trained checkpoint, subscribes to
#     the obs/camera topics, runs inference, and publishes the two command topics.
#
# SAFETY / ORDER OF OPERATIONS:
#   1. On the pendant, load+Play the ExternalControl program for the LEFT arm.
#   2. Jog the UR to initial_match_joint_pos before launching, e.g.
#        ros2 run factr_teleop return_ur_to_initial_match \
#            --config-file ur7e_leader_left.yaml
#      The rollout-drive node refuses to start (to avoid a jump) if the arm is
#      more than 0.5 rad/joint from that pose.
#   3. Launch this file. The arm HOLDS its pose until the first cmd_ur_pos from
#      the policy arrives, then tracks the policy.
#
# The Franka bridge and ZED nodes from the upstream Franka pipeline are removed:
# this deployment uses a UR7e follower and two RealSense cameras.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="ur7e_leader_left.yaml",
        description="factr_teleop config used for the UR7e follower servo gains, "
                    "addresses, and initial_match_joint_pos.",
    )

    # Trained checkpoint directory (absolute). Contains latest_ckpt.ckpt,
    # agent_config.yaml, rollout_config.yaml.
    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value="/home/leo/FACTR/checkpoints/test/rollout",
        description="Absolute path to the trained rollout checkpoint directory.",
    )

    # ---- RealSense cameras (match record_data_left.sh) ----------------------
    realsense_left_node = Node(
        package="cameras",
        executable="realsense",
        name="realsense_left",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"config_file": "realsense_left_cam.yaml"},  # D455 -> /realsense/left/im
        ],
    )

    realsense_top_node = Node(
        package="cameras",
        executable="realsense",
        name="realsense_top",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"config_file": "top_cam.yaml"},  # D435 -> /realsense/top/im
        ],
    )

    # ---- UR7e rollout-drive node (consumes cmd_* topics, servos the arm) -----
    ur7e_rollout_node = Node(
        package="factr_teleop",
        executable="factr_teleop_ur7e_rollout",
        name="factr_teleop_ur7e_rollout",
        output="screen",
        emulate_tty=True,
        # On Ctrl-C the node zeros direct torque and releases RTDE cleanly; give it
        # a moment before launch escalates SIGINT -> SIGTERM -> SIGKILL.
        sigterm_timeout="8.0",
        sigkill_timeout="12.0",
        parameters=[
            {"config_file": LaunchConfiguration("config_file")},
        ],
    )

    # ---- Policy inference node ----------------------------------------------
    policy_rollout_node = Node(
        package="bc",
        executable="policy_rollout",
        name="policy_rollout_node",
        output="screen",
        parameters=[
            {"save_data": True},
            {"data_dir": LaunchConfiguration("data_dir")},
        ],
    )

    return LaunchDescription([
        config_file_arg,
        data_dir_arg,
        realsense_left_node,
        realsense_top_node,
        ur7e_rollout_node,
        policy_rollout_node,
    ])
