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
# UR7e policy-rollout in SIMULATION. Same as rollout.py but the real UR7e drive
# node and the two RealSense camera nodes are replaced by a single ur_sim bridge
# node (factr_teleop_ur7e_sim), which steps a MuJoCo world and republishes the
# exact obs + camera topics the policy consumes. The policy node is identical to
# the hardware launch -- it cannot tell it is talking to sim.
#
# No pendant / ExternalControl / initial-pose jog is needed: the sim resets to a
# known pose on startup.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value="/home/leo/FACTR/checkpoints/test/rollout",
        description="Absolute path to the trained rollout checkpoint directory.",
    )
    show_viewer_arg = DeclareLaunchArgument(
        "show_viewer",
        default_value="true",
        description="Open a live MuJoCo viewer window during the sim rollout.",
    )

    ur7e_sim_node = Node(
        package="factr_teleop",
        executable="factr_teleop_ur7e_sim",
        name="factr_teleop_ur7e_sim",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"name": "left"},
            {"control_hz": 30.0},
            {"show_viewer": LaunchConfiguration("show_viewer")},
        ],
    )

    policy_rollout_node = Node(
        package="bc",
        executable="policy_rollout",
        name="policy_rollout_node",
        output="screen",
        parameters=[
            {"save_data": False},
            {"data_dir": LaunchConfiguration("data_dir")},
        ],
    )

    return LaunchDescription([
        data_dir_arg,
        show_viewer_arg,
        ur7e_sim_node,
        policy_rollout_node,
    ])
