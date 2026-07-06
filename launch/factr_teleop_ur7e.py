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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Which leader config to load. Override for the other arm, e.g.
    #   ros2 launch launch/factr_teleop_ur7e.py config_file:=ur7e_leader_right.yaml
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="ur7e_leader_left.yaml",
        description="Config filename in factr_teleop/configs to load.",
    )
    # Node name. Keep distinct per arm so two can run at once (left/right).
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="factr_teleop_ur7e",
        description="ROS node name (use distinct names for dual-arm).",
    )
    leader_match_only_arg = DeclareLaunchArgument(
        "leader_match_only",
        default_value="false",
        description="Only print leader start-match errors and exit after matching.",
    )
    collision_safety_arg = DeclareLaunchArgument(
        "collision_safety",
        default_value="false",
        description="Use /factr_teleop/{left,right}/safe_ur_pos from an external safe-target safety source.",
    )
    safe_target_timeout_arg = DeclareLaunchArgument(
        "safe_target_timeout",
        default_value="0.25",
        description="Hold current UR pose if safe_ur_pos is older than this many seconds.",
    )

    factr_teleop_ur7e = Node(
        package="factr_teleop",
        executable="factr_teleop_ur7e",
        name=LaunchConfiguration("node_name"),
        output="screen",
        emulate_tty=True,
        # On Ctrl-C the node slowly returns the leader arm home before exiting.
        # Give it time to finish before launch escalates SIGINT -> SIGTERM -> SIGKILL
        # (defaults are only 5 s each, which would force-kill mid-return).
        sigterm_timeout="22.0",
        sigkill_timeout="27.0",
        parameters=[
            {"config_file": LaunchConfiguration("config_file")},
            {
                "leader_match_only": ParameterValue(
                    LaunchConfiguration("leader_match_only"),
                    value_type=bool,
                )
            },
            {
                "collision_safety": ParameterValue(
                    LaunchConfiguration("collision_safety"),
                    value_type=bool,
                )
            },
            {
                "safe_target_timeout": ParameterValue(
                    LaunchConfiguration("safe_target_timeout"),
                    value_type=float,
                )
            },
        ],
    )

    return LaunchDescription([
        config_file_arg,
        node_name_arg,
        leader_match_only_arg,
        collision_safety_arg,
        safe_target_timeout_arg,
        factr_teleop_ur7e,
    ])
