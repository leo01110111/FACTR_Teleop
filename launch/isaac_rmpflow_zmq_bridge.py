from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    active_sides_arg = DeclareLaunchArgument(
        "active_sides",
        default_value="right",
        description="Comma-separated sides to bridge: left, right, or left,right.",
    )
    isaac_endpoint_arg = DeclareLaunchArgument(
        "isaac_endpoint",
        default_value="tcp://127.0.0.1:5557",
        description="ZMQ endpoint for the Isaac/Lula RMPFlow server.",
    )
    request_hz_arg = DeclareLaunchArgument(
        "request_hz",
        default_value="100.0",
        description="Bridge request rate in Hz.",
    )
    state_timeout_s_arg = DeclareLaunchArgument(
        "state_timeout_s",
        default_value="0.10",
        description="Maximum age for /ur/<side>/obs_ur_state before holding.",
    )
    desired_timeout_s_arg = DeclareLaunchArgument(
        "desired_timeout_s",
        default_value="0.10",
        description="Maximum age for desired_ur_pos before holding.",
    )
    isaac_response_timeout_s_arg = DeclareLaunchArgument(
        "isaac_response_timeout_s",
        default_value="0.05",
        description="Maximum wait for Isaac server response before holding.",
    )
    max_joint_step_rad_arg = DeclareLaunchArgument(
        "max_joint_step_rad",
        default_value="0.05",
        description="Maximum per-cycle safe target step accepted from Isaac.",
    )
    publish_safe_targets_arg = DeclareLaunchArgument(
        "publish_safe_targets",
        default_value="true",
        description="If false, run shadow mode and do not publish safe_ur_pos.",
    )

    bridge = Node(
        package="factr_teleop",
        executable="isaac_rmpflow_zmq_bridge",
        name="isaac_rmpflow_zmq_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"active_sides": LaunchConfiguration("active_sides")},
            {"isaac_endpoint": LaunchConfiguration("isaac_endpoint")},
            {"request_hz": ParameterValue(LaunchConfiguration("request_hz"), value_type=float)},
            {"state_timeout_s": ParameterValue(LaunchConfiguration("state_timeout_s"), value_type=float)},
            {"desired_timeout_s": ParameterValue(LaunchConfiguration("desired_timeout_s"), value_type=float)},
            {
                "isaac_response_timeout_s": ParameterValue(
                    LaunchConfiguration("isaac_response_timeout_s"),
                    value_type=float,
                )
            },
            {"max_joint_step_rad": ParameterValue(LaunchConfiguration("max_joint_step_rad"), value_type=float)},
            {"publish_safe_targets": ParameterValue(LaunchConfiguration("publish_safe_targets"), value_type=bool)},
        ],
    )

    return LaunchDescription([
        active_sides_arg,
        isaac_endpoint_arg,
        request_hz_arg,
        state_timeout_s_arg,
        desired_timeout_s_arg,
        isaac_response_timeout_s_arg,
        max_joint_step_rad_arg,
        publish_safe_targets_arg,
        bridge,
    ])
