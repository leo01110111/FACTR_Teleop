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
    input_endpoint_arg = DeclareLaunchArgument(
        "input_endpoint",
        default_value="tcp://127.0.0.1:5558",
        description="ZMQ endpoint where the Isaac stream server receives latest state/desired inputs.",
    )
    output_endpoint_arg = DeclareLaunchArgument(
        "output_endpoint",
        default_value="tcp://127.0.0.1:5559",
        description="ZMQ endpoint where the Isaac stream server publishes safe targets.",
    )
    publish_hz_arg = DeclareLaunchArgument(
        "publish_hz",
        default_value="500.0",
        description="ROS-to-Isaac latest-input publish rate in Hz.",
    )
    state_timeout_s_arg = DeclareLaunchArgument(
        "state_timeout_s",
        default_value="1.00",
        description="Maximum age for /ur/<side>/obs_ur_state before holding.",
    )
    desired_timeout_s_arg = DeclareLaunchArgument(
        "desired_timeout_s",
        default_value="1.00",
        description="Maximum age for desired_ur_pos before holding.",
    )
    safe_response_timeout_s_arg = DeclareLaunchArgument(
        "safe_response_timeout_s",
        default_value="5.00",
        description="Maximum age for streamed Isaac safe output before holding.",
    )
    max_joint_step_rad_arg = DeclareLaunchArgument(
        "max_joint_step_rad",
        default_value="0.05",
        description="Maximum per-controller-step safe target step requested from Isaac.",
    )
    max_safe_target_distance_rad_arg = DeclareLaunchArgument(
        "max_safe_target_distance_rad",
        default_value="0.05",
        description="Reject streamed q_safe if it is farther than this from current q.",
    )
    max_sequence_lag_arg = DeclareLaunchArgument(
        "max_sequence_lag",
        default_value="2",
        description="Reject streamed responses older than this many bridge input sequences.",
    )
    publish_safe_targets_arg = DeclareLaunchArgument(
        "publish_safe_targets",
        default_value="false",
        description="If false, run shadow mode and do not publish safe_ur_pos.",
    )
    require_rmp_policy_arg = DeclareLaunchArgument(
        "require_rmp_policy",
        default_value="true",
        description="If true, active safe-target publishing rejects non-RMP stream policies.",
    )
    hold_stale_state_arg = DeclareLaunchArgument(
        "hold_stale_state",
        default_value="true",
        description="If true, keep streaming the last known q for a side whose state update is late.",
    )
    hold_stale_desired_arg = DeclareLaunchArgument(
        "hold_stale_desired",
        default_value="true",
        description="If true, hold current q for a side whose desired target is missing/stale instead of dropping all input.",
    )

    bridge = Node(
        package="factr_teleop",
        executable="isaac_rmpflow_stream_bridge",
        name="isaac_rmpflow_stream_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"active_sides": LaunchConfiguration("active_sides")},
            {"input_endpoint": LaunchConfiguration("input_endpoint")},
            {"output_endpoint": LaunchConfiguration("output_endpoint")},
            {"publish_hz": ParameterValue(LaunchConfiguration("publish_hz"), value_type=float)},
            {"state_timeout_s": ParameterValue(LaunchConfiguration("state_timeout_s"), value_type=float)},
            {"desired_timeout_s": ParameterValue(LaunchConfiguration("desired_timeout_s"), value_type=float)},
            {
                "safe_response_timeout_s": ParameterValue(
                    LaunchConfiguration("safe_response_timeout_s"),
                    value_type=float,
                )
            },
            {"max_joint_step_rad": ParameterValue(LaunchConfiguration("max_joint_step_rad"), value_type=float)},
            {
                "max_safe_target_distance_rad": ParameterValue(
                    LaunchConfiguration("max_safe_target_distance_rad"),
                    value_type=float,
                )
            },
            {"max_sequence_lag": ParameterValue(LaunchConfiguration("max_sequence_lag"), value_type=int)},
            {"publish_safe_targets": ParameterValue(LaunchConfiguration("publish_safe_targets"), value_type=bool)},
            {"require_rmp_policy": ParameterValue(LaunchConfiguration("require_rmp_policy"), value_type=bool)},
            {"hold_stale_state": ParameterValue(LaunchConfiguration("hold_stale_state"), value_type=bool)},
            {"hold_stale_desired": ParameterValue(LaunchConfiguration("hold_stale_desired"), value_type=bool)},
        ],
    )

    return LaunchDescription([
        active_sides_arg,
        input_endpoint_arg,
        output_endpoint_arg,
        publish_hz_arg,
        state_timeout_s_arg,
        desired_timeout_s_arg,
        safe_response_timeout_s_arg,
        max_joint_step_rad_arg,
        max_safe_target_distance_rad_arg,
        max_sequence_lag_arg,
        publish_safe_targets_arg,
        require_rmp_policy_arg,
        hold_stale_state_arg,
        hold_stale_desired_arg,
        bridge,
    ])
