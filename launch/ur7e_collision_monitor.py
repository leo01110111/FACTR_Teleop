from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    i2rt_path_arg = DeclareLaunchArgument(
        "i2rt_path",
        default_value="/home/srianumakonda/openpi-yam-maxlab/gello/i2rt",
        description="Path to the openpi-yam i2rt package root.",
    )
    scene_yaml_arg = DeclareLaunchArgument(
        "scene_yaml",
        default_value="/home/srianumakonda/openpi-yam-maxlab/gello/i2rt/i2rt/robot_models/ur7e_maxlab/scene.yaml",
        description="UR7e maxlab scene YAML for the openpi-yam safety controller.",
    )
    stale_timeout_arg = DeclareLaunchArgument(
        "stale_timeout",
        default_value="0.25",
        description="Fail-safe timeout for observed/desired arm states.",
    )
    rate_hz_arg = DeclareLaunchArgument(
        "rate_hz",
        default_value="500.0",
        description="Safety QP update rate in Hz.",
    )
    activation_mode_arg = DeclareLaunchArgument(
        "activation_mode",
        default_value="interarm_distance",
        description="QP activation mode: interarm_distance or clearance.",
    )
    interarm_activation_distance_arg = DeclareLaunchArgument(
        "interarm_activation_distance",
        default_value="0.15",
        description="Run the openpi-yam QP when current or desired interarm distance is below this many meters.",
    )
    interarm_release_distance_arg = DeclareLaunchArgument(
        "interarm_release_distance",
        default_value="0.18",
        description="Keep the QP latched until interarm distance rises above this many meters.",
    )
    release_velocity_limits_arg = DeclareLaunchArgument(
        "release_velocity_limits",
        default_value="2.0,2.0,2.0,3.0,3.0,3.0",
        description="Per-joint rad/s caps for catching back up to desired_q after QP release.",
    )
    safety_activation_clearance_arg = DeclareLaunchArgument(
        "safety_activation_clearance",
        default_value="0.10",
        description="Extra clearance in meters beyond safety margins before the QP engages in activation_mode:=clearance.",
    )
    active_sides_arg = DeclareLaunchArgument(
        "active_sides",
        default_value="left,right",
        description="Comma-separated sides to require/publish: left, right, or left,right.",
    )
    command_mode_arg = DeclareLaunchArgument(
        "command_mode",
        default_value="posture",
        description="Safety command mode: velocity or posture.",
    )
    velocity_tracking_gain_arg = DeclareLaunchArgument(
        "velocity_tracking_gain",
        default_value="8.0",
        description="Proportional catch-up gain added to desired leader joint velocity in velocity mode.",
    )
    left_wrist_3_offset_arg = DeclareLaunchArgument(
        "left_wrist_3_offset",
        default_value="1.57079632679",
        description="Real-to-sim offset added to left wrist_3 before the QP.",
    )
    right_wrist_3_offset_arg = DeclareLaunchArgument(
        "right_wrist_3_offset",
        default_value="3.14159265359",
        description="Real-to-sim offset added to right wrist_3 before the QP.",
    )
    left_fallback_q_arg = DeclareLaunchArgument(
        "left_fallback_q",
        default_value="1.5700,-1.5700,1.5700,-1.5700,-1.5700,-1.5700",
        description="Inactive left arm fallback joint pose in real UR joint coordinates.",
    )
    right_fallback_q_arg = DeclareLaunchArgument(
        "right_fallback_q",
        default_value="-1.5700,-1.5700,-1.5700,-1.5700,1.5700,0.0000",
        description="Inactive right arm fallback joint pose in real UR joint coordinates.",
    )

    monitor = Node(
        package="factr_teleop",
        executable="ur7e_collision_monitor",
        name="ur7e_collision_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"i2rt_path": LaunchConfiguration("i2rt_path")},
            {"scene_yaml": LaunchConfiguration("scene_yaml")},
            {"stale_timeout": ParameterValue(LaunchConfiguration("stale_timeout"), value_type=float)},
            {"rate_hz": ParameterValue(LaunchConfiguration("rate_hz"), value_type=float)},
            {"activation_mode": LaunchConfiguration("activation_mode")},
            {
                "interarm_activation_distance": ParameterValue(
                    LaunchConfiguration("interarm_activation_distance"),
                    value_type=float,
                )
            },
            {
                "interarm_release_distance": ParameterValue(
                    LaunchConfiguration("interarm_release_distance"),
                    value_type=float,
                )
            },
            {"release_velocity_limits": LaunchConfiguration("release_velocity_limits")},
            {
                "safety_activation_clearance": ParameterValue(
                    LaunchConfiguration("safety_activation_clearance"),
                    value_type=float,
                )
            },
            {"active_sides": LaunchConfiguration("active_sides")},
            {"command_mode": LaunchConfiguration("command_mode")},
            {
                "velocity_tracking_gain": ParameterValue(
                    LaunchConfiguration("velocity_tracking_gain"),
                    value_type=float,
                )
            },
            {"left_wrist_3_offset": ParameterValue(LaunchConfiguration("left_wrist_3_offset"), value_type=float)},
            {"right_wrist_3_offset": ParameterValue(LaunchConfiguration("right_wrist_3_offset"), value_type=float)},
            {"left_fallback_q": LaunchConfiguration("left_fallback_q")},
            {"right_fallback_q": LaunchConfiguration("right_fallback_q")},
        ],
    )

    return LaunchDescription([
        i2rt_path_arg,
        scene_yaml_arg,
        stale_timeout_arg,
        rate_hz_arg,
        activation_mode_arg,
        interarm_activation_distance_arg,
        interarm_release_distance_arg,
        release_velocity_limits_arg,
        safety_activation_clearance_arg,
        active_sides_arg,
        command_mode_arg,
        velocity_tracking_gain_arg,
        left_wrist_3_offset_arg,
        right_wrist_3_offset_arg,
        left_fallback_q_arg,
        right_fallback_q_arg,
        monitor,
    ])
