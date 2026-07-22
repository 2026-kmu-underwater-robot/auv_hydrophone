from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    odometry_topic = LaunchConfiguration("odometry_topic")
    snr_topic = LaunchConfiguration("snr_topic")
    state_topic = LaunchConfiguration("state_topic")
    waypoint_topic = LaunchConfiguration("waypoint_topic")
    scan_center_topic = LaunchConfiguration("scan_center_topic")
    region_gradient_topic = LaunchConfiguration("region_gradient_topic")
    rolling_gradient_topic = LaunchConfiguration("rolling_gradient_topic")
    snr_trend_topic = LaunchConfiguration("snr_trend_topic")
    rc_override_topic = LaunchConfiguration("rc_override_topic")

    float_names = [
        ("arena_length_m", "15.0"),
        ("arena_width_m", "16.0"),
        ("arena_offset_x_m", "0.0"),
        ("arena_offset_y_m", "0.0"),
        ("arena_safety_margin_m", "0.5"),
        ("initial_scan_radius_m", "1.5"),
        ("rescan_radius_m", "0.7"),
        ("homing_waypoint_step_m", "0.8"),
        ("homing_zigzag_offset_m", "0.2"),
        ("rolling_gradient_alpha", "0.15"),
        ("waypoint_reach_tolerance_m", "0.15"),
        ("waypoint_dwell_s", "0.1"),
        ("region_sample_spacing_m", "0.15"),
        ("min_region_gradient_magnitude", "0.05"),
        ("min_region_lateral_spread_m", "0.10"),
        ("slope_decrease_threshold_db_per_m2", "1.0"),
        ("vision_near_zone_width_m", "2.0"),
        ("odometry_timeout_s", "0.5"),
        ("max_snr_odom_skew_s", "0.15"),
        ("forward_gain", "0.8"),
        ("forward_limit", "0.5"),
        ("yaw_gain", "1.15"),
        ("yaw_limit", "0.72"),
        ("move_heading_tolerance_rad", "0.35"),
        ("vision_heading_tolerance_rad", "0.12"),
        ("rc_pwm_span", "400.0"),
        ("rate_hz", "30.0"),
    ]
    int_names = [
        ("region_scan_waypoint_count", "8"),
        ("homing_slope_window_size", "12"),
        ("min_homing_slope_samples", "5"),
        ("min_homing_gradient_samples", "8"),
        ("slope_decrease_limit", "5"),
    ]
    values = {name: LaunchConfiguration(name) for name, _ in float_names + int_names}

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "snr_topic", default_value="/audio_frequency_detector/snr_db_stamped"
        ),
        DeclareLaunchArgument("state_topic", default_value="/homing/control_state"),
        DeclareLaunchArgument(
            "waypoint_topic", default_value="/homing/current_waypoint"
        ),
        DeclareLaunchArgument("scan_center_topic", default_value="/homing/scan_center"),
        DeclareLaunchArgument(
            "vision_search_request_topic",
            default_value="/homing/vision_search_active",
        ),
        DeclareLaunchArgument(
            "target_confirmed_topic", default_value="/vision/target_confirmed"
        ),
        DeclareLaunchArgument(
            "region_gradient_topic", default_value="/homing/region_gradient"
        ),
        DeclareLaunchArgument(
            "rolling_gradient_topic", default_value="/homing/rolling_gradient"
        ),
        DeclareLaunchArgument(
            "homing_direction_topic", default_value="/homing/homing_direction"
        ),
        DeclareLaunchArgument("snr_trend_topic", default_value="/homing/snr_trend"),
        DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument(
            "arena_start_corner",
            default_value="bottom_left",
            description=(
                "bottom_left (inside=-Y) or bottom_right (inside=+Y); "
                "initial heading is always arena +X."
            ),
        ),
        DeclareLaunchArgument("invert_rc_yaw", default_value="true"),
        DeclareLaunchArgument("vision_handoff_enabled", default_value="true"),
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in float_names + int_names
    ]

    common_topics = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "odometry_topic": odometry_topic,
        "state_topic": state_topic,
        "region_gradient_topic": region_gradient_topic,
        "rolling_gradient_topic": rolling_gradient_topic,
        "snr_trend_topic": snr_trend_topic,
    }
    estimator_parameters = {
        **common_topics,
        "snr_topic": snr_topic,
        "region_sample_spacing_m": ParameterValue(
            values["region_sample_spacing_m"], value_type=float
        ),
        "homing_slope_window_size": ParameterValue(
            values["homing_slope_window_size"], value_type=int
        ),
        "min_homing_slope_samples": ParameterValue(
            values["min_homing_slope_samples"], value_type=int
        ),
        "min_homing_gradient_samples": ParameterValue(
            values["min_homing_gradient_samples"], value_type=int
        ),
        "min_region_gradient_magnitude": ParameterValue(
            values["min_region_gradient_magnitude"], value_type=float
        ),
        "min_region_lateral_spread_m": ParameterValue(
            values["min_region_lateral_spread_m"], value_type=float
        ),
        "odometry_timeout_s": ParameterValue(
            values["odometry_timeout_s"], value_type=float
        ),
        "max_snr_odom_skew_s": ParameterValue(
            values["max_snr_odom_skew_s"], value_type=float
        ),
    }
    controller_parameters = {
        **common_topics,
        "waypoint_topic": waypoint_topic,
        "scan_center_topic": scan_center_topic,
        "vision_search_request_topic": LaunchConfiguration(
            "vision_search_request_topic"
        ),
        "target_confirmed_topic": LaunchConfiguration("target_confirmed_topic"),
        "homing_direction_topic": LaunchConfiguration("homing_direction_topic"),
        "rc_override_topic": rc_override_topic,
        "arena_start_corner": LaunchConfiguration("arena_start_corner"),
        "invert_rc_yaw": ParameterValue(
            LaunchConfiguration("invert_rc_yaw"), value_type=bool
        ),
        "vision_handoff_enabled": ParameterValue(
            LaunchConfiguration("vision_handoff_enabled"), value_type=bool
        ),
    }
    for name in [
        "arena_length_m",
        "arena_width_m",
        "arena_offset_x_m",
        "arena_offset_y_m",
        "arena_safety_margin_m",
        "initial_scan_radius_m",
        "rescan_radius_m",
        "homing_waypoint_step_m",
        "homing_zigzag_offset_m",
        "rolling_gradient_alpha",
        "waypoint_reach_tolerance_m",
        "waypoint_dwell_s",
        "slope_decrease_threshold_db_per_m2",
        "vision_near_zone_width_m",
        "odometry_timeout_s",
        "forward_gain",
        "forward_limit",
        "yaw_gain",
        "yaw_limit",
        "move_heading_tolerance_rad",
        "vision_heading_tolerance_rad",
        "rc_pwm_span",
        "rate_hz",
    ]:
        controller_parameters[name] = ParameterValue(values[name], value_type=float)
    for name in [
        "region_scan_waypoint_count",
        "slope_decrease_limit",
    ]:
        controller_parameters[name] = ParameterValue(values[name], value_type=int)

    container = ComposableNodeContainer(
        name="region_local_gradient_homing_pipeline",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::AudioFrequencyDetectorNode",
                name="audio_frequency_detector",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            ),
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::RegionLocalGradientEstimatorNode",
                name="region_local_gradient_estimator",
                parameters=[estimator_parameters],
            ),
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::WaypointHomingControllerNode",
                name="waypoint_homing_controller",
                parameters=[controller_parameters],
            ),
        ],
        output="screen",
    )
    return LaunchDescription(arguments + [container])
