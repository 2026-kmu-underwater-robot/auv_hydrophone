from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    raw_odometry_topic = LaunchConfiguration("raw_odometry_topic")
    homing_odometry_topic = LaunchConfiguration("homing_odometry_topic")
    signal_mode = LaunchConfiguration("signal_mode")
    noise_bag = LaunchConfiguration("noise_bag")
    frequency_hz = LaunchConfiguration("frequency_hz")
    pinger_x = LaunchConfiguration("pinger_x")
    pinger_y = LaunchConfiguration("pinger_y")
    pinger_z = LaunchConfiguration("pinger_z")
    source_amplitude = LaunchConfiguration("source_amplitude")
    clean_noise_amplitude = LaunchConfiguration("clean_noise_amplitude")
    minimum_distance_m = LaunchConfiguration("minimum_distance_m")
    attenuation_power = LaunchConfiguration("attenuation_power")
    sound_speed_mps = LaunchConfiguration("sound_speed_mps")
    sim_arena_yaw_rad = LaunchConfiguration("sim_arena_yaw_rad")
    launch_rviz = LaunchConfiguration("launch_rviz")

    homing_float_names = [
        ("arena_length_m", "15.0"),
        ("arena_width_m", "16.0"),
        # 15 x 16 m 경기장의 +Y/-X 코너에서 0.55 m 안쪽인 AUV 시작점을
        # odom (0,0)으로 변환한 경기장 경계 오프셋.
        ("arena_offset_x_m", "-0.55"),
        ("arena_offset_y_m", "0.55"),
        ("arena_safety_margin_m", "0.55"),
        ("initial_scan_radius_m", "1.50"),
        ("rescan_radius_m", "0.70"),
        ("homing_waypoint_step_m", "0.80"),
        ("homing_zigzag_offset_m", "0.20"),
        ("rolling_gradient_alpha", "0.15"),
        ("waypoint_reach_tolerance_m", "0.15"),
        ("waypoint_dwell_s", "0.1"),
        ("region_sample_spacing_m", "0.15"),
        ("min_region_gradient_magnitude", "0.05"),
        ("min_region_lateral_spread_m", "0.10"),
        ("slope_decrease_threshold_db_per_m2", "1.0"),
        ("vision_near_zone_width_m", "2.0"),
        ("forward_gain", "1.4"),
        ("forward_limit", "0.45"),
        ("yaw_gain", "1.15"),
        ("yaw_limit", "0.72"),
        ("vision_heading_tolerance_rad", "0.12"),
    ]
    homing_int_names = [
        ("region_scan_waypoint_count", "8"),
        ("homing_slope_window_size", "12"),
        ("min_homing_slope_samples", "5"),
        ("min_homing_gradient_samples", "8"),
        ("slope_decrease_limit", "5"),
    ]
    homing_values = {
        name: LaunchConfiguration(name)
        for name, _ in homing_float_names + homing_int_names
    }

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "raw_odometry_topic", default_value="/odometry/filtered"
        ),
        DeclareLaunchArgument(
            "homing_odometry_topic", default_value="/homing/sim_odometry"
        ),
        DeclareLaunchArgument(
            "sim_arena_yaw_rad",
            default_value="0.0",
            description="MuJoCo world에서 arena +X축이 향하는 yaw.",
        ),
        DeclareLaunchArgument(
            "signal_mode",
            default_value="noisy",
            description="clean 또는 실측 배경 잡음이 섞인 noisy.",
        ),
        DeclareLaunchArgument(
            "noise_bag",
            default_value="/home/kim/new_hydrophone_ws/localization_20260719_185918",
        ),
        DeclareLaunchArgument("frequency_hz", default_value="21164.0"),
        DeclareLaunchArgument("pinger_x", default_value="0.0"),
        DeclareLaunchArgument("pinger_y", default_value="-7.28"),
        DeclareLaunchArgument("pinger_z", default_value="-10.95"),
        DeclareLaunchArgument("source_amplitude", default_value="0.03"),
        DeclareLaunchArgument("clean_noise_amplitude", default_value="0.001"),
        DeclareLaunchArgument("minimum_distance_m", default_value="0.5"),
        DeclareLaunchArgument("attenuation_power", default_value="2.0"),
        DeclareLaunchArgument("sound_speed_mps", default_value="1500.0"),
        DeclareLaunchArgument("arena_start_corner", default_value="bottom_left"),
        DeclareLaunchArgument("invert_rc_yaw", default_value="true"),
        DeclareLaunchArgument("vision_handoff_enabled", default_value="true"),
        DeclareLaunchArgument(
            "vision_search_request_topic",
            default_value="/homing/vision_search_active",
        ),
        DeclareLaunchArgument(
            "target_confirmed_topic", default_value="/vision/target_confirmed"
        ),
        DeclareLaunchArgument(
            "rolling_gradient_topic", default_value="/homing/rolling_gradient"
        ),
        DeclareLaunchArgument(
            "homing_direction_topic", default_value="/homing/homing_direction"
        ),
        DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="true이면 V3 SNR map/gradient RViz를 함께 실행한다.",
        ),
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in homing_float_names + homing_int_names
    ]

    pinger_audio = Node(
        package="audio_capture",
        executable="pinger_buoy_audio_sim.py",
        name="pinger_buoy_audio_sim",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "signal_mode": signal_mode,
                "noise_bag": noise_bag,
                "frequency_hz": ParameterValue(frequency_hz, value_type=float),
                "pinger_x": ParameterValue(pinger_x, value_type=float),
                "pinger_y": ParameterValue(pinger_y, value_type=float),
                "pinger_z": ParameterValue(pinger_z, value_type=float),
                "source_amplitude": ParameterValue(
                    source_amplitude, value_type=float
                ),
                "clean_noise_amplitude": ParameterValue(
                    clean_noise_amplitude, value_type=float
                ),
                "minimum_distance_m": ParameterValue(
                    minimum_distance_m, value_type=float
                ),
                "attenuation_power": ParameterValue(
                    attenuation_power, value_type=float
                ),
                "sound_speed_mps": ParameterValue(
                    sound_speed_mps, value_type=float
                ),
                "odometry_topic": raw_odometry_topic,
            }
        ],
    )

    odometry_rebaser = Node(
        package="audio_capture",
        executable="sim_odometry_rebaser",
        name="sim_odometry_rebaser",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "input_topic": raw_odometry_topic,
                "output_topic": homing_odometry_topic,
                "arena_yaw_rad": ParameterValue(
                    sim_arena_yaw_rad, value_type=float
                ),
            }
        ],
    )

    package_share = get_package_share_directory("audio_capture")
    homing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_share + "/launch/region_local_gradient_homing.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odometry_topic": homing_odometry_topic,
            "arena_start_corner": LaunchConfiguration("arena_start_corner"),
            "invert_rc_yaw": LaunchConfiguration("invert_rc_yaw"),
            "vision_handoff_enabled": LaunchConfiguration(
                "vision_handoff_enabled"
            ),
            "vision_search_request_topic": LaunchConfiguration(
                "vision_search_request_topic"
            ),
            "target_confirmed_topic": LaunchConfiguration(
                "target_confirmed_topic"
            ),
            "rolling_gradient_topic": LaunchConfiguration(
                "rolling_gradient_topic"
            ),
            "homing_direction_topic": LaunchConfiguration(
                "homing_direction_topic"
            ),
            "rc_override_topic": LaunchConfiguration("rc_override_topic"),
            **homing_values,
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_share + "/launch/region_local_gradient_rviz.launch.py"
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odometry_topic": homing_odometry_topic,
            "arena_length_m": LaunchConfiguration("arena_length_m"),
            "arena_width_m": LaunchConfiguration("arena_width_m"),
            "arena_offset_x_m": LaunchConfiguration("arena_offset_x_m"),
            "arena_offset_y_m": LaunchConfiguration("arena_offset_y_m"),
            "arena_safety_margin_m": LaunchConfiguration("arena_safety_margin_m"),
            "vision_near_zone_width_m": LaunchConfiguration(
                "vision_near_zone_width_m"
            ),
            "arena_start_corner": LaunchConfiguration("arena_start_corner"),
            "map_cell_size_m": LaunchConfiguration(
                "region_sample_spacing_m"
            ),
            "rolling_gradient_topic": LaunchConfiguration(
                "rolling_gradient_topic"
            ),
            "homing_direction_topic": LaunchConfiguration(
                "homing_direction_topic"
            ),
        }.items(),
    )

    return LaunchDescription(
        arguments + [pinger_audio, odometry_rebaser, homing, rviz]
    )
