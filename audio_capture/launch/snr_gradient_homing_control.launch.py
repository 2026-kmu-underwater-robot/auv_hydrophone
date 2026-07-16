from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# [폐루프 homing launch 구성] 오디오 분석·방향 추정·AUV 제어 세 노드와 안전 기본값을 정의한다.
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    audio_topic = LaunchConfiguration("audio_topic")
    audio_stamped_topic = LaunchConfiguration("audio_stamped_topic")
    use_stamped_audio = LaunchConfiguration("use_stamped_audio")
    odometry_topic = LaunchConfiguration("odometry_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    reference_frequency_hz = LaunchConfiguration("reference_frequency_hz")
    audio_input_latency_s = LaunchConfiguration("audio_input_latency_s")
    direction_source = LaunchConfiguration("direction_source")
    horizontal_only = LaunchConfiguration("horizontal_only")
    particle_count = LaunchConfiguration("particle_count")
    particle_area_width_m = LaunchConfiguration("particle_area_width_m")
    particle_area_height_m = LaunchConfiguration("particle_area_height_m")
    particle_start_corner = LaunchConfiguration("particle_start_corner")
    particle_area_yaw_rad = LaunchConfiguration("particle_area_yaw_rad")
    particle_roughening_std_m = LaunchConfiguration("particle_roughening_std_m")
    control_enabled = LaunchConfiguration("control_enabled")
    controller_dry_run = LaunchConfiguration("controller_dry_run")
    enable_arrival_detection = LaunchConfiguration("enable_arrival_detection")
    geofence_margin_m = LaunchConfiguration("geofence_margin_m")
    initial_diagonal_distance_m = LaunchConfiguration("initial_diagonal_distance_m")
    initial_diagonal_command = LaunchConfiguration("initial_diagonal_command")
    initial_diagonal_timeout_s = LaunchConfiguration("initial_diagonal_timeout_s")
    search_forward = LaunchConfiguration("search_forward")
    search_yaw = LaunchConfiguration("search_yaw")
    search_turn_sign = LaunchConfiguration("search_turn_sign")
    invert_rc_yaw = LaunchConfiguration("invert_rc_yaw")
    invert_rc_lateral = LaunchConfiguration("invert_rc_lateral")
    rc_override_topic = LaunchConfiguration("rc_override_topic")
    rc_preview_topic = LaunchConfiguration("rc_preview_topic")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("audio_topic", default_value="/audio"),
        DeclareLaunchArgument("audio_stamped_topic", default_value="/audio_stamped"),
        DeclareLaunchArgument(
            "use_stamped_audio",
            default_value="true",
            description="실기에서는 capture node의 원본 PTS가 든 stamped audio를 사용한다.",
        ),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument(
            "reference_frequency_hz",
            default_value="21164.0",
            description="예상 pinger 주파수(Hz).",
        ),
        DeclareLaunchArgument(
            "audio_input_latency_s",
            default_value="0.0",
            description="오디오 수신 시각에서 추가로 보정할 캡처/전송 지연(초).",
        ),
        DeclareLaunchArgument(
            "direction_source",
            default_value="blend",
            description="방향 선택 방식: gradient, particle 또는 blend.",
        ),
        DeclareLaunchArgument(
            "horizontal_only",
            default_value="true",
            description="첫 고정 pinger 시험에서는 AUV와 pinger 수심을 맞추고 true를 사용한다.",
        ),
        DeclareLaunchArgument(
            "particle_area_width_m",
            default_value="2.0",
            description="수조/경기장의 가로 폭(m).",
        ),
        DeclareLaunchArgument(
            "particle_area_height_m",
            default_value="5.0",
            description="수조/경기장의 세로 길이(m).",
        ),
        DeclareLaunchArgument("particle_count", default_value="500"),
        DeclareLaunchArgument(
            "particle_start_corner",
            default_value="bottom_left",
            description="AUV 시작 모서리: bottom_left 또는 bottom_right.",
        ),
        DeclareLaunchArgument(
            "particle_area_yaw_rad",
            default_value="0.0",
            description="odom 좌표계에서 경기장 왼쪽→오른쪽 축의 yaw(rad).",
        ),
        DeclareLaunchArgument(
            "particle_roughening_std_m",
            default_value="0.05",
            description="고정 pinger 수조 시험용 재표본화 위치 잡음 표준편차.",
        ),
        DeclareLaunchArgument(
            "control_enabled",
            default_value="false",
            description="true이면 모서리 이탈 대각선 이동 후 초기 원형 탐색을 시작한다.",
        ),
        DeclareLaunchArgument(
            "controller_dry_run",
            default_value="true",
            description="true이면 preview만 발행하고 실제 MAVROS RC override는 발행하지 않는다.",
        ),
        DeclareLaunchArgument(
            "enable_arrival_detection",
            default_value="false",
            description="near-source 임계값을 수조에서 보정하기 전에는 false로 둔다.",
        ),
        DeclareLaunchArgument(
            "geofence_margin_m",
            default_value="0.20",
            description="수조/경기장 벽 안쪽에서 수평 이동을 clamp할 안전 여유 거리(m).",
        ),
        DeclareLaunchArgument(
            "initial_diagonal_distance_m",
            default_value="0.7",
            description="모서리에서 원형 탐색 시작점까지 대각선으로 이동할 odometry 거리(m).",
        ),
        DeclareLaunchArgument(
            "initial_diagonal_command",
            default_value="0.30",
            description="초기 대각선 구간의 정규화된 수평 이동 명령(0~1).",
        ),
        DeclareLaunchArgument(
            "initial_diagonal_timeout_s",
            default_value="10.0",
            description="목표 거리에 도달하지 못하면 제어를 해제하는 안전 timeout(초).",
        ),
        DeclareLaunchArgument("search_forward", default_value="0.30"),
        DeclareLaunchArgument("search_yaw", default_value="0.30"),
        DeclareLaunchArgument("search_turn_sign", default_value="1.0"),
        DeclareLaunchArgument("invert_rc_yaw", default_value="true"),
        DeclareLaunchArgument("invert_rc_lateral", default_value="false"),
        DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("rc_preview_topic", default_value="/homing/rc_override_preview"),
    ]

    container = ComposableNodeContainer(
        name="snr_gradient_homing_control_pipeline",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::AudioPhaseEstimatorNode",
                name="audio_phase_estimator",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "audio_topic": audio_topic,
                        "audio_stamped_topic": audio_stamped_topic,
                        "use_stamped_audio": ParameterValue(
                            use_stamped_audio, value_type=bool
                        ),
                        "odometry_topic": odometry_topic,
                        "depth_topic": depth_topic,
                        "reference_frequency_hz": ParameterValue(
                            reference_frequency_hz, value_type=float
                        ),
                        "initial_demodulation_frequency_hz": ParameterValue(
                            reference_frequency_hz, value_type=float
                        ),
                        "audio_input_latency_s": ParameterValue(
                            audio_input_latency_s, value_type=float
                        ),
                        "publish_homing_direction": False,
                    }
                ],
            ),
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::SnrGradientHomingNode",
                name="snr_gradient_homing",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "odometry_topic": odometry_topic,
                        "depth_topic": depth_topic,
                        "output_frame": "base_link",
                        "output_frame_id": "base_link",
                        "direction_source": direction_source,
                        "horizontal_only": ParameterValue(horizontal_only, value_type=bool),
                        "particle_count": ParameterValue(particle_count, value_type=int),
                        "particle_area_width_m": ParameterValue(
                            particle_area_width_m, value_type=float
                        ),
                        "particle_area_height_m": ParameterValue(
                            particle_area_height_m, value_type=float
                        ),
                        "particle_start_corner": particle_start_corner,
                        "particle_area_yaw_rad": ParameterValue(
                            particle_area_yaw_rad, value_type=float
                        ),
                        "particle_roughening_std_m": ParameterValue(
                            particle_roughening_std_m, value_type=float
                        ),
                    }
                ],
            ),
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::SnrGradientHomingControllerNode",
                name="snr_gradient_homing_controller",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "control_enabled": ParameterValue(control_enabled, value_type=bool),
                        "dry_run": ParameterValue(controller_dry_run, value_type=bool),
                        "enable_arrival_detection": ParameterValue(
                            enable_arrival_detection, value_type=bool
                        ),
                        "odometry_topic": odometry_topic,
                        "particle_area_width_m": ParameterValue(
                            particle_area_width_m, value_type=float
                        ),
                        "particle_area_height_m": ParameterValue(
                            particle_area_height_m, value_type=float
                        ),
                        "particle_start_corner": particle_start_corner,
                        "particle_area_yaw_rad": ParameterValue(
                            particle_area_yaw_rad, value_type=float
                        ),
                        "geofence_margin_m": ParameterValue(
                            geofence_margin_m, value_type=float
                        ),
                        "initial_diagonal_distance_m": ParameterValue(
                            initial_diagonal_distance_m, value_type=float
                        ),
                        "initial_diagonal_command": ParameterValue(
                            initial_diagonal_command, value_type=float
                        ),
                        "initial_diagonal_timeout_s": ParameterValue(
                            initial_diagonal_timeout_s, value_type=float
                        ),
                        "required_direction_frame": "base_link",
                        "search_forward": ParameterValue(search_forward, value_type=float),
                        "search_yaw": ParameterValue(search_yaw, value_type=float),
                        "search_turn_sign": ParameterValue(search_turn_sign, value_type=float),
                        "invert_rc_yaw": ParameterValue(invert_rc_yaw, value_type=bool),
                        "invert_rc_lateral": ParameterValue(
                            invert_rc_lateral, value_type=bool
                        ),
                        "rc_override_topic": rc_override_topic,
                        "rc_preview_topic": rc_preview_topic,
                    }
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(arguments + [container])
