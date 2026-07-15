from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    reference_frequency_hz = LaunchConfiguration("reference_frequency_hz")
    direction_source = LaunchConfiguration("direction_source")
    control_enabled = LaunchConfiguration("control_enabled")
    search_forward = LaunchConfiguration("search_forward")
    search_yaw = LaunchConfiguration("search_yaw")
    search_turn_sign = LaunchConfiguration("search_turn_sign")
    invert_rc_yaw = LaunchConfiguration("invert_rc_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "reference_frequency_hz",
                default_value="21164.0",
                description="예상 pinger 주파수(Hz).",
            ),
            DeclareLaunchArgument(
                "direction_source",
                default_value="blend",
                description="방향 선택 방식: gradient, particle 또는 blend.",
            ),
            DeclareLaunchArgument(
                "control_enabled",
                default_value="false",
                description="true이면 초기 원형 탐색과 RC override 제어를 시작한다.",
            ),
            DeclareLaunchArgument(
                "search_forward",
                default_value="0.30",
                description="초기/재탐색 시 정규화된 전진 명령(0~1).",
            ),
            DeclareLaunchArgument(
                "search_yaw",
                default_value="0.30",
                description="초기/재탐색 시 정규화된 yaw 명령(0~1).",
            ),
            DeclareLaunchArgument(
                "search_turn_sign",
                default_value="1.0",
                description="초기 탐색 회전 부호: 1.0 또는 -1.0.",
            ),
            DeclareLaunchArgument(
                "invert_rc_yaw",
                default_value="true",
                description="기체의 RC yaw 채널 방향 반전 여부.",
            ),
            ComposableNodeContainer(
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
                                "reference_frequency_hz": ParameterValue(
                                    reference_frequency_hz, value_type=float
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
                                "output_frame": "base_link",
                                "output_frame_id": "base_link",
                                "direction_source": direction_source,
                            }
                        ],
                    ),
                    ComposableNode(
                        package="audio_capture",
                        plugin="audio_capture::SnrGradientHomingControllerNode",
                        name="snr_gradient_homing_controller",
                        parameters=[
                            {
                                "control_enabled": ParameterValue(
                                    control_enabled, value_type=bool
                                ),
                                "required_direction_frame": "base_link",
                                "search_forward": ParameterValue(
                                    search_forward, value_type=float
                                ),
                                "search_yaw": ParameterValue(
                                    search_yaw, value_type=float
                                ),
                                "search_turn_sign": ParameterValue(
                                    search_turn_sign, value_type=float
                                ),
                                "invert_rc_yaw": ParameterValue(
                                    invert_rc_yaw, value_type=bool
                                ),
                            }
                        ],
                    ),
                ],
                output="screen",
            ),
        ]
    )
