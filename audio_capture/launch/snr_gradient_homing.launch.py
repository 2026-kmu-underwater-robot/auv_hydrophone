from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    reference_frequency_hz = LaunchConfiguration("reference_frequency_hz")
    output_frame = LaunchConfiguration("output_frame")
    direction_source = LaunchConfiguration("direction_source")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "reference_frequency_hz",
                default_value="21164.0",
                description="Expected pinger frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "output_frame",
                default_value="base_link",
                description="Direction output frame; base_link is the safe controller-compatible default.",
            ),
            DeclareLaunchArgument(
                "direction_source",
                default_value="blend",
                description="Direction source: gradient, particle, or blend.",
            ),
            ComposableNodeContainer(
                name="snr_gradient_homing_pipeline",
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
                                "reference_frequency_hz": reference_frequency_hz,
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
                                "output_frame": output_frame,
                                "direction_source": direction_source,
                            }
                        ],
                    ),
                ],
                output="screen",
            ),
        ]
    )
