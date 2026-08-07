from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("hydrophone_ctrl")
    config_file = LaunchConfiguration("config_file")
    launch_audio_detector = LaunchConfiguration("launch_audio_detector")
    launch_rviz = LaunchConfiguration("launch_rviz")

    default_config = PathJoinSubstitution(
        [package_share, "config", "haredcoded_pwm_homing.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "haredcoded_pwm_homing.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="PWM sequence and visualization parameter YAML.",
            ),
            DeclareLaunchArgument(
                "launch_audio_detector",
                default_value="true",
                description="Start the existing frequency/SNR detector.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with the time-SNR display.",
            ),
            Node(
                package="audio_capture",
                executable="audio_frequency_detector",
                name="audio_frequency_detector",
                output="screen",
                condition=IfCondition(launch_audio_detector),
                parameters=[config_file],
            ),
            Node(
                package="hydrophone_ctrl",
                executable="haredcoded_pwm_node",
                name="haredcoded_pwm_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_file],
            ),
            Node(
                package="hydrophone_ctrl",
                executable="haredcoded_pwm_visualizer",
                name="haredcoded_pwm_visualizer",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="haredcoded_pwm_rviz",
                output="screen",
                condition=IfCondition(launch_rviz),
                arguments=["-d", rviz_config],
            ),
        ]
    )
