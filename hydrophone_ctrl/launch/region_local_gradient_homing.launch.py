from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    default_config_file = PathJoinSubstitution(
        [
            FindPackageShare("hydrophone_ctrl"),
            "config",
            "region_local_gradient_homing_competition_tank.yaml",
        ]
    )

    config_file_argument = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="ROS 2 parameter YAML for the homing pipeline.",
    )
    launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch the real-robot RViz visualizer with the same YAML.",
    )

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
                parameters=[config_file],
            ),
            ComposableNode(
                package="hydrophone_ctrl",
                plugin="audio_capture::RegionLocalGradientEstimatorNode",
                name="region_local_gradient_estimator",
                parameters=[config_file],
            ),
            ComposableNode(
                package="hydrophone_ctrl",
                plugin="audio_capture::WaypointHomingControllerNode",
                name="waypoint_homing_controller",
                parameters=[config_file],
            ),
        ],
        output="screen",
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hydrophone_ctrl"),
                    "launch",
                    "region_local_gradient_rviz.launch.py",
                ]
            )
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={
            "config_file": config_file,
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            config_file_argument,
            launch_rviz_argument,
            container,
            rviz,
        ]
    )
