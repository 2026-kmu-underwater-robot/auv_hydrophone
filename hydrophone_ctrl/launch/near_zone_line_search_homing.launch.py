from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    default_config_file = PathJoinSubstitution(
        [
            FindPackageShare("hydrophone_ctrl"),
            "config",
            "near_zone_line_search_competition_tank.yaml",
        ]
    )

    config_file_argument = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="ROS 2 parameter YAML for near-zone line search.",
    )

    return LaunchDescription(
        [
            config_file_argument,
            Node(
                package="hydrophone_ctrl",
                executable="near_zone_line_search_controller",
                name="near_zone_line_search_controller",
                output="screen",
                emulate_tty=True,
                parameters=[config_file],
            )
        ]
    )
