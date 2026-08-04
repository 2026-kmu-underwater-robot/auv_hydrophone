from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("hydrophone_ctrl")
    config_file = LaunchConfiguration("config_file")
    default_config_file = PathJoinSubstitution(
        [
            package_share,
            "config",
            "pinger_buoy_homing_sim_competition_tank.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description=(
                    "Simulation parameter YAML. Select the experiment or "
                    "competition tank profile here."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            package_share,
                            "launch",
                            "pinger_buoy_region_local_homing_sim.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "controller_mode": "line_search",
                    "config_file": config_file,
                }.items(),
            )
        ]
    )
