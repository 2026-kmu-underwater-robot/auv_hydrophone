from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    controller_mode = LaunchConfiguration("controller_mode")
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
    launch_rviz = LaunchConfiguration("launch_rviz")

    package_share = FindPackageShare("hydrophone_ctrl")
    default_config_file = PathJoinSubstitution(
        [
            package_share,
            "config",
            "pinger_buoy_homing_sim_experiment_tank.yaml",
        ]
    )

    arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description=(
                "Simulation parameter YAML. Select the experiment or competition "
                "tank profile here."
            ),
        ),
        DeclareLaunchArgument(
            "controller_mode",
            default_value="region",
            description="region 또는 line_search. 두 제어기는 동시에 실행하지 않는다.",
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
        # 기본 pinger 좌표는 scene.xml 실험 수조 프리셋의 pinger_source와 같다.
        DeclareLaunchArgument("pinger_x", default_value="2.20"),
        DeclareLaunchArgument("pinger_y", default_value="-1.00"),
        DeclareLaunchArgument("pinger_z", default_value="-0.65"),
        DeclareLaunchArgument("source_amplitude", default_value="0.03"),
        DeclareLaunchArgument("clean_noise_amplitude", default_value="0.001"),
        DeclareLaunchArgument("minimum_distance_m", default_value="0.5"),
        DeclareLaunchArgument("attenuation_power", default_value="2.0"),
        DeclareLaunchArgument("sound_speed_mps", default_value="1500.0"),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="true이면 V3 SNR map/gradient RViz를 함께 실행한다.",
        ),
    ]

    pinger_audio = Node(
        package="audio_capture",
        executable="pinger_buoy_audio_sim.py",
        name="pinger_buoy_audio_sim",
        output="screen",
        parameters=[
            config_file,
            {
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
            },
        ],
    )

    odometry_rebaser = Node(
        package="audio_capture",
        executable="sim_odometry_rebaser",
        name="sim_odometry_rebaser",
        output="screen",
        parameters=[config_file],
    )

    region_homing = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", controller_mode, "' == 'region'"])
        ),
        scoped=True,
        actions=[
            SetLaunchConfiguration("launch_rviz", "false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            package_share,
                            "launch",
                            "region_local_gradient_homing.launch.py",
                        ]
                    )
                ),
                launch_arguments={"config_file": config_file}.items(),
            ),
        ],
    )

    line_search_detector = Node(
        package="audio_capture",
        executable="audio_frequency_detector",
        name="audio_frequency_detector",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller_mode, "' == 'line_search'"])
        ),
        parameters=[config_file],
    )

    line_search_homing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "near_zone_line_search_homing.launch.py"]
            )
        ),
        condition=IfCondition(
            PythonExpression(["'", controller_mode, "' == 'line_search'"])
        ),
        launch_arguments={"config_file": config_file}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "region_local_gradient_rviz.launch.py"]
            )
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={
            "config_file": config_file,
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription(
        arguments
        + [
            pinger_audio,
            odometry_rebaser,
            region_homing,
            line_search_detector,
            line_search_homing,
            rviz,
        ]
    )
