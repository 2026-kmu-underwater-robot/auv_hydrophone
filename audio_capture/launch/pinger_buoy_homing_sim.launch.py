from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    noise_bag = LaunchConfiguration("noise_bag")
    frequency_hz = LaunchConfiguration("frequency_hz")
    pinger_x = LaunchConfiguration("pinger_x")
    pinger_y = LaunchConfiguration("pinger_y")
    pinger_z = LaunchConfiguration("pinger_z")
    source_amplitude = LaunchConfiguration("source_amplitude")
    minimum_distance_m = LaunchConfiguration("minimum_distance_m")
    sound_speed_mps = LaunchConfiguration("sound_speed_mps")
    odometry_topic = LaunchConfiguration("odometry_topic")
    arena_start_corner = LaunchConfiguration("arena_start_corner")
    arena_yaw_rad = LaunchConfiguration("arena_yaw_rad")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "noise_bag",
            default_value="/home/kim/new_hydrophone_ws/localization_20260707_193328",
            description="실측 배경 노이즈를 읽을 ROS 2 bag 디렉터리.",
        ),
        DeclareLaunchArgument("frequency_hz", default_value="21134.0"),
        DeclareLaunchArgument("pinger_x", default_value="2.5"),
        DeclareLaunchArgument("pinger_y", default_value="1.4"),
        DeclareLaunchArgument("pinger_z", default_value="-0.5"),
        DeclareLaunchArgument("source_amplitude", default_value="0.03"),
        DeclareLaunchArgument("minimum_distance_m", default_value="0.5"),
        DeclareLaunchArgument("sound_speed_mps", default_value="1500.0"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("arena_start_corner", default_value="bottom_right"),
        DeclareLaunchArgument(
            "arena_yaw_rad",
            default_value="3.141592653589793",
        ),
    ]

    pinger_audio = Node(
        package="audio_capture",
        executable="pinger_buoy_audio_sim.py",
        name="pinger_buoy_audio_sim",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "noise_bag": noise_bag,
                "frequency_hz": ParameterValue(frequency_hz, value_type=float),
                "pinger_x": ParameterValue(pinger_x, value_type=float),
                "pinger_y": ParameterValue(pinger_y, value_type=float),
                "pinger_z": ParameterValue(pinger_z, value_type=float),
                "source_amplitude": ParameterValue(
                    source_amplitude, value_type=float
                ),
                "minimum_distance_m": ParameterValue(
                    minimum_distance_m, value_type=float
                ),
                "sound_speed_mps": ParameterValue(
                    sound_speed_mps, value_type=float
                ),
                "odometry_topic": odometry_topic,
            }
        ],
    )

    homing_launch_path = (
        get_package_share_directory("audio_capture")
        + "/launch/snr_gradient_homing_control.launch.py"
    )
    homing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(homing_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_stamped_audio": "true",
            "audio_topic": "/audio",
            "audio_stamped_topic": "/audio_stamped",
            "odometry_topic": odometry_topic,
            "reference_frequency_hz": frequency_hz,
            "arena_start_corner": arena_start_corner,
            "arena_yaw_rad": arena_yaw_rad,
        }.items(),
    )

    return LaunchDescription(arguments + [pinger_audio, homing])
