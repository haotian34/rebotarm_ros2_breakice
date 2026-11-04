from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    auto_start = LaunchConfiguration("auto_start")
    rate_hz = LaunchConfiguration("rate_hz")
    track_speed = LaunchConfiguration("track_speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("rate_hz", default_value="60.0"),
            # true = 启动后立即自动表演一次；false = 等待话题触发
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument(
                "track_speed",
                default_value="1.1",
                description="预计算轨迹播放速度倍率",
            ),
            Node(
                package="rebotarm_dance",
                executable="dance_node",
                name="rebotarm_dance",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "rate_hz": ParameterValue(rate_hz, value_type=float),
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                        "track_speed": ParameterValue(track_speed, value_type=float),
                    }
                ],
            ),
        ]
    )
