from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # true = 启动后立即自动表演一次；false = 等待话题触发
            DeclareLaunchArgument("auto_start", default_value="false"),
            Node(
                package="rebotarm_dance",
                executable="dance_node",
                name="rebotarm_dance",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                    }
                ],
            ),
        ]
    )
