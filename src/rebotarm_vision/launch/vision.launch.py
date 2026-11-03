from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    camera_index = LaunchConfiguration("camera_index")
    show_debug = LaunchConfiguration("show_debug")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("camera_index", default_value="0"),
            # 无显示环境（SSH 等）设 show_debug:=false
            DeclareLaunchArgument("show_debug", default_value="true"),
            Node(
                package="rebotarm_vision",
                executable="vision_gesture_node",
                name="rebotarm_vision_gesture",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "camera_index": ParameterValue(camera_index, value_type=int),
                        "show_debug": ParameterValue(show_debug, value_type=bool),
                    }
                ],
            ),
        ]
    )
