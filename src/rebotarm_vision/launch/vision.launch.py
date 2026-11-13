from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rebotarm_vision.gesture_classifier import DEFAULT_PARAMS


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    camera_index = LaunchConfiguration("camera_index")
    camera_device = LaunchConfiguration("camera_device")
    camera_mode = LaunchConfiguration("camera_mode")
    show_debug = LaunchConfiguration("show_debug")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("camera_device", default_value=""),
            DeclareLaunchArgument("camera_mode", default_value="manual"),
            # 无显示环境（SSH 等）设 show_debug:=false
            DeclareLaunchArgument("show_debug", default_value="true"),
            *(DeclareLaunchArgument(key, default_value=str(value))
              for key, value in DEFAULT_PARAMS.items()),
            Node(
                package="rebotarm_vision",
                executable="vision_gesture_node",
                name="rebotarm_vision_gesture",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "camera_index": ParameterValue(camera_index, value_type=int),
                        "camera_device": ParameterValue(camera_device, value_type=str),
                        "camera_mode": camera_mode,
                        "show_debug": ParameterValue(show_debug, value_type=bool),
                        **{key: ParameterValue(LaunchConfiguration(key), value_type=float)
                           for key in DEFAULT_PARAMS},
                    }
                ],
            ),
        ]
    )
