from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    cooldown_sec = LaunchConfiguration("cooldown_sec")
    reset_pose = LaunchConfiguration("reset_pose")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # 动作结束后忽略手势的冷却时间（秒）
            DeclareLaunchArgument("cooldown_sec", default_value="2.0"),
            # 动作结束后的复位姿态：zeros=全零初始位 / home=悬空鸡脖子 / none=不复位
            DeclareLaunchArgument("reset_pose", default_value="zeros"),
            Node(
                package="rebotarm_gesture_actions",
                executable="trajectory_player_node",
                name="rebotarm_trajectory_player",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "reset_pose": reset_pose,
                    }
                ],
            ),
            Node(
                package="rebotarm_gesture_actions",
                executable="gesture_dispatcher_node",
                name="rebotarm_gesture_dispatcher",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "cooldown_sec": cooldown_sec,
                    }
                ],
            ),
        ]
    )
