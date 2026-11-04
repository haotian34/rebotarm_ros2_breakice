from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument('arm_namespace', default_value='rebotarm'),
            DeclareLaunchArgument('camera_index', default_value='0'),
            DeclareLaunchArgument('show_debug', default_value='true')]
    common = {'arm_namespace': LaunchConfiguration('arm_namespace')}
    return LaunchDescription(args + [
        Node(package='rebotarm_vision', executable='vision_gesture_node',
             output='screen', parameters=[common, {
                 'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                 'show_debug': ParameterValue(LaunchConfiguration('show_debug'), value_type=bool),
                 'publish_gestures': False}]),
        Node(package='rebotarm_vision', executable='palm_follow_node',
             output='screen', parameters=[common]),
    ])
