from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('arm_namespace')
    return LaunchDescription([
        DeclareLaunchArgument('arm_namespace', default_value='rebotarm'),
        DeclareLaunchArgument('execute_motion', default_value='true'),
        DeclareLaunchArgument('show_subtitles', default_value='true'),
        DeclareLaunchArgument('subtitle_timeout', default_value='8.0'),
        DeclareLaunchArgument('start_player', default_value='true'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('rebotarm_voice'), 'config', 'voice.yaml'])),
        Node(package='rebotarm_gesture_actions', executable='trajectory_player_node',
             output='screen', condition=IfCondition(LaunchConfiguration('start_player')),
             parameters=[{'arm_namespace': ns, 'subtitle_timeout': ParameterValue(
                 LaunchConfiguration('subtitle_timeout'), value_type=float)}]),
        Node(package='rebotarm_voice', executable='rviz_subtitles',
             output='screen', condition=IfCondition(LaunchConfiguration('show_subtitles')),
             parameters=[{'arm_namespace': ns, 'subtitle_timeout': ParameterValue(
                 LaunchConfiguration('subtitle_timeout'), value_type=float)}]),
        Node(package='rebotarm_voice', executable='voice_control_node',
             name='rebotarm_voice', output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'arm_namespace': ns,
                 'execute_motion': ParameterValue(LaunchConfiguration('execute_motion'), value_type=bool),
             }]),
    ])
