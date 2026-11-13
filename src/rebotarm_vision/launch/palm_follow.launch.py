from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    speed_defaults = {'max_joint_speed': '5.0', 'max_tcp_speed': '0.6',
                      'follow_gain': '1.65', 'smoothing': '0.85',
                      'chase_gain_scale': '0.1090909091', 'rate_hz': '100.0',
                      'max_joint_accel': '12.0', 'max_tcp_accel': '2.0',
                      'braking_gain': '1.5',
                      'max_target_lead': '0.06', 'home_joint_speed': '0.8'}
    args = [DeclareLaunchArgument('arm_namespace', default_value='rebotarm'),
            DeclareLaunchArgument('camera_mode', default_value='real'),
            DeclareLaunchArgument('camera_device', default_value=''),
            DeclareLaunchArgument('camera_index', default_value='0'),
            DeclareLaunchArgument('show_debug', default_value='true')]
    common = {'arm_namespace': LaunchConfiguration('arm_namespace')}
    args += [DeclareLaunchArgument(name, default_value=value)
             for name, value in speed_defaults.items()]
    speed_params = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
                    for name in speed_defaults}
    return LaunchDescription(args + [
        Node(package='rebotarm_vision', executable='vision_gesture_node',
             output='screen', parameters=[common, {
                 'camera_mode': ParameterValue(LaunchConfiguration('camera_mode'), value_type=str),
                 'camera_device': ParameterValue(LaunchConfiguration('camera_device'), value_type=str),
                 'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                 'show_debug': ParameterValue(LaunchConfiguration('show_debug'), value_type=bool),
                 'model_complexity': 0,
                 'capture_buffer_size': 1,
                 'latest_frame_only': True,
                 'rate_hz': 60.0,  # 更频繁地取最新帧，无新帧就返回，不重复推理
                 'palm_grace_sec': 0.10,
                 'publish_gestures': False}]),
        Node(package='rebotarm_vision', executable='palm_follow_node',
             output='screen', parameters=[common, speed_params]),
    ])
