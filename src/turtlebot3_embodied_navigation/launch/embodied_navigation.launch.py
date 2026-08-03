#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('turtlebot3_embodied_navigation')
    model_path = LaunchConfiguration('model_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    policy_mode = LaunchConfiguration('policy_mode')
    policy_interface = LaunchConfiguration('policy_interface')
    policy_endpoint = LaunchConfiguration('policy_endpoint')
    continuous_policy_endpoint = LaunchConfiguration('continuous_policy_endpoint')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to the exported YOLO11n ONNX model'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'policy_mode', default_value='disabled',
            description='disabled, shadow, or active VLA high-level policy mode'),
        DeclareLaunchArgument(
            'policy_interface', default_value='discrete_skill',
            description='discrete_skill or continuous_local_goal'),
        DeclareLaunchArgument(
            'policy_endpoint', default_value='http://127.0.0.1:8089/select_action',
            description='Loopback endpoint exposed by tools/vla/vla_policy_server.py'),
        DeclareLaunchArgument(
            'continuous_policy_endpoint', default_value='http://127.0.0.1:8089/predict_local_goal',
            description='Loopback endpoint for continuous local VLA goals'),
        Node(
            package='turtlebot3_embodied_navigation',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[
                PathJoinSubstitution([package_share, 'config', 'object_detector.yaml']),
                {'model_path': model_path, 'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='turtlebot3_embodied_navigation',
            executable='find_object_server',
            name='find_object_server',
            output='screen',
            parameters=[
                PathJoinSubstitution([package_share, 'config', 'find_object.yaml']),
                {
                    'use_sim_time': use_sim_time,
                    'policy_mode': policy_mode,
                    'policy_interface': policy_interface,
                    'policy_endpoint': policy_endpoint,
                    'continuous_policy_endpoint': continuous_policy_endpoint,
                },
            ],
        ),
    ])
