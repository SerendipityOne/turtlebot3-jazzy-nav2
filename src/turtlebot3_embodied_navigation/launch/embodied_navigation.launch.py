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

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to the exported YOLOX-Nano ONNX model'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
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
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
