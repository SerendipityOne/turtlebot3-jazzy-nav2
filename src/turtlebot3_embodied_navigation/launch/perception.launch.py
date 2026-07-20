#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to the exported YOLOX-Nano ONNX model'),
        Node(
            package='turtlebot3_embodied_navigation',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'model_path': model_path,
            }],
        ),
    ])
