#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    intra_op_threads = LaunchConfiguration('intra_op_threads')
    detector_config = PathJoinSubstitution([
        FindPackageShare('turtlebot3_embodied_navigation'),
        'config',
        'object_detector.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to the exported YOLO11n ONNX model'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'intra_op_threads',
            default_value=str(min(2, os.cpu_count() or 1)),
            description='Read-only ONNX Runtime intra-op thread count'),
        Node(
            package='turtlebot3_embodied_navigation',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[detector_config, {
                'use_sim_time': use_sim_time,
                'model_path': model_path,
                'intra_op_threads': ParameterValue(
                    intra_op_threads, value_type=int),
            }],
        ),
    ])
