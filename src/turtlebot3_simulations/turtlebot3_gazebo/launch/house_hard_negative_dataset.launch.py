#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(package_share, 'worlds', 'turtlebot3_house.world')
    robot = os.path.join(
        package_share, 'models', 'turtlebot3_waffle_pi_cam', 'model.sdf')

    headless = LaunchConfiguration('headless')
    output_dir = LaunchConfiguration('output_dir')
    train_count = LaunchConfiguration('train_count')
    validation_count = LaunchConfiguration('validation_count')
    seed = LaunchConfiguration('seed')
    settle_frames = LaunchConfiguration('settle_frames')

    server = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-r', '-s', '-v2', world,
             '--force-version', '8'],
        name='gazebo_server', output='screen')
    client = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-g', '-v2', '--force-version', '8'],
        name='gazebo_client', output='screen', condition=UnlessCondition(headless))
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', 'default', '-name', 'waffle_pi_cam', '-file', robot,
            '-x', '1.083', '-y', '1.090', '-z', '0.01'],
        output='screen')
    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/camera/image'],
        remappings=[('/camera/image', '/camera/color/image_raw')],
        output='screen')
    generator = Node(
        package='turtlebot3_gazebo', executable='house_hard_negative_generator',
        parameters=[{
            'use_sim_time': True,
            'output_dir': output_dir,
            'train_count': train_count,
            'validation_count': validation_count,
            'seed': seed,
            'settle_frames': settle_frames,
        }],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('train_count', default_value='600'),
        DeclareLaunchArgument('validation_count', default_value='100'),
        DeclareLaunchArgument('seed', default_value='20260723'),
        DeclareLaunchArgument('settle_frames', default_value='5'),
        SetEnvironmentVariable(
            'GZ_SIM_HEADLESS_RENDERING', '1', condition=IfCondition(headless)),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(package_share, 'models')),
        server,
        client,
        TimerAction(period=2.0, actions=[spawn_robot, image_bridge]),
        TimerAction(period=5.0, actions=[generator]),
        RegisterEventHandler(OnProcessExit(target_action=generator, on_exit=[Shutdown()])),
    ])
