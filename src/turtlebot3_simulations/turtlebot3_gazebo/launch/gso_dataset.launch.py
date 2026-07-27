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
    world = os.path.join(package_share, 'worlds', 'gso_dataset.world')
    robot_model = os.path.join(
        package_share, 'models', 'turtlebot3_waffle_pi_cam', 'model.sdf')

    headless = LaunchConfiguration('headless')
    output_dir = LaunchConfiguration('output_dir')
    asset_manifest = LaunchConfiguration('asset_manifest')
    approval_file = LaunchConfiguration('approval_file')
    target_count = LaunchConfiguration('target_count')
    seed = LaunchConfiguration('seed')
    settle_frames = LaunchConfiguration('settle_frames')
    capture_timeout = LaunchConfiguration('capture_timeout')
    raised_distance_min = LaunchConfiguration('raised_distance_min')
    raised_distance_max = LaunchConfiguration('raised_distance_max')
    floor_distance_min = LaunchConfiguration('floor_distance_min')
    floor_distance_max = LaunchConfiguration('floor_distance_max')
    maximum_box_area_ratio = LaunchConfiguration('maximum_box_area_ratio')
    require_full_primary_coverage = LaunchConfiguration(
        'require_full_primary_coverage')
    maximum_rejection_ratio = LaunchConfiguration('maximum_rejection_ratio')
    negative_only = LaunchConfiguration('negative_only')
    negative_train_count = LaunchConfiguration('negative_train_count')
    negative_validation_count = LaunchConfiguration('negative_validation_count')

    # ros_gz_sim's launcher uses shell=True, so Launch signals its shell instead of
    # the Gazebo process. Launch gz directly to make shutdown reach the real process.
    server = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-r', '-s', '-v2', world,
             '--force-version', '8'],
        name='gazebo_server', output='screen')
    client = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-g', '-v2', '--force-version', '8'],
        name='gazebo_client', output='screen', condition=UnlessCondition(headless))
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'gso_dataset', '-name', 'waffle_pi_cam', '-file', robot_model,
            '-x', '-1.0', '-y', '0.0', '-z', '0.01'],
        output='screen')
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image', '/camera/depth_image'],
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen')
    generator = Node(
        package='turtlebot3_gazebo',
        executable='gso_dataset_generator',
        parameters=[{
            'output_dir': output_dir,
            'asset_manifest': asset_manifest,
            'approval_file': approval_file,
            'target_count': target_count,
            'seed': seed,
            'settle_frames': settle_frames,
            'capture_timeout': capture_timeout,
            'raised_distance_min': raised_distance_min,
            'raised_distance_max': raised_distance_max,
            'floor_distance_min': floor_distance_min,
            'floor_distance_max': floor_distance_max,
            'maximum_box_area_ratio': maximum_box_area_ratio,
            'require_full_primary_coverage': require_full_primary_coverage,
            'maximum_rejection_ratio': maximum_rejection_ratio,
            'negative_only': negative_only,
            'negative_train_count': negative_train_count,
            'negative_validation_count': negative_validation_count,
        }],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('asset_manifest'),
        DeclareLaunchArgument('approval_file', default_value='approval.txt'),
        DeclareLaunchArgument('target_count', default_value='100'),
        DeclareLaunchArgument('seed', default_value='20260720'),
        DeclareLaunchArgument('settle_frames', default_value='15'),
        DeclareLaunchArgument('capture_timeout', default_value='45.0'),
        DeclareLaunchArgument('raised_distance_min', default_value='1.4'),
        DeclareLaunchArgument('raised_distance_max', default_value='1.7'),
        DeclareLaunchArgument('floor_distance_min', default_value='0.8'),
        DeclareLaunchArgument('floor_distance_max', default_value='2.5'),
        DeclareLaunchArgument('maximum_box_area_ratio', default_value='0.75'),
        DeclareLaunchArgument('require_full_primary_coverage', default_value='false'),
        DeclareLaunchArgument('maximum_rejection_ratio', default_value='-1.0'),
        DeclareLaunchArgument('negative_only', default_value='false'),
        DeclareLaunchArgument('negative_train_count', default_value='0'),
        DeclareLaunchArgument('negative_validation_count', default_value='0'),
        SetEnvironmentVariable(
            'GZ_SIM_HEADLESS_RENDERING', '1', condition=IfCondition(headless)),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(package_share, 'models')),
        server,
        client,
        TimerAction(period=2.0, actions=[spawn_robot, image_bridge]),
        TimerAction(period=6.0, actions=[generator]),
        RegisterEventHandler(OnProcessExit(target_action=generator, on_exit=[Shutdown()])),
    ])
