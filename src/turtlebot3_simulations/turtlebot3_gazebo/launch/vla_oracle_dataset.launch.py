#!/usr/bin/env python3

"""Generate bounded, labeled VLA episodes in turtlebot3_house."""

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
    gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(gazebo_share, 'worlds', 'turtlebot3_house.world')
    model = os.path.join(
        gazebo_share, 'models', 'turtlebot3_waffle_pi_cam', 'model.sdf')
    urdf = os.path.join(
        gazebo_share, 'urdf', 'turtlebot3_waffle_pi_cam.urdf')
    bridge_config = os.path.join(
        gazebo_share, 'params', 'turtlebot3_waffle_pi_cam_bridge.yaml')
    with open(urdf, encoding='utf-8') as stream:
        robot_description = stream.read()

    output_dir = LaunchConfiguration('output_dir')
    approval_file = LaunchConfiguration('approval_file')
    episode_count = LaunchConfiguration('episode_count')
    seed = LaunchConfiguration('seed')
    target_absent_ratio = LaunchConfiguration('target_absent_ratio')
    policy_interface = LaunchConfiguration('policy_interface')
    resume = LaunchConfiguration('resume')
    preview = LaunchConfiguration('preview')

    server = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-r', '-s', '-v2', world,
             '--force-version', '8'],
        name='gazebo_server', output='screen')
    client = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-g', '-v2', '--force-version', '8'],
        name='gazebo_client', output='screen', condition=IfCondition(preview))
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', 'default', '-name', 'waffle_pi_cam', '-file', model,
            '-x', '-2.0', '-y', '0.5', '-z', '0.01'],
        output='screen')
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen')
    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/camera/image'],
        remappings=[('/camera/image', '/camera/color/image_raw')],
        output='screen')
    generator = Node(
        package='turtlebot3_gazebo', executable='vla_oracle_dataset_generator',
        parameters=[{
            'use_sim_time': True,
            'output_dir': output_dir,
            'approval_file': approval_file,
            'episode_count': episode_count,
            'seed': seed,
            'target_absent_ratio': target_absent_ratio,
            'policy_interface': policy_interface,
            'resume': resume,
        }],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir', description='A new raw VLA episode directory'),
        DeclareLaunchArgument(
            'approval_file', default_value='approval.txt',
            description='File containing APPROVED when episode_count exceeds 200'),
        DeclareLaunchArgument('episode_count', default_value='200'),
        DeclareLaunchArgument('seed', default_value='20260727'),
        DeclareLaunchArgument('target_absent_ratio', default_value='0.30'),
        DeclareLaunchArgument(
            'policy_interface', default_value='discrete_skill',
            description='discrete_skill or continuous_local_goal Oracle schema'),
        DeclareLaunchArgument(
            'resume', default_value='false',
            description='Keep only complete contiguous episodes in an existing output directory'),
        DeclareLaunchArgument('preview', default_value='false'),
        SetEnvironmentVariable(
            'GZ_SIM_HEADLESS_RENDERING', '1', condition=UnlessCondition(preview)),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
        server,
        client,
        TimerAction(
            period=2.0,
            actions=[spawn_robot, state_publisher, bridge, image_bridge]),
        TimerAction(period=6.0, actions=[generator]),
        RegisterEventHandler(OnProcessExit(target_action=generator, on_exit=[Shutdown()])),
    ])
