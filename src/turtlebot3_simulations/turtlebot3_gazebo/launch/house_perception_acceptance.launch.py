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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    embodied_share = get_package_share_directory('turtlebot3_embodied_navigation')
    world = os.path.join(gazebo_share, 'worlds', 'turtlebot3_house.world')
    model = os.path.join(
        gazebo_share, 'models', 'turtlebot3_waffle_pi_cam', 'model.sdf')
    urdf = os.path.join(
        gazebo_share, 'urdf', 'turtlebot3_waffle_pi_cam.urdf')
    bridge_config = os.path.join(
        gazebo_share, 'params', 'turtlebot3_waffle_pi_cam_bridge.yaml')
    detector_config = os.path.join(
        embodied_share, 'config', 'object_detector.yaml')
    with open(urdf, encoding='utf-8') as stream:
        robot_description = stream.read()

    model_path = LaunchConfiguration('model_path')
    asset_manifest = LaunchConfiguration('asset_manifest')
    output_dir = LaunchConfiguration('output_dir')
    preview = LaunchConfiguration('preview')
    case_index = LaunchConfiguration('case_index')
    intra_op_threads = LaunchConfiguration('intra_op_threads')

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
        arguments=['/camera/image', '/camera/depth_image'],
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen')
    detector = Node(
        package='turtlebot3_embodied_navigation', executable='object_detector',
        parameters=[detector_config, {
            'use_sim_time': True,
            'model_path': model_path,
            'intra_op_threads': ParameterValue(
                intra_op_threads, value_type=int),
        }],
        output='screen')
    runner = Node(
        package='turtlebot3_gazebo', executable='house_perception_acceptance',
        parameters=[{
            'use_sim_time': True,
            'output_dir': output_dir,
            'asset_manifest': asset_manifest,
            'preview': preview,
            'case_index': case_index,
        }],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path', description='Absolute path to the fixed YOLO11n ONNX model'),
        DeclareLaunchArgument(
            'asset_manifest', description='Absolute path to assets.lock.tsv'),
        DeclareLaunchArgument(
            'output_dir', description='New output directory for reports and overlays'),
        DeclareLaunchArgument('preview', default_value='false'),
        DeclareLaunchArgument('case_index', default_value='-1'),
        DeclareLaunchArgument(
            'intra_op_threads',
            default_value=str(min(2, os.cpu_count() or 1)),
            description='Read-only ONNX Runtime intra-op thread count'),
        SetEnvironmentVariable(
            'GZ_SIM_HEADLESS_RENDERING', '1', condition=UnlessCondition(preview)),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
        server,
        client,
        TimerAction(
            period=2.0,
            actions=[spawn_robot, state_publisher, bridge, image_bridge]),
        TimerAction(period=4.0, actions=[detector]),
        TimerAction(period=6.0, actions=[runner]),
        RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[Shutdown()])),
    ])
