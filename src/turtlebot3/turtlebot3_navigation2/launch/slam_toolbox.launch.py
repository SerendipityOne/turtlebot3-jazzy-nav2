import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('turtlebot3_navigation2')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')

    default_params = os.path.join(
        navigation_share, 'param', 'slam_toolbox.yaml')
    slam_launch = os.path.join(
        slam_toolbox_share, 'launch', 'online_sync_launch.py')
    rviz_config = os.path.join(
        slam_toolbox_share, 'config', 'slam_toolbox_default.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for mapping'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params,
            description='Full path to Slam Toolbox parameters'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='slam_rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])