import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('turtlebot3_teleop')
    teleop_config = os.path.join(package_share, 'config', 'xbox_teleop.yaml')
    smoother_config = os.path.join(
        package_share, 'config', 'xbox_velocity_smoother.yaml')
    device_id = LaunchConfiguration('device_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='Linux joystick device index'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': ParameterValue(device_id, value_type=int),
                'deadzone': 0.15,
                'autorepeat_rate': 20.0,
            }]),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[teleop_config, {'publish_stamped_twist': True}],
            remappings=[('/cmd_vel', '/cmd_vel_joy')]),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[smoother_config],
            remappings=[
                ('/cmd_vel', '/cmd_vel_joy'),
                ('/cmd_vel_smoothed', '/cmd_vel'),
            ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_xbox_teleop',
            output='screen',
            parameters=[smoother_config]),
    ])
