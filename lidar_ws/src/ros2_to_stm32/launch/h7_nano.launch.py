"""Headless MID-360 and H743 MINI bringup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('ros2_to_stm32'))
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='H743 USART3 USB serial device'),
        DeclareLaunchArgument('user_config_path',
                              default_value=str(package / 'config/mid360_real.json')),
        DeclareLaunchArgument('lio_params',
                              default_value=str(package / 'config/fast_lio_mid360_real.yaml')),
        DeclareLaunchArgument('publish_map', default_value='false',
                              description='Publish the accumulated FAST-LIO map'),
        DeclareLaunchArgument('show_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/real_lidar.launch.py')),
            launch_arguments={
                'show_rviz': LaunchConfiguration('show_rviz'),
                'publish_map': LaunchConfiguration('publish_map'),
                'lio_params': LaunchConfiguration('lio_params'),
                'user_config_path': LaunchConfiguration('user_config_path'),
            }.items(),
        ),
        Node(package='ros2_to_stm32', executable='pos_trans', output='screen'),
        Node(package='ros2_to_stm32', executable='h7_mini', output='screen',
             parameters=[{'port': LaunchConfiguration('serial_port')}]),
    ])
