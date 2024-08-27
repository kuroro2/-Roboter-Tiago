import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coke_detector',  # Replace with your package name
            executable='coke_detector',  # Replace with your detector node executable name
            name='coke_detector',
            output='screen'
        ),
        Node(
            package='coke_detector',  # Replace with your package name
            executable='arm_move',  # Replace with your arm move node executable name
            name='arm_move',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('coke_detector'), 'config', 'rviz_config.rviz')]  # Replace with your rviz config file if you have one
        ),
        ])

