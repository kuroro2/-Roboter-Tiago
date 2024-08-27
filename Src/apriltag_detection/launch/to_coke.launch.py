import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        
    pkg_share = get_package_share_directory("apriltag_detection")
    config_file = os.path.join(pkg_share, "config", "config_file.yaml")


    
    launch_description = LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('image_rect', '/head_front_camera/rgb/image_raw'),
                ('camera_info', '/head_front_camera/rgb/camera_info')
            ],
        ),
        Node(
            package='apriltag_detection',
            executable='to_coke',
            output='screen',
        )
    ])

    return launch_description
