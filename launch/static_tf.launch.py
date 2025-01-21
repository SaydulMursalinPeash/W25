from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform for back LIDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_lidar_tf',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '1.0', '0.0', 'base_link', 'tugbot/scan_back/scan_back']
        ),
        # Static transform for front LIDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_lidar_tf',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'tugbot/scan_front/scan_front']
        ),
    ])
