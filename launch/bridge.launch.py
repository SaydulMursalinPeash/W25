from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('W25'), 'launch', 'bridge_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            parameters=[
                {'config_file': config_file_path}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_lidar_tf',
            arguments=['-0.2075', '0.0', '0.205', '0.0', '0.0', '1.0', '0.0', 'base_link', 'tugbot/scan_back/scan_back']
        ),
        # Static transform for front LIDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_lidar_tf',
            arguments=['0.221', '0.0', '0.1404', '0.0', '0.0', '0.0', 'base_link', 'tugbot/scan_front/scan_front']
        ),
        # Static transform for front camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_camera_tf',
            arguments=['0.0553', '0.0', '0.4323', '0.0', '0.0', '0.0', 'base_link', 'tugbot/camera_front/camera_front']
        ),
        # Static transform for back camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_camera_tf',
            arguments=['-0.241', '0.0', '0.2303', '0.0', '0.0', '3.141592654', 'base_link', 'tugbot/camera_back/camera_back']
        ),
    ])
