from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_conf = os.path.join(get_package_share_directory('W25'),'map','tugbot_map.yaml')
    amcl_conf = os.path.join(get_package_share_directory('W25'),'config','amcl.yaml')
    return LaunchDescription(
        [
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': map_conf}
                ]
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                output='screen',
                name='amcl',
                parameters=[
                    amcl_conf
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            )
        ]
    )



######    ros2 lifecycle set /map_server configure
######    ros2 lifecycle set /map_server activate
######    ros2 lifecycle set /amcl configure
######    ros2 lifecycle set /amcl activate