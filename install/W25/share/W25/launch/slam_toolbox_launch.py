import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file', default=os.path.join(
        get_package_share_directory('W25'), 'config', 'slam_config.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()


# ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/peash/mobile_ws/src/W25/config/slam_config.yaml use_sim_time:=true