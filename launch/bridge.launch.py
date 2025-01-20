from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')
    
    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name',
        description='Name of the ros_gz_bridge node'
    )
    
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        description='Path to the config file'
    )
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=bridge_name,
        parameters=[config_file],
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(bridge_node)
    
    return ld