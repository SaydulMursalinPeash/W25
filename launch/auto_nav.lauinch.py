from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare map file argument
        DeclareLaunchArgument(
            'map',
            default_value='~/sim_ws/src/W25/map/wirehouse_map.yaml',
            description='Full path to the map file'
        ),
        # Declare RViz config file argument
        DeclareLaunchArgument(
            'rviz_config',
            default_value='~/sim_ws/src/W25/config/rviz_config.rviz',
            description='Full path to the RViz config file'
        ),
        # Build the workspace
        ExecuteProcess(
            cmd=['colcon', 'build'],
            cwd='~/sim_ws',
            shell=True,
            output='screen'
        ),
        # Source the ROS 2 environment
        ExecuteProcess(
            cmd=['source', '~/.bashrc'],
            shell=True,
            output='screen'
        ),
        # Launch the bridge
        ExecuteProcess(
            cmd=['ros2', 'launch', 'W25', 'bridge.launch.py'],
            shell=True,
            output='screen'
        ),
        # Launch AMCL
        ExecuteProcess(
            cmd=['ros2', 'launch', 'W25', 'amcl.launch.py'],
            shell=True,
            output='screen'
        ),
        # Configure and activate map_server
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
            shell=True,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
            shell=True,
            output='screen'
        ),
        # Configure and activate AMCL
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
            shell=True,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
            shell=True,
            output='screen'
        ),
        # Launch Nav2 bringup
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                'map:=' + LaunchConfiguration('map')
            ],
            shell=True,
            output='screen'
        ),
        # Launch RViz with the specified config file
        ExecuteProcess(
            cmd=[
                'rviz2',
                '-d', LaunchConfiguration('rviz_config')
            ],
            shell=True,
            output='screen'
        )
    ])