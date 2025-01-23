from launch import LaunchDescription
from launch_ros.actions import Node
from tf_transformations import quaternion_from_euler

def generate_launch_description():
    # Convert Euler angles to quaternions
    back_lidar_quat = quaternion_from_euler(0.0, 0.0, 3.141592654)
    front_lidar_quat = quaternion_from_euler(0.0, 0.0, 0.0)
    front_camera_quat = quaternion_from_euler(0.0, 0.0, 0.0)
    back_camera_quat = quaternion_from_euler(0.0, 0.0, 3.141592654)

    return LaunchDescription([
        # Static transform for back LIDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_lidar_tf',
            arguments=[
                '-0.2075', '0.0', '0.205',
                str(back_lidar_quat[0]), str(back_lidar_quat[1]), str(back_lidar_quat[2]), str(back_lidar_quat[3]),
                'base_link', 'tugbot/scan_back/scan_back'
            ]
        ),
        # Static transform for front LIDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_lidar_tf',
            arguments=[
                '0.221', '0.0', '0.1404',
                str(front_lidar_quat[0]), str(front_lidar_quat[1]), str(front_lidar_quat[2]), str(front_lidar_quat[3]),
                'base_link', 'tugbot/scan_front/scan_front'
            ]
        ),
        # Static transform for front camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_camera_tf',
            arguments=[
                '0.0553', '0.0', '0.4323',
                str(front_camera_quat[0]), str(front_camera_quat[1]), str(front_camera_quat[2]), str(front_camera_quat[3]),
                'base_link', 'tugbot/camera_front/camera_front'
            ]
        ),
        # Static transform for back camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_camera_tf',
            arguments=[
                '-0.241', '0.0', '0.2303',
                str(back_camera_quat[0]), str(back_camera_quat[1]), str(back_camera_quat[2]), str(back_camera_quat[3]),
                'base_link', 'tugbot/camera_back/camera_back'
            ]
        ),
    ])
