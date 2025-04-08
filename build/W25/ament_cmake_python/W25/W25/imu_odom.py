#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, PoseWithCovariance, TwistWithCovariance
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')
        
        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # Subscriber
        self.imu_subscription = self.create_subscription(
            Imu, '/w25/imu', self.imu_callback, 10)
        
        # Publisher
        self.odom_publisher = self.create_publisher(Odometry, '/imu_odom', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.current_position = np.zeros(3)
        self.current_orientation = R.from_quat([0, 0, 0, 1])
        self.current_velocity = np.zeros(3)
        
        # Time tracking
        self.last_time = self.get_clock().now()
        
        # Integration parameters
        self.gravity = 9.81  # m/s^2
        
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Extract acceleration and angular velocity
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # Calculate rotation delta
        rotation_delta = R.from_rotvec([0, 0, gyro_z * dt])
        
        # Update orientation
        self.current_orientation *= rotation_delta
        
        # Update position
        self.current_velocity += np.array([acc_x, acc_y, acc_z]) * dt
        self.current_position += self.current_velocity * dt
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set the position
        odom_msg.pose.pose.position.x = self.current_position[0]
        odom_msg.pose.pose.position.y = self.current_position[1]
        odom_msg.pose.pose.position.z = self.current_position[2]
        
        # Set the orientation
        quat = self.current_orientation.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Set the velocity
        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.linear.z = self.current_velocity[2]
        odom_msg.twist.twist.angular.x = gyro_x
        odom_msg.twist.twist.angular.y = gyro_y
        odom_msg.twist.twist.angular.z = gyro_z
        
        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)
        
        # Update last time
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = IMUOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()