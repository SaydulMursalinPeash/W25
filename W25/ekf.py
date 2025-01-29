#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TwistWithCovariance, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # Initialize state [x, y, theta, v, omega]
        self.state = np.zeros(5)  # [x, y, theta, v, omega]
        self.P = np.eye(5)  # State covariance
        self.Q = np.eye(5) * 0.1  # Process noise (adjust as needed)
        self.R = np.eye(3) * 0.1  # Measurement noise (for x, y, theta)

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, '/w25/odom', self.odom_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, '/w25/imu', self.imu_callback, 10
        )

        # Publisher for fused odometry
        self.pub_fused_odom = self.create_publisher(Odometry, '/odom', 10)

        # Timer for prediction step (adjust frequency as needed)
        self.dt = 0.1  # Time step
        self.timer = self.create_timer(self.dt, self.predict_step)

        # Store latest measurements
        self.latest_odom = None
        self.latest_imu = None

    def odom_callback(self, msg):
        # Store wheel odometry data (v and omega)
        self.latest_odom = msg
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self.state[3] = v  # Update velocity
        self.state[4] = omega  # Update angular velocity

    def imu_callback(self, msg):
        # Store IMU data (orientation and angular velocity)
        self.latest_imu = msg
        # Extract theta from IMU quaternion (simplified)
        q = msg.orientation
        theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        omega_imu = msg.angular_velocity.z  # Angular velocity from IMU
        self.update_step(theta, omega_imu)

    def predict_step(self):
        if self.latest_odom is None:
            return

        # Get velocity and angular velocity from wheel odometry
        v = self.state[3]
        omega = self.state[4]
        theta = self.state[2]

        # State transition matrix
        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt, np.cos(theta) * self.dt, 0],
            [0, 1, v * np.cos(theta) * self.dt, np.sin(theta) * self.dt, 0],
            [0, 0, 1, 0, self.dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        # Predict state
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q

    def update_step(self, theta_measurement, omega_measurement):
        # Measurement matrix (H maps state to measurement [x, y, theta, omega])
        H = np.array([
            [1, 0, 0, 0, 0],  # For x position
            [0, 1, 0, 0, 0],  # For y position
            [0, 0, 1, 0, 0],  # For theta (orientation)
        ])

        # Measurement vector z (including the IMU theta and omega)
        z = np.array([self.state[0], self.state[1], theta_measurement])

        # Compute Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state with IMU measurements
        self.state = self.state + K @ (z - H @ self.state)
        self.P = (np.eye(5) - K @ H) @ self.P

        # Optionally, update angular velocity (omega) from IMU angular velocity
        self.state[4] = omega_measurement  # This directly updates omega

        # Publish fused odometry
        self.publish_fused_odom()

    def publish_fused_odom(self):
        fused_odom = Odometry()
        fused_odom.header.stamp = self.get_clock().now().to_msg()
        fused_odom.header.frame_id = 'odom'
        fused_odom.child_frame_id = 'base_link'

        # Populate pose
        fused_odom.pose.pose.position.x = self.state[0]
        fused_odom.pose.pose.position.y = self.state[1]
        fused_odom.pose.pose.orientation.z = np.sin(self.state[2] / 2)
        fused_odom.pose.pose.orientation.w = np.cos(self.state[2] / 2)

        # Populate twist (v and omega from wheel odometry)
        if self.latest_odom:
            fused_odom.twist = self.latest_odom.twist

        self.pub_fused_odom.publish(fused_odom)

def main(args=None):
    rclpy.init(args=args)
    ekf_node = ExtendedKalmanFilter()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
