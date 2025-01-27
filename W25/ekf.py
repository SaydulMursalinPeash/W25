#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.linalg import inv

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__('ekf_sensor_fusion_node')
        
        # Declare parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        
        # Initialize state vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        self.state = np.zeros(10)
        self.state[0] = self.get_parameter('initial_x').value
        self.state[1] = self.get_parameter('initial_y').value
        self.state[2] = self.get_parameter('initial_z').value
        self.state[6] = 1.0  # Initial quaternion with no rotation
        
        # Covariance matrices
        self.covariance = np.eye(10) * 0.1
        self.Q = np.eye(10) * 0.01  # Process noise
        self.R_imu = np.eye(6) * 0.1  # IMU measurement noise
        self.R_odom = np.eye(6) * 0.05  # Odometry measurement noise
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/w25/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/w25/odom', self.odom_callback, 10)
        
        # Publishers
        self.fused_odom_pub = self.create_publisher(Odometry, '/fused_odometry', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Time tracking
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('EKF Sensor Fusion Node Initialized')
        
    def predict_state(self, dt):
        # State extraction
        x, y, z = self.state[0:3]
        vx, vy, vz = self.state[3:6]
        qw, qx, qy, qz = self.state[6:10]
        
        # Position update
        self.state[0] = x + vx * dt
        self.state[1] = y + vy * dt
        self.state[2] = z + vz * dt
        
        # Quaternion integration
        omega = np.array([0, self.state[7], self.state[8], self.state[9]])
        q = np.array([qw, qx, qy, qz])
        q_dot = 0.5 * np.array(self.quaternion_multiply(omega, q))
        
        self.state[6:10] += q_dot * dt
        self.state[6:10] /= np.linalg.norm(self.state[6:10])
        
        # Compute Jacobian
        F = np.eye(10)
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Update covariance
        self.covariance = F @ self.covariance @ F.T + self.Q
        
    def update_imu(self, acc, gyro):
        # IMU measurement update
        H = np.zeros((6, 10))
        H[0:3, 3:6] = np.eye(3)  # Velocity
        H[3:6, 6:10] = self.compute_quaternion_derivative()  # Ensure the shape matches (3, 4)
        
        # Innovation
        z_pred = np.concatenate([self.state[3:6], self.state[6:10]])
        z_meas = np.concatenate([acc, gyro])
        y = z_meas - z_pred
        
        # Kalman gain
        S = H @ self.covariance @ H.T + self.R_imu
        K = self.covariance @ H.T @ inv(S)
        
        # State and covariance update
        self.state += K @ y
        self.covariance = (np.eye(10) - K @ H) @ self.covariance
        
    def update_odom(self, pose, twist):
        # Odometry measurement update
        H = np.eye(10)[:6, :]
        
        # Innovation
        z_pred = np.concatenate([self.state[0:3], self.state[3:6]])
        z_meas = np.concatenate([pose, twist])
        y = z_meas - z_pred
        
        # Kalman gain
        S = H @ self.covariance @ H.T + self.R_odom
        K = self.covariance @ H.T @ inv(S)
        
        # State and covariance update
        self.state += K @ y
        self.covariance = (np.eye(10) - K @ H) @ self.covariance
        
    def publish_fused_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = self.state[2]
        
        # Orientation
        odom.pose.pose.orientation.w = self.state[6]
        odom.pose.pose.orientation.x = self.state[7]
        odom.pose.pose.orientation.y = self.state[8]
        odom.pose.pose.orientation.z = self.state[9]
        
        # Velocity
        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.linear.y = self.state[4]
        odom.twist.twist.linear.z = self.state[5]
        
        # Publish odometry
        self.fused_odom_pub.publish(odom)
        
        # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)
        
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Predict step
        self.predict_state(dt)
        
        # Update with IMU data
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.update_imu(acc, gyro)
        
        self.last_time = current_time
        self.publish_fused_odometry()
        
    def odom_callback(self, msg):
        # Update with Odometry data
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.update_odom(pose, twist)
        
        self.publish_fused_odometry()
        
    def quaternion_multiply(self, q1, q2):
        # Quaternion multiplication helper
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return [
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ]
        
    def compute_quaternion_derivative(self):
        # Example implementation that returns a (3, 4) matrix
        # This is a placeholder implementation; adjust as needed for your system
        qw, qx, qy, qz = self.state[6:10]
        
        # Compute the quaternion derivative matrix
        dq_dt = np.array([
            [-qx, -qy, -qz],
            [ qw, -qz,  qy],
            [ qz,  qw, -qx],
            [-qy,  qx,  qw]
        ])
        
        return dq_dt[:3, :]  # Return a (3, 4) matrix

def main(args=None):
    rclpy.init(args=args)
    node = ExtendedKalmanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()