#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg  import Imu



class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.get_logger().info('Kalman Filter Node has been started')
        self.odom_subscriber_ = self.create_subscription(Odometry,"/w25/odom",self.odom_callback,10)
        self.imu_subscriber_ = self.create_subscription(Imu,"/w25/imu",self.imu_callback,10)
        self.odom_publisher_ = self.create_publisher(Odometry,"/w25/odom_filtered",10)
        self.mean_ = 0.0
        self.variance_ = 1000
        self.imu_angular_z_ = 0.0
        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0
        self.kalman_odom = Odometry()
        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5
    def measurement_update(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) / (self.variance_ + self.measurement_variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_)
    def odom_callback(self,msg):
        self.kalman_odom = msg
        if self.is_first_odom_:
            self.mean_ = msg.twist.twist.angular.z
            self.last_angular_z_ = msg.twist.twist.angular.z
            self.is_first_odom_ = False
            return 
        self.StatePrediction()
        self.StateUpdate()

    def imu_callback(self,msg):
        self.imu_angular_z_ = msg.angular_velocity.z

