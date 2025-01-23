#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random

class OdomNoisyNode(Node):
    def __init__(self):
        super().__init__('odom_noisy_node')
        self.get_logger().info('Odom Noisy Node has been started')
        self.odom_subscriber_ = self.create_subscription(Odometry, "/w25/odom", self.odom_callback, 10)
        self.noisy_odom_publisher_ = self.create_publisher(Odometry, "/w25/odom_noisy", 10)

    def odom_callback(self, msg):
        noisy_odom = Odometry()
        noisy_odom.header = msg.header
        noisy_odom.child_frame_id = msg.child_frame_id
        noisy_odom.pose = msg.pose
        noisy_odom.twist = msg.twist

        # Add noise to the angular z component
        noisy_odom.twist.twist.angular.z +=  random.gauss(0, 0.05)  # Mean 0, standard deviation 0.1

        # Publish the noisy odometry data
        self.noisy_odom_publisher_.publish(noisy_odom)

def main(args=None):
    rclpy.init(args=args)
    odom_noisy_node = OdomNoisyNode()
    rclpy.spin(odom_noisy_node)
    odom_noisy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()