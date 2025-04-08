#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SyncTopicsNode(Node):
    def __init__(self):
        super().__init__('sync_topics_node')

        # Set QoS Profile
        qos_profile = QoSProfile(
            history=QoSProfile.HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers to /scan and /odom topics with QoS
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_profile)
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/odom', qos_profile=qos_profile)

        # ApproximateTimeSynchronizer with slop for more flexibility
        self.ts = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.odom_sub], 10, 0.5)
        self.ts.registerCallback(self.callback_function)

    def callback_function(self, scan_msg, odom_msg):
        # This is where you process the synchronized data
        self.get_logger().info('Received synchronized messages:')
        self.get_logger().info(f'Scan: {scan_msg.header.stamp}, Odometry: {odom_msg.header.stamp}')
        
        # You can add further processing here
        # Example: process scan data and odometry together for SLAM or any other application

def main(args=None):
    rclpy.init(args=args)

    sync_topics_node = SyncTopicsNode()

    rclpy.spin(sync_topics_node)

    sync_topics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
