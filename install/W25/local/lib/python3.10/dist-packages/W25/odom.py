import customtkinter as ctk
from tkinter import Canvas, messagebox
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import transforms3d.euler as euler
import math
import sys

class IMUVisualizer(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.running = True

        self.title("IMU Visualizer with Odometry")
        self.geometry("800x600")

        # Initialize ROS 2
        rclpy.init(args=None)
        self.node = IMUNode()

        # Start ROS2 spin in a separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin_thread, daemon=True)
        self.ros_thread.start()

        # GUI Layout
        self.setup_gui()

        # Bind the window close event
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_gui(self):
        # Main frame
        self.main_frame = ctk.CTkFrame(self)
        self.main_frame.pack(fill="both", expand=True, padx=20, pady=20)

        # IMU controls
        self.topic_label = ctk.CTkLabel(self.main_frame, text="Select IMU Topic:")
        self.topic_label.pack(pady=10)

        self.topic_var = ctk.StringVar(value="")
        self.topic_dropdown = ctk.CTkOptionMenu(self.main_frame, variable=self.topic_var, 
                                                command=self.update_topic)
        self.topic_dropdown.pack(pady=10)

        self.refresh_topics_button = ctk.CTkButton(self.main_frame, text="Refresh IMU Topics", 
                                                  command=self.refresh_topics)
        self.refresh_topics_button.pack(pady=10)

        # IMU Data display
        self.data_frame = ctk.CTkFrame(self.main_frame)
        self.data_frame.pack(pady=20, fill="both", expand=True)

        self.accel_label = ctk.CTkLabel(self.data_frame, text="Linear Acceleration: x=0.0, y=0.0, z=0.0")
        self.accel_label.pack(pady=5)

        self.angular_label = ctk.CTkLabel(self.data_frame, text="Angular Velocity: x=0.0, y=0.0, z=0.0")
        self.angular_label.pack(pady=5)

        self.orientation_label = ctk.CTkLabel(self.data_frame, text="Orientation: x=0.0, y=0.0, z=0.0, w=0.0")
        self.orientation_label.pack(pady=5)

        self.odometry_label = ctk.CTkLabel(self.data_frame, text="Odometry: x=0.0, y=0.0, theta=0.0")
        self.odometry_label.pack(pady=5)

        # IMU Direction Indicator
        self.canvas = Canvas(self.data_frame, width=300, height=300, bg="white")
        self.canvas.pack(pady=10)
        self.arrow = self.canvas.create_line(150, 150, 150, 50, width=5, arrow="last")

        # Populate initial topics
        self.refresh_topics()

        # Update GUI periodically
        self.update_gui_data()

    def update_topic(self, topic_name):
        self.node.update_topic(topic_name)

    def refresh_topics(self):
        topics = self.node.get_imu_topics()
        self.topic_dropdown.configure(values=topics)
        if topics:
            self.topic_var.set(topics[0])
            self.update_topic(topics[0])

    def update_gui_data(self):
        if not self.running:
            return

        # Update IMU data
        data = self.node.get_latest_imu_data()
        if data:
            self.accel_label.configure(
                text=f"Linear Acceleration: x={data['linear_acceleration'].x:.2f}, "
                     f"y={data['linear_acceleration'].y:.2f}, z={data['linear_acceleration'].z:.2f}")
            self.angular_label.configure(
                text=f"Angular Velocity: x={data['angular_velocity'].x:.2f}, "
                     f"y={data['angular_velocity'].y:.2f}, z={data['angular_velocity'].z:.2f}")
            self.orientation_label.configure(
                text=f"Orientation: x={data['orientation'].x:.2f}, "
                     f"y={data['orientation'].y:.2f}, z={data['orientation'].z:.2f}, "
                     f"w={data['orientation'].w:.2f}")
            self.update_direction_indicator(data['orientation'])

            # Calculate and update odometry
            odometry = self.node.calculate_odometry()
            if odometry:
                self.odometry_label.configure(
                    text=f"Odometry: x={odometry['x']:.2f}, y={odometry['y']:.2f}, theta={odometry['theta']:.2f}")

        self.after(100, self.update_gui_data)

    def update_direction_indicator(self, orientation):
        roll, pitch, yaw = euler.quat2euler(
            [orientation.w, orientation.x, orientation.y, orientation.z]
        )
        length = 100
        center_x, center_y = 150, 150
        end_x = center_x + length * math.cos(yaw)
        end_y = center_y - length * math.sin(yaw)
        self.canvas.coords(self.arrow, center_x, center_y, end_x, end_y)

    def ros_spin_thread(self):
        try:
            while rclpy.ok() and self.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"Error in ROS spin thread: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.cleanup()
            self.quit()
            self.destroy()
            sys.exit(0)

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_visualizer_node')
        self.subscriber = None
        self.latest_data = None

        # Variables for odometry calculation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

    def update_topic(self, topic_name):
        if self.subscriber:
            self.destroy_subscription(self.subscriber)
        self.subscriber = self.create_subscription(
            Imu, topic_name, self.imu_callback, 10)

    def imu_callback(self, msg):
        self.latest_data = {
            'linear_acceleration': msg.linear_acceleration,
            'angular_velocity': msg.angular_velocity,
            'orientation': msg.orientation
        }

        # Calculate odometry
        self.calculate_odometry_from_msg(msg)

    def get_latest_imu_data(self):
        return self.latest_data

    def get_imu_topics(self):
        topics = self.get_topic_names_and_types()
        return [name for name, types in topics if 'sensor_msgs/msg/Imu' in types]

    def calculate_odometry_from_msg(self, msg):
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # Use linear acceleration and angular velocity to update position
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        omega = msg.angular_velocity.z

        self.theta += omega * dt
        self.x += 0.5 * ax * dt**2 * math.cos(self.theta)
        self.y += 0.5 * ay * dt**2 * math.sin(self.theta)

    def calculate_odometry(self):
        return {'x': self.x, 'y': self.y, 'theta': self.theta}

if __name__ == "__main__":
    app = IMUVisualizer()
    app.mainloop()
