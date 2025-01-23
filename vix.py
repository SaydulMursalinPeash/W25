import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from tkinter import Canvas, messagebox
from PIL import Image, ImageTk
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image as SensorImage
from geometry_msgs.msg import Twist
import transforms3d.euler as euler
import math
import numpy as np
import cv2
import sys
from rclpy.executors import MultiThreadedExecutor

class IMUVisualizer(ttk.Window):
    def __init__(self):
        # Use a dark theme for a modern look
        super().__init__(themename="darkly")

        # Flag for controlling ROS spin thread
        self.running = True

        self.title("ROS Sensor & Control Visualizer")
        self.geometry("1400x800")

        # Initialize ROS 2
        rclpy.init(args=None)
        self.node = IMUNode()
        self.camera_node = CameraNode()
        self.teleop_node = TeleopNode()

        # Create executor for handling multiple nodes
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.executor.add_node(self.camera_node)
        self.executor.add_node(self.teleop_node)

        # Start ROS2 spin in separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin_thread, daemon=True)
        self.ros_thread.start()

        # GUI Layout
        self.setup_gui()

        # Bind the window close event
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_gui(self):
        # Create a main container with padding
        self.main_container = ttk.Frame(self, padding=20)
        self.main_container.pack(fill=BOTH, expand=YES)

        # Configure grid layout
        self.main_container.columnconfigure(0, weight=1)
        self.main_container.columnconfigure(1, weight=2)
        self.main_container.columnconfigure(2, weight=1)
        self.main_container.rowconfigure(0, weight=1)

        # Left column - IMU controls
        self.setup_imu_controls()

        # Middle column - Camera feed
        self.setup_camera_controls()

        # Right column - Teleop controls
        self.setup_teleop_controls()

        # Populate initial topics
        self.refresh_topics()
        self.refresh_camera_topics()
        self.refresh_teleop_topics()

        # Update GUI periodically
        self.update_gui_data()

    def setup_imu_controls(self):
        # IMU Frame with a card-like appearance
        imu_frame = ttk.Frame(self.main_container, bootstyle="secondary")
        imu_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Topic Selection
        topic_label = ttk.Label(imu_frame, text="Select IMU Topic:", bootstyle="inverse-secondary")
        topic_label.pack(pady=(10, 5))

        self.topic_var = ttk.StringVar(value="")
        self.topic_dropdown = ttk.Combobox(imu_frame, textvariable=self.topic_var, 
                                           bootstyle="secondary")
        self.topic_dropdown.pack(pady=5, padx=10, fill=X)
        self.topic_dropdown.bind('<<ComboboxSelected>>', 
                                 lambda event: self.update_topic(self.topic_var.get()))

        refresh_button = ttk.Button(imu_frame, text="Refresh Topics", 
                                    command=self.refresh_topics, 
                                    bootstyle="secondary-outline")
        refresh_button.pack(pady=10)

        # IMU Data display
        data_frame = ttk.LabelFrame(imu_frame, text="IMU Data", bootstyle="secondary")
        data_frame.pack(pady=10, padx=10, fill=X)

        self.accel_label = ttk.Label(data_frame, text="Linear Acceleration: x=0.0, y=0.0, z=0.0", 
                                     bootstyle="inverse-secondary")
        self.accel_label.pack(pady=5)
        
        self.angular_label = ttk.Label(data_frame, text="Angular Velocity: x=0.0, y=0.0, z=0.0", 
                                       bootstyle="inverse-secondary")
        self.angular_label.pack(pady=5)

        self.orientation_label = ttk.Label(data_frame, 
                                           text="Orientation: x=0.0, y=0.0, z=0.0, w=0.0", 
                                           bootstyle="inverse-secondary")
        self.orientation_label.pack(pady=5)

        # IMU Direction Indicator
        self.canvas = Canvas(imu_frame, width=300, height=300, bg="white")
        self.canvas.pack(pady=10)
        self.arrow = self.canvas.create_line(150, 150, 150, 50, width=5, arrow="last")

    def setup_camera_controls(self):
        # Camera Frame with a card-like appearance
        camera_frame = ttk.Frame(self.main_container, bootstyle="secondary")
        camera_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Topic Selection
        topic_label = ttk.Label(camera_frame, text="Select Camera Topic:", 
                                bootstyle="inverse-secondary")
        topic_label.pack(pady=(10, 5))

        self.camera_topic_var = ttk.StringVar(value="")
        self.camera_topic_dropdown = ttk.Combobox(camera_frame, 
                                                  textvariable=self.camera_topic_var, 
                                                  bootstyle="secondary")
        self.camera_topic_dropdown.pack(pady=5, padx=10, fill=X)
        self.camera_topic_dropdown.bind('<<ComboboxSelected>>', 
                                        lambda event: self.update_camera_topic(self.camera_topic_var.get()))

        refresh_button = ttk.Button(camera_frame, text="Refresh Camera Topics", 
                                    command=self.refresh_camera_topics, 
                                    bootstyle="secondary-outline")
        refresh_button.pack(pady=10)

        # Debug information display
        self.debug_label = ttk.Label(camera_frame, text="Camera Debug Info: No data", 
                                     bootstyle="inverse-secondary")
        self.debug_label.pack(pady=5)

        # Camera Feed Canvas
        self.camera_canvas = Canvas(camera_frame, width=640, height=360, bg="black")
        self.camera_canvas.pack(pady=20)

        # Store the current PhotoImage
        self.current_image = None

    def setup_teleop_controls(self):
        # Teleop Frame with a card-like appearance
        teleop_frame = ttk.Frame(self.main_container, bootstyle="secondary")
        teleop_frame.grid(row=0, column=2, padx=10, pady=10, sticky="nsew")

        # Topic Selection
        topic_label = ttk.Label(teleop_frame, text="Select Teleop Topic:", 
                                bootstyle="inverse-secondary")
        topic_label.pack(pady=(10, 5))

        self.teleop_topic_var = ttk.StringVar(value="")
        self.teleop_topic_dropdown = ttk.Combobox(teleop_frame, 
                                                  textvariable=self.teleop_topic_var, 
                                                  bootstyle="secondary")
        self.teleop_topic_dropdown.pack(pady=5, padx=10, fill=X)
        self.teleop_topic_dropdown.bind('<<ComboboxSelected>>', 
                                        lambda event: self.update_teleop_topic(self.teleop_topic_var.get()))

        refresh_button = ttk.Button(teleop_frame, text="Refresh Teleop Topics", 
                                    command=self.refresh_teleop_topics, 
                                    bootstyle="secondary-outline")
        refresh_button.pack(pady=10)

        # Teleop control buttons
        button_frame = ttk.Frame(teleop_frame)
        button_frame.pack(pady=10)

        # Create a grid for navigation buttons
        button_style = {"bootstyle": "secondary", "width": 8}
        
        self.up_button = ttk.Button(button_frame, text="▲", 
                                    command=lambda: self.teleop_node.publish_velocity(0.5, 0.0),
                                    **button_style)
        self.up_button.grid(row=0, column=1, pady=5)

        self.left_button = ttk.Button(button_frame, text="◀", 
                                      command=lambda: self.teleop_node.publish_velocity(0.0, 0.5),
                                      **button_style)
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        self.stop_button = ttk.Button(button_frame, text="Stop", 
                                      command=lambda: self.teleop_node.publish_velocity(0.0, 0.0),
                                      **button_style)
        self.stop_button.grid(row=1, column=1, pady=5)

        self.right_button = ttk.Button(button_frame, text="▶", 
                                       command=lambda: self.teleop_node.publish_velocity(0.0, -0.5),
                                       **button_style)
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        self.down_button = ttk.Button(button_frame, text="▼", 
                                      command=lambda: self.teleop_node.publish_velocity(-0.5, 0.0),
                                      **button_style)
        self.down_button.grid(row=2, column=1, pady=5)

    def update_topic(self, topic_name):
        self.node.update_topic(topic_name)

    def refresh_topics(self):
        topics = self.node.get_imu_topics()
        self.topic_dropdown.configure(values=topics)
        if topics:
            self.topic_var.set(topics[0])
            self.update_topic(topics[0])

    def update_camera_topic(self, topic_name):
        self.camera_node.update_topic(topic_name)

    def refresh_camera_topics(self):
        topics = self.camera_node.get_camera_topics()
        self.camera_topic_dropdown.configure(values=topics)
        if topics:
            self.camera_topic_var.set(topics[0])
            self.update_camera_topic(topics[0])

    def update_teleop_topic(self, topic_name):
        self.teleop_node.update_topic(topic_name)

    def refresh_teleop_topics(self):
        topics = self.teleop_node.get_cmd_vel_topics()
        self.teleop_topic_dropdown.configure(values=topics)
        if topics:
            self.teleop_topic_var.set(topics[0])
            self.update_teleop_topic(topics[0])

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

        # Update camera feed
        camera_data = self.camera_node.get_latest_image()
        if camera_data is not None:
            self.update_camera_feed(camera_data)

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

    def update_camera_feed(self, img_msg):
        try:
            # Debug information
            debug_info = (f"Image info - Width: {img_msg.width}, Height: {img_msg.height}, "
                        f"Encoding: {img_msg.encoding}, Step: {img_msg.step}")
            self.debug_label.configure(text=debug_info)

            # Determine image shape based on encoding
            if img_msg.encoding == 'mono8':
                img_array = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    (img_msg.height, img_msg.width)
                )
                img_array = cv2.cvtColor(img_array, cv2.COLOR_GRAY2RGB)
            elif img_msg.encoding == 'bgr8':
                img_array = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    (img_msg.height, img_msg.width, 3)
                )
                img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
            elif img_msg.encoding == 'rgb8':
                img_array = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    (img_msg.height, img_msg.width, 3)
                )
            elif img_msg.encoding in ['16UC1', '32FC1']:
                # Convert depth image to 8-bit grayscale
                if img_msg.encoding == '16UC1':
                    depth_array = np.frombuffer(img_msg.data, dtype=np.uint16).reshape(
                        (img_msg.height, img_msg.width)
                    )
                else:  # 32FC1
                    depth_array = np.frombuffer(img_msg.data, dtype=np.float32).reshape(
                        (img_msg.height, img_msg.width)
                    )
                
                # Normalize depth image
                depth_normalized = cv2.normalize(
                    depth_array, 
                    None, 
                    0, 
                    255, 
                    cv2.NORM_MINMAX, 
                    dtype=cv2.CV_8U
                )
                
                # Apply color map
                img_array = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            else:
                print(f"Unsupported encoding: {img_msg.encoding}")
                return

            # Resize image to fit canvas
            image = Image.fromarray(img_array)
            image = image.resize((640, 360))
            
            # Convert to PhotoImage
            self.current_image = ImageTk.PhotoImage(image)
            
            # Update canvas
            self.camera_canvas.delete("all")
            self.camera_canvas.create_image(
                320, 180,  # Center of canvas
                anchor="center",
                image=self.current_image
            )
            
        except Exception as e:
            print(f"Error updating camera feed: {e}")
            import traceback
            traceback.print_exc()
    def ros_spin_thread(self):
        try:
            while rclpy.ok() and self.running:
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f"Error in ROS spin thread: {e}")
        finally:
            if self.executor:
                self.executor.shutdown()

    def cleanup(self):
        print("Cleaning up resources...")
        self.running = False
        
        if self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
            
        if self.executor:
            self.executor.shutdown()
            
        if self.node:
            self.node.destroy_node()
            
        if self.camera_node:
            self.camera_node.destroy_node()

        if self.teleop_node:
            self.teleop_node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()
        
        print("Cleanup completed")

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            print("Closing application...")
            self.cleanup()
            self.quit()
            self.destroy()
            sys.exit(0)


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_visualizer_node')
        self.subscriber = None
        self.latest_data = None

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

    def get_latest_imu_data(self):
        return self.latest_data

    def get_imu_topics(self):
        topics = self.get_topic_names_and_types()
        return [name for name, types in topics if 'sensor_msgs/msg/Imu' in types]


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_visualizer_node')
        self.subscriber = None
        self.latest_image = None
        self.get_logger().info("Camera node initialized")

    def update_topic(self, topic_name):
        if self.subscriber:
            self.destroy_subscription(self.subscriber)
        self.subscriber = self.create_subscription(
            SensorImage, topic_name, self.camera_callback, 10)

    def camera_callback(self, msg):
        self.latest_image = msg

    def get_latest_image(self):
        return self.latest_image

    def get_camera_topics(self):
        topics = self.get_topic_names_and_types()
        return [name for name, types in topics if 'sensor_msgs/msg/Image' in types]


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_visualizer_node')
        self.publisher = None

    def update_topic(self, topic_name):
        if self.publisher:
            self.destroy_publisher(self.publisher)
        self.publisher = self.create_publisher(Twist, topic_name, 10)

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def get_cmd_vel_topics(self):
        topics = self.get_topic_names_and_types()
        return [name for name, types in topics if 'geometry_msgs/msg/Twist' in types]


if __name__ == "__main__":
    app = IMUVisualizer()
    app.mainloop()