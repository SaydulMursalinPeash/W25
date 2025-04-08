import customtkinter as ctk
from tkinter import Canvas, messagebox
from tkinter.scrolledtext import ScrolledText
from PIL import Image, ImageTk
import threading
import subprocess
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

class IMUVisualizer(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Flag for controlling ROS spin thread
        self.running = True

        self.title("IMU, Camera, and Teleop Visualizer")
        self.geometry("1200x800")  # Increased width to accommodate the new column

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
        # Main frame
        self.main_frame = ctk.CTkFrame(self)
        self.main_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)  # Use grid layout for the main frame

        # Left side - IMU controls
        self.setup_imu_controls()

        # Middle column - Camera feed
        self.setup_camera_controls()

        # Right column - Teleop controls
        self.setup_teleop_controls()

        # Ensure the window expands as needed
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)  # IMU controls column
        self.grid_columnconfigure(1, weight=2)  # Camera feed column
        self.grid_columnconfigure(2, weight=1)  # Teleop controls column

        # Populate initial topics
        self.refresh_topics()
        self.refresh_camera_topics()
        self.refresh_teleop_topics()

        # Update GUI periodically
        self.update_gui_data()

    def setup_imu_controls(self):
        self.control_frame = ctk.CTkFrame(self.main_frame)
        self.control_frame.grid(row=0, column=0, padx=20, sticky="nsew")

        self.topic_label = ctk.CTkLabel(self.control_frame, text="Select IMU Topic:")
        self.topic_label.pack(pady=10)

        self.topic_var = ctk.StringVar(value="")
        self.topic_dropdown = ctk.CTkOptionMenu(self.control_frame, variable=self.topic_var, 
                                              command=self.update_topic)
        self.topic_dropdown.pack(pady=10)

        self.refresh_topics_button = ctk.CTkButton(self.control_frame, 
                                                 text="Refresh IMU Topics", 
                                                 command=self.refresh_topics)
        self.refresh_topics_button.pack(pady=10)

        # IMU Data display
        self.data_frame = ctk.CTkFrame(self.control_frame)
        self.data_frame.pack(pady=20, fill="both", expand=True)

        self.accel_label = ctk.CTkLabel(self.data_frame, 
                                      text="Linear Acceleration: x=0.0, y=0.0, z=0.0")
        self.accel_label.pack(pady=5)
        
        self.angular_label = ctk.CTkLabel(self.data_frame, 
                                        text="Angular Velocity: x=0.0, y=0.0, z=0.0")
        self.angular_label.pack(pady=5)

        self.orientation_label = ctk.CTkLabel(self.data_frame, 
                                           text="Orientation: x=0.0, y=0.0, z=0.0, w=0.0")
        self.orientation_label.pack(pady=5)

        # IMU Direction Indicator
        self.canvas = Canvas(self.control_frame, width=300, height=300, bg="white")
        self.canvas.pack(pady=10)
        self.arrow = self.canvas.create_line(150, 150, 150, 50, width=5, arrow="last")

        # Add Bridge and AutoNav buttons
        self.bridge_button = ctk.CTkButton(self.control_frame, text="Bridge", command=self.run_bridge)
        self.bridge_button.pack(pady=10)

        self.auto_nav_button = ctk.CTkButton(self.control_frame, text="AutoNav", command=self.run_auto_nav)
        self.auto_nav_button.pack(pady=10)

        # Add a console output window
        self.console_output = ScrolledText(self.control_frame, height=10, wrap="word", state="disabled")
        self.console_output.pack(pady=10, fill="both", expand=True)

    def run_bridge(self):
        """Run the Bridge commands."""
        commands = [
            "cd ~/sim_ws",
            "source ~/.bashrc",
            "colcon build",
            "ros2 launch W25 bridge.launch.py"
        ]
        self.execute_commands(commands)

    def run_auto_nav(self):
        """Run the AutoNav commands."""
        commands = [
            "ros2 launch W25 amcl.launch.py",
            "ros2 lifecycle set /map_server configure",
            "ros2 lifecycle set /map_server activate",
            "ros2 lifecycle set /amcl configure",
            "ros2 lifecycle set /amcl activate",
            "ros2 launch nav2_bringup bringup_launch.py map:=~/sim_ws/src/W25/map/wirehouse_map.yaml",
            "rviz2"
        ]
        self.execute_commands(commands)

    def execute_commands(self, commands):
        """Execute a list of shell commands in a separate thread and display output in the console."""
        def run_commands():
            self.console_output.configure(state="normal")
            self.console_output.delete(1.0, "end")  # Clear previous output

            for command in commands:
                self.console_output.insert("end", f"$ {command}\n")
                self.console_output.see("end")
                try:
                    result = subprocess.run(command, shell=True, text=True, capture_output=True, check=True)
                    self.console_output.insert("end", result.stdout)
                except subprocess.CalledProcessError as e:
                    self.console_output.insert("end", e.stderr)
                self.console_output.see("end")

            self.console_output.configure(state="disabled")

        # Run the commands in a separate thread
        threading.Thread(target=run_commands, daemon=True).start()

    def setup_camera_controls(self):
        self.visual_frame = ctk.CTkFrame(self.main_frame)
        self.visual_frame.grid(row=0, column=1, padx=20, sticky="nsew")

        self.camera_topic_label = ctk.CTkLabel(self.visual_frame, text="Select Camera Topic:")
        self.camera_topic_label.pack(pady=10)

        self.camera_topic_var = ctk.StringVar(value="")
        self.camera_topic_dropdown = ctk.CTkOptionMenu(self.visual_frame, 
                                                     variable=self.camera_topic_var, 
                                                     command=self.update_camera_topic)
        self.camera_topic_dropdown.pack(pady=10)

        self.refresh_camera_topics_button = ctk.CTkButton(self.visual_frame, 
                                                        text="Refresh Camera Topics", 
                                                        command=self.refresh_camera_topics)
        self.refresh_camera_topics_button.pack(pady=10)

        # Debug information display
        self.debug_label = ctk.CTkLabel(self.visual_frame, text="Camera Debug Info: No data")
        self.debug_label.pack(pady=5)

        # Camera Feed Canvas - making it more compact
        self.camera_canvas = Canvas(self.visual_frame, width=640, height=360, bg="black")
        self.camera_canvas.pack(pady=20)

        # Store the current PhotoImage
        self.current_image = None

    def setup_teleop_controls(self):
        # Teleop controls moved to the right column
        self.teleop_frame = ctk.CTkFrame(self.main_frame)
        self.teleop_frame.grid(row=0, column=2, padx=20, sticky="nsew")

        self.teleop_topic_label = ctk.CTkLabel(self.teleop_frame, text="Select Teleop Topic:")
        self.teleop_topic_label.pack(pady=10)

        self.teleop_topic_var = ctk.StringVar(value="")
        self.teleop_topic_dropdown = ctk.CTkOptionMenu(self.teleop_frame, 
                                                    variable=self.teleop_topic_var, 
                                                    command=self.update_teleop_topic)
        self.teleop_topic_dropdown.pack(pady=10)

        self.refresh_teleop_topics_button = ctk.CTkButton(self.teleop_frame, 
                                                        text="Refresh Teleop Topics", 
                                                        command=self.refresh_teleop_topics)
        self.refresh_teleop_topics_button.pack(pady=10)

        # Teleop control buttons in a vertical arrangement
        self.button_frame = ctk.CTkFrame(self.teleop_frame)
        self.button_frame.pack(pady=10)

        # Create a vertical layout for buttons
        button_size = 60  # Define button size to make them round

        # Up-Down-Left-Right with Stop button at the center
        self.up_button = ctk.CTkButton(self.button_frame, text="▲", 
                                       command=lambda: self.teleop_node.publish_velocity(0.5, 0.0),
                                       width=button_size, height=button_size)
        self.up_button.grid(row=0, column=1, pady=5)

        self.left_button = ctk.CTkButton(self.button_frame, text="◀", 
                                         command=lambda: self.teleop_node.publish_velocity(0.0, 0.5),
                                         width=button_size, height=button_size)
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        self.stop_button = ctk.CTkButton(self.button_frame, text="Stop", 
                                         command=lambda: self.teleop_node.publish_velocity(0.0, 0.0),
                                         width=button_size, height=button_size)
        self.stop_button.grid(row=1, column=1, pady=5)

        self.right_button = ctk.CTkButton(self.button_frame, text="▶", 
                                          command=lambda: self.teleop_node.publish_velocity(0.0, -0.5),
                                          width=button_size, height=button_size)
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        self.down_button = ctk.CTkButton(self.button_frame, text="▼", 
                                        command=lambda: self.teleop_node.publish_velocity(-0.5, 0.0),
                                        width=button_size, height=button_size)
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
            # Update debug information
            debug_info = (f"Image info - Width: {img_msg.width}, Height: {img_msg.height}, "
                        f"Encoding: {img_msg.encoding}, Step: {img_msg.step}")
            self.debug_label.configure(text=debug_info)

            # Convert ROS image message to numpy array
            img_array = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, -1
            )

            # Handle different encodings
            if img_msg.encoding == 'bgr8':
                img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
            elif img_msg.encoding == 'rgb8':
                pass  # Already in RGB format
            elif img_msg.encoding == 'mono8':
                img_array = cv2.cvtColor(img_array, cv2.COLOR_GRAY2RGB)
            elif img_msg.encoding == '16UC1':
                # Normalize depth image to 0-255 for display
                img_array = cv2.normalize(img_array, None, 0, 255, cv2.NORM_MINMAX)
                img_array = np.uint8(img_array)
                img_array = cv2.applyColorMap(img_array, cv2.COLORMAP_JET)
            elif img_msg.encoding == '32FC1':
                # Normalize depth image to 0-255 for display
                img_array = cv2.normalize(img_array, None, 0, 255, cv2.NORM_MINMAX)
                img_array = np.uint8(img_array)
                img_array = cv2.applyColorMap(img_array, cv2.COLORMAP_JET)
            else:
                print(f"Unsupported encoding: {img_msg.encoding}")
                return

            # Convert numpy array to PIL Image
            image = Image.fromarray(img_array)
            
            # Resize image to fit canvas
            image = image.resize((640, 360))  # Reduced height to make it more compact
            
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

