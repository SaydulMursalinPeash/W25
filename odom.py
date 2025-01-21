import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import customtkinter as ctk
import math

class OdometryGUI(ctk.CTk, Node):
    def __init__(self):
        ctk.CTk.__init__(self)
        Node.__init__(self, 'odometry_gui')

        self.title("Odometry GUI")
        self.geometry("800x600")

        self.x = 400  # Center of the canvas
        self.y = 300  # Center of the canvas
        self.theta = 0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_sub = None

        self.create_widgets()
        self.update_imu_topics()

        self.last_time = self.get_clock().now()

        # Create IMU data window
        self.imu_window = ctk.CTkToplevel(self)
        self.imu_window.title("IMU Data")
        self.imu_window.geometry("400x300")
        self.imu_label = ctk.CTkLabel(self.imu_window, text="IMU Data: ", font=("Helvetica", 16))
        self.imu_label.pack(pady=10)

    def create_widgets(self):
        self.topic_label = ctk.CTkLabel(self, text="IMU Topic:")
        self.topic_label.pack(pady=10)

        self.topic_var = ctk.StringVar()
        self.topic_menu = ctk.CTkOptionMenu(self, variable=self.topic_var, command=self.update_subscription)
        self.topic_menu.pack(pady=10)

        self.canvas = ctk.CTkCanvas(self, width=800, height=600)
        self.canvas.pack()

    def update_imu_topics(self):
        topics = self.get_topic_names_and_types()
        imu_topics = [name for name, types in topics if 'sensor_msgs/msg/Imu' in types]
        self.topic_menu.configure(values=imu_topics)
        if imu_topics:
            self.topic_var.set(imu_topics[0])
            self.update_subscription(imu_topics[0])
        else:
            print("No IMU topics found")

    def update_subscription(self, topic_name):
        if self.imu_sub:
            self.destroy_subscription(self.imu_sub)
        print(f"Subscribing to IMU topic: {topic_name}")
        self.imu_sub = self.create_subscription(Imu, topic_name, self.imu_callback, 10)

    def imu_callback(self, msg):
        print("Received IMU data")
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.linear_velocity = msg.linear_acceleration.x * dt
        self.angular_velocity = msg.angular_velocity.z * dt

        self.theta += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.theta) * dt * 100  # Scale for better visibility
        self.y += self.linear_velocity * math.sin(self.theta) * dt * 100  # Scale for better visibility

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)

        self.update_canvas()

        # Update IMU data window
        imu_data_text = f"Linear Acceleration:\n  x: {msg.linear_acceleration.x:.2f}\n  y: {msg.linear_acceleration.y:.2f}\n  z: {msg.linear_acceleration.z:.2f}\n\n"
        imu_data_text += f"Angular Velocity:\n  x: {msg.angular_velocity.x:.2f}\n  y: {msg.angular_velocity.y:.2f}\n  z: {msg.angular_velocity.z:.2f}"
        self.imu_label.config(text=imu_data_text)

    def update_canvas(self):
        self.canvas.delete("all")
        self.canvas.create_oval(self.x - 5, self.y - 5, self.x + 5, self.y + 5, fill="blue")
        self.canvas.create_line(self.x, self.y, self.x + 20 * math.cos(self.theta), self.y + 20 * math.sin(self.theta), fill="red")

def main(args=None):
    rclpy.init(args=args)
    app = OdometryGUI()
    app.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()