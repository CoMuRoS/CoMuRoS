#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TimePublisher(Node):
    def __init__(self):
        super().__init__("time_publisher")
        self.publisher_ = self.create_publisher(String, "/current_time", 10)
        self.timer_period = 1.0  # seconds
        self.start_time = time.time()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("TimePublisher node has been started.")

    def timer_callback(self):
        msg = String()
        elapsed = int(time.time() - self.start_time)
        hours, rem = divmod(elapsed, 3600)
        minutes, seconds = divmod(rem, 60)
        # Format elapsed time as HH:MM:SS (like datetime)
        msg.data = f"Hours: {hours:02d}, Minutes: {minutes:02d}, Seconds: {seconds:02d}"
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    time_publisher = TimePublisher()
    try:
        rclpy.spin(time_publisher)
    except KeyboardInterrupt:
        time_publisher.get_logger().info("Keyboard interrupt detected. Shutting down.")
    finally:
        time_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()