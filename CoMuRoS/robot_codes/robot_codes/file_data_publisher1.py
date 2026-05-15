#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time


class FileDataPublisher(Node):
    def __init__(self):
        super().__init__('file_data_publisher')
        self.get_logger().info("File Data Publisher Node started")
        # self.publisher_ = self.create_publisher(String, '/file_data', 10)
        self.publisher_1 = self.create_publisher(String, '/chat/task_status', 10)
        # self.publisher_2 = self.create_publisher(String, '/chat/input', 10)
        # self.robot_state_pub = self.create_publisher(String, '/robot_states', 10)
        # self.go2_state = self.create_publisher(String, '/go2_task_status', 10)


        self.file_path = '/home/vipul/ws/mia_ws/src/robot_codes/robot_codes/data1.txt'
        self.last_position = 0  # Track last read position
        self.in_prog = True

        # Ensure file exists, else create it
        if not os.path.exists(self.file_path):
            open(self.file_path, 'w').close()

        # Set last_position to file's current end (ignoring past data)
        self.last_position = os.path.getsize(self.file_path)

        # Timer to check file updates
        self.timer1 = self.create_timer(1.0, self.check_file_update)  # Checks every second
        # self.timer2 = self.create_timer(1.0, self.task_progress)  # Checks every second


    def check_file_update(self):
        try:

            file_size = os.path.getsize(self.file_path)
            # If file was truncated, reset last_position
            if file_size < self.last_position:
                self.last_position = 0

            with open(self.file_path, 'r') as f:
                f.seek(self.last_position)  # Move to last read position
                new_data = f.read()  # Read new data

                if new_data:  # If new data is available
                    msg = String()
                    
                    msg.data = new_data.strip()  # Strip to remove unwanted spaces/newlines
                    self.publisher_1.publish(msg)
                    # msg.data = "Go2 (msg) My tasks are complete."
                    # self.publisher_2.publish(msg)

                    self.get_logger().info(f'Published: {msg.data}')
                
                self.last_position = f.tell()  # Update last position
        except Exception as e:
            self.get_logger().error(f'Error reading file: {e}')
    
    # def task_progress(self):
        

    #     try:
    #         with open("data1.txt", 'r') as file:
    #             status = file.read()
    #         msg = String()
    #         if "IN PROGRESS" in status:
    #             msg.data = status
    #             self.go2_state.publish(msg)
    #             self.in_prog = True

    #         if self.in_prog:
    #             if "TASKS COMPLETE" in status:
    #                 msg.data = status
    #                 self.go2_state.publish(msg)
    #                 self.in_prog = False

        
        # except Exception as e:
        #     print(f"Error reading file data1.txt: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = FileDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

