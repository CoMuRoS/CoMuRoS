#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os
import datetime

class ChatManager(Node):
    """
    ChatManager Node:
      - Routes messages between users and robots.
      - Stores chat history and provides it on request.
      - Loads previous history on startup and appends new messages on shutdown.
    """

    def __init__(self):
        super().__init__("chat_manager")
        self.chat_log = []

        self.get_logger().info("[ChatManager] Initializing...")

        # Load previous chat history from file
        # self.load_history()

        # Subscriptions and Publishers
        self.create_subscription(String, "/chat/input", self.handle_input, 10)
        self.output_pub = self.create_publisher(String, "/chat/output", 10)
        self.history_pub = self.create_publisher(String, "/chat/history", 10)

        # History Retrieval Service
        self.create_service(Trigger, "get_chat_history", self.handle_history)

        package_name = "chatty"
        directry = "data"
        package_path = get_package_share_directory(package_name)

        script_name = "chat_history.txt"
        self.file_path = os.path.join(package_path, directry, script_name)

        self.get_logger().info("[ChatManager] Ready and running.")

    def load_history(self):
        """Loads previous chat history from file."""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, "r") as file:
                    self.chat_log = file.read().splitlines()
                self.get_logger().info(f"[ChatManager] Loaded {len(self.chat_log)} previous messages.")
            except Exception as e:
                self.get_logger().error(f"[ChatManager] Failed to load history: {e}")

    def handle_input(self, msg):
        """Handles incoming chat messages and distributes them."""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        parts = msg.data.split("|", 1)

        if len(parts) == 2:
            role, content = parts[0].strip(), parts[1].strip()
        else:
            role, content = "unknown", msg.data

        chat_entry = f"[{timestamp}] {role.capitalize()}: {content}"
        self.get_logger().info(f"[ChatManager] Received on /input-> {chat_entry}")

        # Store in chat history
        self.chat_log.append(chat_entry)

        # Publish to output topic
        out_msg = String()
        out_msg.data = chat_entry
        self.output_pub.publish(out_msg)

        # Publish full history
        history_msg = String()
        history_msg.data = "\n".join(self.chat_log)
        self.history_pub.publish(history_msg)

    def handle_history(self, request, response):
        """Handles chat history requests from clients."""
        response.success = True
        response.message = "\n".join(self.chat_log)
        self.get_logger().info(f"[ChatManager] Returning {len(self.chat_log)} chat entries.")
        return response

    def save_history(self):
        """Appends new chat messages to history file on shutdown."""
        try:
            with open(self.file_path, "a") as file:
                for line in self.chat_log:
                    file.write(line + "\n")
            self.get_logger().info(f"[ChatManager] Appended {len(self.chat_log)} new messages to {self.file_path}.")
        except Exception as e:
            self.get_logger().error(f"[ChatManager] Failed to save history: {e}")

    def destroy_node(self):
        """Handles cleanup before shutdown."""
        self.save_history()
        super().destroy_node()


def main():
    """Entry point for the ChatManager node."""
    rclpy.init()
    node = ChatManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[ChatManager] Shutting down...")
    finally:
        node.destroy_node()
        # node.save_history()
        rclpy.shutdown()

if __name__ == "__main__":
    main()