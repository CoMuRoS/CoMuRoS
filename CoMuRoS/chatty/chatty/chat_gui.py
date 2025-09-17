#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import customtkinter as ctk
from threading import Thread
from datetime import datetime  

class ChatGUI(Node):
    def __init__(self):
        super().__init__("human_gui")
        
        self.publisher = self.create_publisher(String, "/chat/input", 10)
        self.subscription = self.create_subscription(String, "/chat/output", self.on_output, 10)

        # Set theme
        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        # Updated Color Palette
        self.colors = {
            "bg_light": "#f0f0f0", # Light background
            "human_msg": "#3F88C5", # Blue 
            "task_msg": "#FFA851",  # Light Orange
            "button_hover": "#ff6f61", # Light Coral
            "go2_msg": '#888888',  # Grey 
            "burger_msg": "#008080",  # Teal 
            "waffle_msg": "#9932a8",  # Purple 
            "drone_msg" : "#4E5734", # Grey
            "formation_msg" : "#6B6DE6", # Grey
            "x_arm_msg": "#3A9026"  # Purple 
            # here you can add more colors for different robots or messages
        }

        self.window = ctk.CTk()
        self.window.title("Interactive Chat Interface")
        self.window.geometry("700x600")
        self.window.configure(fg_color=self.colors["bg_light"])

        # Header
        self.header_frame = ctk.CTkFrame(self.window, fg_color=self.colors["human_msg"], corner_radius=0)  
        self.header_frame.pack(fill='x', pady=(0, 10))
        
        self.header_label = ctk.CTkLabel(
            self.header_frame, 
            text="CHAT INTERFACE", 
            font=("Montserrat", 22, "bold"),  
            text_color="#ffffff"
        )
        self.header_label.pack(pady=10)
        
        # Chat area (Scrollable)
        self.chat_area = ctk.CTkScrollableFrame(self.window, fg_color=self.colors["bg_light"])
        self.chat_area.pack(padx=15, pady=10, fill='both', expand=True)

        # Input area
        entry_frame = ctk.CTkFrame(self.window, fg_color=self.colors["human_msg"], corner_radius=10, height=70)
        entry_frame.pack(fill='x', padx=15, pady=15)
        entry_frame.pack_propagate(False)

        # Entry field
        self.entry = ctk.CTkEntry(
            entry_frame, 
            font=("Montserrat", 14, "bold"),  
            placeholder_text="Type your message here...",
            height=35,
            corner_radius=20,
            fg_color="#ffffff",
            text_color="#000000",
        )
        self.entry.pack(side='left', expand=True, fill='x', padx=10, pady=(0, 10))
        self.entry.bind("<Return>", self.send_message)

        # Send button
        send_btn = ctk.CTkButton(
            entry_frame, 
            text="SEND", 
            command=self.send_message, 
            font=("Montserrat", 14, "bold"),  
            fg_color="#ff5252",
            hover_color=self.colors["button_hover"],
            corner_radius=20,
            width=100,
            height=35
        )
        send_btn.pack(side='right', padx=10, pady=(0, 10))

        self.get_logger().info("[ChatGUI] GUI node initialized.")

        self.history_fetched = False
        self.window.after(500, self.fetch_history_once)

    def on_output(self, msg):
        line = msg.data
        self.get_logger().info(f"[ChatGUI] /chat/output -> {line}")
        
        # Remove `[dd:mm:yy timestamp]` part
        if "]" in line:
            line = line.split("] ", 1)[-1]

        self.window.after(0, lambda: self.append_text(line))

    def append_text(self, text_line):
        # Get timestamp in HH:MM format
        timestamp = datetime.now().strftime("%H:%M")

        if "Human:" in text_line:
            message_color = self.colors["human_msg"]
            label_text = "Human"
            text_line = text_line.replace("Human:", "").strip()
            align = "e"

        elif "Burger (msg)" in text_line :
            text_line = text_line.replace("Unknown:", "").strip()
            text_line = text_line.replace("(msg)", "").strip()
            text_line = text_line.replace(":", "").strip()
            message_color = self.colors["burger_msg"]
            label_text = "Burger"
            text_line = text_line.replace("Burger", "").strip()
            align = "w"

        elif "Waffle (msg)" in text_line:
            text_line = text_line.replace("Unknown:", "").strip()
            text_line = text_line.replace("(msg)", "").strip()
            text_line = text_line.replace(":", "").strip()
            message_color = self.colors["waffle_msg"]
            label_text = "Waffle"
            text_line = text_line.replace("Waffle", "").strip()
            align = "w"

        elif "Go2 (msg)" in text_line:
            text_line = text_line.replace("Unknown:", "").strip()
            text_line = text_line.replace(":", "").strip()
            text_line = text_line.replace("(msg)", "").strip()
            message_color = self.colors["go2_msg"]
            label_text = "Go2"
            text_line = text_line.replace("Go2", "").strip()
            align = "w"

        elif "Drone (msg)" in text_line:
            text_line = text_line.replace("Unknown:", "").strip()
            text_line = text_line.replace(":", "").strip()
            text_line = text_line.replace("(msg)", "").strip()
            message_color = self.colors["drone_msg"]
            label_text = "Drone"
            text_line = text_line.replace("Drone", "").strip()
            align = "w"

        elif "Formation (msg)" in text_line:
            text_line = text_line.replace("Unknown:", "").strip()
            text_line = text_line.replace(":", "").strip()
            text_line = text_line.replace("(msg)", "").strip()
            message_color = self.colors["formation_msg"]
            label_text = "Formation"
            text_line = text_line.replace("Formation", "").strip()
            align = "w"

        elif "X Arm (msg)" in text_line:
            text_line = text_line.replace("Unknown:", "").strip()
            text_line = text_line.replace(":", "").strip()
            text_line = text_line.replace("(msg)", "").strip()
            message_color = self.colors["x_arm_msg"]
            label_text = "X Arm"
            text_line = text_line.replace("X Arm", "").strip()
            align = "w"

        elif "Taskmanager:" in text_line or "Unknown" in text_line :
            text_line = text_line.replace("Unknown:", "").strip()
            message_color = self.colors["task_msg"]
            label_text = "Task-Master"
            text_line = text_line.replace("Taskmanager:", "").strip()
            align = "w"

        else:
            message_color = "#ffffff"
            label_text = "System"
            align = "w"

        # Message frame
        message_frame = ctk.CTkFrame(self.chat_area, fg_color=message_color, corner_radius=10)
        message_frame.pack(fill="x", padx=10, pady=5, anchor=align)

        # Label (Sender + Timestamp)
        label = ctk.CTkLabel(
            message_frame,
            text=f"{label_text} • {timestamp}",
            font=("Montserrat", 15, "bold"),
            text_color="#ffffff",
            justify="left"
        )
        label.pack(anchor="w", padx=10, pady=(5, 0))

        # Message text
        message_label = ctk.CTkLabel(
            message_frame,
            text=text_line,
            font=("Montserrat", 14, "bold"),
            text_color="#ffffff",
            wraplength=500,
            justify="left"
        )
        message_label.pack(padx=10, pady=(0, 5), anchor="w")

        self.chat_area.update_idletasks()
        self.chat_area._parent_canvas.yview_moveto(1)

    def send_message(self, event=None):
        user_input = self.entry.get().strip()
        if user_input:
            out_msg = String()
            out_msg.data = f"human|{user_input}"
            self.publisher.publish(out_msg)
            self.get_logger().info(f"[ChatGUI] Sent -> {user_input}")
        self.entry.delete(0, 'end')

    def fetch_history_once(self):
        if self.history_fetched:
            return
        self.get_logger().info("[ChatGUI] Attempting to fetch old chat.")
        client = self.create_client(Trigger, "get_chat_history")
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("[ChatGUI] Manager not ready, skipping old history fetch.")
            self.history_fetched = True
            return
        
        req = Trigger.Request()
        future = client.call_async(req)

        def check_done():
            if future.done():
                res = future.result()
                if res and res.success:
                    lines = res.message.split("\n")
                    self.get_logger().info(f"[ChatGUI] Received {len(lines)} lines of old chat.")
                    for line in lines:
                        if line.strip():
                            self.window.after(0, lambda l=line: self.append_text(l))
                else:
                    self.get_logger().error("[ChatGUI] Could not fetch old history or manager gave error.")
                self.history_fetched = True
            else:
                self.window.after(200, check_done)
        
        self.window.after(200, check_done)

    def run_gui(self):
        self.get_logger().info("[ChatGUI] Starting Tkinter mainloop.")
        self.window.mainloop()


def main():
    rclpy.init()
    node = ChatGUI()
    
    def spin_bg():
        rclpy.spin(node)
    
    t = Thread(target=spin_bg, daemon=True)
    t.start()

    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()