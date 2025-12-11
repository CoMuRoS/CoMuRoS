#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket, threading, time

class SwitchServer(Node):
    def __init__(self):
        super().__init__('esp_switch_server')
        self.publisher_ = self.create_publisher(String, '/switch_state', 10)
        self.HOST, self.PORT = "0.0.0.0", 5050
        threading.Thread(target=self.tcp_server, daemon=True).start()
        self.get_logger().info(f"SwitchServer started on {self.HOST}:{self.PORT}")

    def tcp_server(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.HOST, self.PORT))
        s.listen(1)
        s.setblocking(False)
        self.get_logger().info("Waiting for ESP32 connection...")

        conn = None
        last_recv = time.time()

        while rclpy.ok():
            # 1. Accept new client
            if conn is None:
                try:
                    conn, addr = s.accept()
                    conn.setblocking(False)
                    self.get_logger().info(f"Connected by {addr}")
                    last_recv = time.time()
                except BlockingIOError:
                    time.sleep(0.1)
                    continue

            # 2. Read data (non-blocking)
            try:
                data = conn.recv(1024)
                if data:
                    msg_str = data.decode(errors='ignore').strip()
                    msg = String()
                    msg.data = msg_str
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"{msg_str}")
                    last_recv = time.time()
                else:
                    raise ConnectionError("peer closed")
            except BlockingIOError:
                # No data yet
                pass
            except Exception as e:
                self.get_logger().warn(f"Lost connection: {e}")
                try: conn.close()
                except: pass
                conn = None
                continue

            # 3. Timeout: 10 s of silence → drop connection
            if conn and (time.time() - last_recv > 10):
                # self.get_logger().warn("No data for 10 s — closing connection")
                # self.get_logger().warn("No data for 10 s ")
                pass
                # try: conn.close()
                # except: pass
                # conn = None

            # time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = SwitchServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
