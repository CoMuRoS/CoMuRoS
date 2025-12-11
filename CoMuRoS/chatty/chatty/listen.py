#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Int32
import numpy as np
import torch
import whisper
import threading

class WhisperSubscriber(Node):
    def __init__(self):
        super().__init__("whisper_subscriber")

        self.declare_parameter("model_name", "medium")
        self.declare_parameter("use_cuda", True)  

        model_name = self.get_parameter("model_name").value
        use_cuda = self.get_parameter("use_cuda").value

        self.audio_sub = self.create_subscription(Float32MultiArray, "/audio_stream",self.audio_callback, 10)
        self.switch_sub = self.create_subscription(String, "/switch_state",self.switch_callback, 10)

        self.pub = self.create_publisher(String, "/chat/input", 10)
        self.face_mode_pub = self.create_publisher(Int32, "/face_mode", 10)

        self.device = "cuda" if (torch.cuda.is_available() and use_cuda) else "cpu"
        self.model = whisper.load_model(model_name, device=self.device)
        self.get_logger().info(f"Loaded Whisper model '{model_name}' on {self.device}")
        self.audio_buffer = []
        self.buffering = False
        self.switch_state = None  
        self.prev_switch_state = None  
        self.buffer_lock = threading.Lock()
        self.get_logger().info("Ready: Switch to PRESSED to start buffering, RELEASED to stop and transcribe.")

    def switch_callback(self, msg):
        """Callback for /speaking_switch_state topic."""
        self.switch_state = msg.data

    def audio_callback(self, msg):
        with self.buffer_lock:
            if self.buffering:
                audio_chunk = np.array(msg.data, dtype=np.float32)
                self.audio_buffer.append(audio_chunk)
                self.get_logger().debug(f"Buffered chunk: {len(audio_chunk)} samples, shape: {audio_chunk.shape}")

    def transcribe_and_publish(self):
        with self.buffer_lock:
            local_buffer = self.audio_buffer.copy()
            self.audio_buffer = []
        if local_buffer and len(local_buffer) * len(local_buffer[0]) > 16000:  # Ensure at least 1 second of audio at 16kHz
            audio = np.concatenate(local_buffer)
            self.get_logger().info(f"Running transcription on buffered audio ({len(audio)} samples)...")
            result = self.model.transcribe(audio, fp16=(self.device == "cuda"), language="en")
            text = result["text"].strip()
            if text:
                msg_out = String()
                msg_out.data = f"human|{text}"  # Prepend 'human|' to the transcribed text
                self.pub.publish(msg_out)
                self.get_logger().info(f"Transcript: {text}")
            else:
                self.get_logger().warn("Whisper returned empty text, skipping publish. (Expected for non-English input)")
            self.get_logger().info("Transcription complete. Ready for next: Switch to PRESSED to start buffering.")
        else:
            self.get_logger().warn(f"No sufficient audio buffered ({len(local_buffer)} chunks).")

def main(args=None):
    if not rclpy.ok():  # Only initialize if not already initialized
        rclpy.init(args=args)
    node = WhisperSubscriber()
    last_state_change = 0.0
    debounce_duration = 0.5  # 0.5 seconds debounce
    try:
        while rclpy.ok():
            current_switch = node.switch_state
            current_time = node.get_clock().now().to_msg().sec + node.get_clock().now().to_msg().nanosec * 1e-9
            # Publish 1 to face_mode topic continuously while switch is PRESSED
            if current_switch == "PRESSED":
                face_mode_msg = Int32()
                face_mode_msg.data = 1
                node.face_mode_pub.publish(face_mode_msg)
            # Detect state transitions with debouncing
            if (current_switch == "PRESSED" and node.prev_switch_state != "PRESSED" and
                not node.buffering and (current_time - last_state_change) > debounce_duration):
                node.buffering = True
                with node.buffer_lock:
                    node.audio_buffer = []
                node.get_logger().info("Started buffering audio...")
                last_state_change = current_time
            if (current_switch == "RELEASED" and node.prev_switch_state == "PRESSED" and
                node.buffering and (current_time - last_state_change) > debounce_duration):
                node.buffering = False
                # Publish 0 to face_mode topic once when switch is RELEASED
                face_mode_msg = Int32()
                face_mode_msg.data = 0
                node.face_mode_pub.publish(face_mode_msg)
                threading.Thread(target=node.transcribe_and_publish).start()  # Non-blocking transcription
                last_state_change = current_time
            node.prev_switch_state = current_switch
            rclpy.spin_once(node, timeout_sec=0.1)  # Non-blocking spin
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        if rclpy.ok():  # Only shutdown if context is still active
            rclpy.shutdown()

if __name__ == "__main__":
    main()