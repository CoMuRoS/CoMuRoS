#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sounddevice as sd
import numpy as np
import scipy.signal

class AudioPublisher(Node):
    def __init__(self):
        super().__init__("microphone")

        # Declare ROS parameters
        self.declare_parameter("device_index", 5)
        self.declare_parameter("fs_device", 48000)
        self.declare_parameter("fs_target", 16000)
        self.declare_parameter("chunk_duration", 0.5)  # Duration of each small chunk

        # Retrieve parameter values
        self.device_index = self.get_parameter("device_index").value
        self.fs_device = self.get_parameter("fs_device").value
        self.fs_target = self.get_parameter("fs_target").value
        self.chunk_duration = self.get_parameter("chunk_duration").value

        self.pub = self.create_publisher(Float32MultiArray, "/audio_stream", 10)
        self.get_logger().info(
            f"AudioPublisher started: mic={self.device_index}, streaming chunks of {self.chunk_duration}s at {self.fs_device}Hz"
        )
        self.timer = self.create_timer(self.chunk_duration, self.publish_chunk)

    def publish_chunk(self):
        self.get_logger().debug("Capturing audio chunk...")
        frames = int(self.fs_device * self.chunk_duration)
        audio_chunk = sd.rec(frames, samplerate=self.fs_device, channels=1,
                             dtype="float32", device=self.device_index)
        sd.wait()
        audio_chunk = np.squeeze(audio_chunk)

        # Downsample chunk for whisper
        if self.fs_device != self.fs_target:
            audio_chunk = scipy.signal.resample(
                audio_chunk, int(len(audio_chunk) * self.fs_target / self.fs_device)
            )

        msg = Float32MultiArray()
        msg.data = audio_chunk.tolist()
        self.pub.publish(msg)
        self.get_logger().debug(f"Published {len(msg.data)} samples (downsampled to {self.fs_target}Hz).")

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()