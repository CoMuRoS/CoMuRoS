import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

class UltrasonicConverter(Node):
    def __init__(self):
        super().__init__('ultrasonic_converter')
        self.subscription = self.create_subscription(
            Range,
            '/waffle/ultrasonic_link/out',
            self.range_callback,
            10
        )
        self.publisher = self.create_publisher(Float32, '/distance', 10)

    def range_callback(self, msg):
        if msg.range == float('inf') or msg.range != msg.range:  # Check for NaN or Inf
            self.get_logger().warn("Invalid range value received")
            return
        
        distance_cm = msg.range * 100.0  # Convert meters to cm
        distance_msg = Float32()
        distance_msg.data = distance_cm
        self.publisher.publish(distance_msg)
        self.get_logger().info(f'Published Distance: {distance_cm:.2f} cm')


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
