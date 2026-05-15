import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        self.timer = self.create_timer(0.1, self.measure_and_publish_distance)

        # ✅ Reset GPIO before setting up (prevents conflicts)
        GPIO.cleanup()  
        GPIO.setmode(GPIO.BCM)

        self.GPIO_TRIGGER = 18
        self.GPIO_ECHO = 24
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

        self.get_logger().info("Ultrasonic Sensor Node Started")

    def measure_distance(self):
        try:
            GPIO.output(self.GPIO_TRIGGER, True)
            time.sleep(0.00001)
            GPIO.output(self.GPIO_TRIGGER, False)

            StartTime = time.time()
            StopTime = time.time()
            timeout = StartTime + 0.02  # ✅ Prevents infinite loop if no response

            while GPIO.input(self.GPIO_ECHO) == 0:
                StartTime = time.time()
                if StartTime > timeout:  
                    return None  # Timeout -> return None

            while GPIO.input(self.GPIO_ECHO) == 1:
                StopTime = time.time()
                if StopTime > timeout:  
                    return None  # Timeout -> return None

            TimeElapsed = StopTime - StartTime
            distance = (TimeElapsed * 34300) / 2  # Convert time to cm

            return distance if 2 <= distance <= 400 else None  # ✅ Ignore invalid values

        except RuntimeError as e:
            self.get_logger().error(f"Sensor error: {str(e)}")
            return None  

    def measure_and_publish_distance(self):
        dist = self.measure_distance()
        if dist is not None:
            self.get_logger().info(f'Measured Distance: {dist:.1f} cm')
            msg = Float32()
            msg.data = dist
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Invalid distance measurement, ignoring.")

    def cleanup(self):
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up.")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Ultrasonic Sensor Node")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
