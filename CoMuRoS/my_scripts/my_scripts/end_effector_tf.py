#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = "base_link"  # Replace with your actual base frame
        self.source_frame = "end_effector_link"  # Replace with your actual end-effector frame

        self.get_logger().info("TransformListenerNode initialized.")

        # Wait and query the transform
        self.timer = self.create_timer(2.0, self.query_transform)  # Check every 2 seconds

    def query_transform(self):
        try:
            # Get the transform between the frames
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),  # Get the latest transform
                timeout=rclpy.time.Duration(seconds=5.0)
            )

            # Extract translation
            translation = transform.transform.translation
            x, y, z = translation.x, translation.y, translation.z

            # Extract rotation (quaternion)
            rotation = transform.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]

            # Convert quaternion to RPY
            rpy_rad = euler_from_quaternion(quaternion)
            rpy_deg = [angle * 180.0 / 3.14159 for angle in rpy_rad]

            # Log the transformation
            self.get_logger().info(f"Transform from '{self.source_frame}' to '{self.target_frame}':")
            self.get_logger().info(f" - Translation: x={x:.3f}, y={y:.3f}, z={z:.3f}")

            # Print Quaternion
            self.get_logger().info(f" - Quaternion: x={quaternion[0]:.5f}, y={quaternion[1]:.5f}, "
                                   f"z={quaternion[2]:.5f}, w={quaternion[3]:.5f}")

            # Print RPY (Roll, Pitch, Yaw)
            self.get_logger().info(f" - Orientation (RPY in degrees): roll={rpy_deg[0]:.2f}°, "
                                   f"pitch={rpy_deg[1]:.2f}°, yaw={rpy_deg[2]:.2f}°")

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = TransformListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

## To pick
# ros2 service call /set_end_effector_pose hello_moveit_interfaces/srv/SetPose "{pose: {position: {x: 0.207, y: -0.004, z: 0.083}, orientation: {x: 0.00262, y: 0.37984, z: -0.00639, w: 0.92503}}}"

# response:
# hello_moveit_interfaces.srv.SetPose_Response(success=True)

## After Lifiting
#ros2 service call /set_end_effector_pose hello_moveit_interfaces/srv/SetPose "{pose: {position: {x: 0.179, y: -0.005, z: 0.3}, orientation: {x: 0.00011, y: 0.01150, z: -0.00920, w: 0.99989}}}"

## To place on Go2
#ros2 service call /set_end_effector_pose hello_moveit_interfaces/srv/SetPose "{pose: {position: {x: 0.234, y: -0.012, z: 0.245}, orientation: {x: -0.00088, y: -0.04599, z: -0.01915, w: 0.99876}}}"


## Go to Home
#ros2 service call /set_end_effector_pose hello_moveit_interfaces/srv/SetPose "{pose: {position: {x: 0.179, y: -0.005, z: 0.3}, orientation: {x: 0.00011, y: 0.01150, z: -0.00920, w: 0.99989}}}"
