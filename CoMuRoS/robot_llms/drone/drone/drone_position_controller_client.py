#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interface.srv import GotoPoseDrone

NAMESPACE = 'r3'

class DroneGotoClient(Node):
    def __init__(self):
        super().__init__("drone_goto_client")
        self.declare_parameter('namespace', NAMESPACE)
        self.namespace: str = self.get_parameter('namespace').value

        # Create service client
        self.cli = self.create_client(GotoPoseDrone, f"/{self.namespace}/goto_pose")

        # Wait for server
        self.get_logger().info(f"Waiting for /{self.namespace}/goto_pose service...")
        self.cli.wait_for_service()
        self.get_logger().info("Service available.")

    def send_goal(self, x, y, z, yaw_deg):
        req = GotoPoseDrone.Request()
        req.x = x
        req.y = y
        req.z = z
        req.yaw_deg = yaw_deg

        self.get_logger().info(
            f"Sending drone goto request: x={x}, y={y}, z={z}, yaw={yaw_deg}°"
        )

        # Synchronous blocking call
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            res = future.result()
            self.get_logger().info(f"[CLIENT] Response: success={res.success}, msg={res.message}")
            return res.success
        else:
            self.get_logger().error("Service call failed.")
            return False


def main(args=None):
    rclpy.init(args=args)

    node = DroneGotoClient()

    # Modify this goal as needed:
    # node.send_goal(x=5.0, y=-1.0, z=5.3, yaw_deg=90.0)
    node.send_goal(x=5.0, y=-1.0, z=5.3, yaw_deg=90.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


'''

ign service -s /world/food_court/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'name: "r3" position: { x: 5.0 y: -1.0 z: 5.3 } orientation: { x: 0.0 y: 0.0 z: 0.7071 w: 0.7071 }'




'''