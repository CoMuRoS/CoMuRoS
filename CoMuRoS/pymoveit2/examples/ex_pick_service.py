#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros
import numpy as np

class OwlViTDepthExtractor(Node):
    def __init__(self):
        super().__init__('owlvit_depth_extractor')

        # Create subscribers
        self.create_subscription(Detection2DArray, '/arm2/owlvit_detections', self.detection_callback, 10)
        self.create_subscription(Image, '/arm2/depth_camera_sensor/depth/image_raw', self.depth_callback, 10)
        self.create_timer(0.2,self.publish_tfs)

        # Depth image and bridge
        self.depth_image = None
        self.bridge = CvBridge()
        self.detections = [] # clear old ones each time
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)   

        # Camera Intrinsic Parameters 
        self.fx = 381.36
        self.fy = 381.36
        self.cx = 320.5
        self.cy = 240.5


    def depth_callback(self, msg):
        try:
            self.depth_encoding = msg.encoding  # NEW: store encoding
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def get_depth_at(self, x, y):
        if self.depth_image is None or self.depth_encoding is None:
            return None

        x = int(round(x))
        y = int(round(y))

        if y >= self.depth_image.shape[0] or x >= self.depth_image.shape[1]:
            return None

        depth = self.depth_image[y, x]

        if self.depth_encoding == "16UC1":
            if depth == 0 or depth >= 9000:  # filter out invalid/too far values
                return None
            depth = depth / 1000.0
        elif self.depth_encoding == "32FC1":
            if np.isnan(depth) or depth <= 0.0 or depth >= 9.0:
                return None
        else:
            self.get_logger().error(f"Unsupported depth encoding: {self.depth_encoding}")
            return None

        return depth

    def detection_callback(self, msg):
        # for detection in msg.detections:
        #     cx = detection.bbox.center.position.x
        #     cy = detection.bbox.center.position.y

        #     # Loop over all classification results
        #     for result in detection.results:
        #         class_id = result.hypothesis.class_id
        #         score = result.hypothesis.score

        #         depth = self.get_depth_at(cx, cy)
        #         if depth is not None:
        #             self.get_logger().info(
        #                 f"Class: {class_id} (score: {score:.2f}) :: center: (cX={cx:.1f}, cY={cy:.1f}) -> Depth: {depth:.3f} meters"
        #             )
        #             self.detected_objects[class_id] = {
        #                 "cx": cx,
        #                 "cy": cy,
        #                 "depth": depth,
        #             }
                    
        #         else:
        #             self.get_logger().warn(
        #                 f"Class: {class_id} :: Could not retrieve depth at (cX={cx:.1f}, cY={cy:.1f})"
        #             )

        self.detections = []  # list of dicts or dataclass objects

        # Inside detection_callback:
        self.detections.clear()

        for detection in msg.detections:
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y

            if len(detection.results) == 0:
                continue  # skip if no class prediction

            top_result = detection.results[0]  # use only the top-scored class
            class_id = top_result.hypothesis.class_id
            score = top_result.hypothesis.score

            depth = self.get_depth_at(cx, cy)
            if depth is None:
                continue

            self.detections.append({
                "class_id": class_id,
                "cx": cx,
                "cy": cy,
                "depth": depth,
                "score": score
            })

    def publish_tfs(self):
        if self.depth_image is None: 
            print('Cannot read image...')
            return        
        
        if self.detections is [] : 
            print('Detections are cleared... ')
            return 

        for detection in self.detections:  
            class_id = detection['class_id']  
            cx = detection['cx']
            cy = detection['cy']
            distance = detection['depth']

            x = (cx - self.cx) * distance / self.fx
            y = (cy - self.cy) * distance / self.fy
            z = float(distance)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_link"
            t.child_frame_id = f"obj_{class_id.replace(' ', '_')}"

            q = quaternion_from_euler(0,0,0)
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OwlViTDepthExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
