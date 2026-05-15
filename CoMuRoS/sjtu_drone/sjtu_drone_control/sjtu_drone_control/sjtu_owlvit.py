#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import threading
from transformers import OwlViTProcessor, OwlViTForObjectDetection
from PIL import Image, ImageDraw, ImageFont
import torch
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as Img
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from std_msgs.msg import Header

class OwlViTDetectionNode(Node):
    def __init__(self):
        super().__init__('simple_drone_owlvit_detection_node')

        # Quality of Service profile for image subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.declare_parameter('use_sim', False)
        self.use_sim = self.get_parameter('use_sim').value

        self.subscription = self.create_subscription(
            Img,
            '/simple_drone/bottom/image_raw',
            self.image_callback,
            qos_profile)

        # Create a publisher for the detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/simple_drone/owlvit_detections',
            10)

        self.img_publisher = self.create_publisher(
            CompressedImage,
            '/simple_drone/image_processed/compressed',
            10)
        
        self.bridge = CvBridge()

        # Frame counter for skipping frames
        self.frame_counter = 0
        self.last_processed_frame = None
        self.processed_frame = None
        self.lock = threading.Lock()

        # -------------------------------
        # 1. Define Synonym Groups
        # -------------------------------
        # Keep the same synonyms
        self.synonym_groups = {

            "human": [
                "a person",
                "a human being",
                "a man",
                "a woman",
                "a child",
                "a girl",
                "a skin color being",
                "a fallen woman",
                "an adult human",
                "a black dress",
                "a dark figure",
                "a person standing",
                "a person walking",
                "a human figure",
                "a standing figure of a person",
                "a person with two legs and arms",
                "an individual with skin and clothes",
                "a biological human",
                "a real person, not a robot"
            ],

            "robot dog": [
                "a dog",
                # "a quadruped with four mechanical grey colored legs",             
                "a dog like robot",
                "a robot resembling a dog",
                "a robot resembling a sitting dog",
                "close up view Go2",
                "monochrome gray dog",
                "a gray dog with orange plate on the back",
                # "a gray  that looks like a mechanical dog",
                "a sitting quadruped",
                "a quadruped"
                "Unitree Go2 robot dog",
                # "a futuristic robotic animal",
                "a robot shaped like a canine",
                "a silver-gray robot with a rounded head and no wheels"
            ],

            "closeup view of Go2": [
                # "a gray thing",
                "alphabets",
                "2",
                "white sticker",
                "the G"
            ],
        }

        # Flatten the synonyms into a single list and build a mapping from query index to canonical label.
        self.all_queries = []
        self.query_to_group = {}
        for group_label, synonyms in self.synonym_groups.items():
            for s in synonyms:
                self.query_to_group[len(self.all_queries)] = group_label
                self.all_queries.append(s)

        self.get_logger().info(f"All queries: {self.all_queries}")
        self.get_logger().info(f"Query-to-group mapping: {self.query_to_group}")

        # -------------------------------
        # 2. Load the OwlViT Model and Processor (GPU Usage)
        # -------------------------------
        self.get_logger().info("Selecting device...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # self.device = "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        self.get_logger().info("Loading OwlViT model, please wait...")
        self.processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
        self.model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32").to(self.device)
        self.get_logger().info("Model loaded.")

        # -------------------------------
        # 3. Define Color Mapping per Category
        # -------------------------------
        self.color_mapping = {
            "robot dog": "blue",
            "turtlebot3": "red",
            # "a green object": "green",
            # "arm mounted robot": "yellow",
            "bottle": "orange",
            "chair": "pink",
            "table": "brown",
            "box": "yellow",
            "human" : "green"

        }

        # (Optional) Load a TrueType font for drawing (fallback to default if not available)
        try:
            self.font = ImageFont.truetype("arial.ttf", 15)
        except Exception:
            self.font = ImageFont.load_default()

        # Start a separate thread for OpenCV window display
        threading.Thread(target=self.display_image, daemon=True).start()

    def simple_nms(self, detections, iou_threshold=0.2):
        """
        A simple Non-Maximum Suppression (NMS) function that keeps detections with high scores and
        removes overlapping bounding boxes.
        :param detections: list of tuples (score, [x_min, y_min, x_max, y_max])
        :param iou_threshold: IoU threshold for suppression
        :return: filtered list of detections
        """
        # Sort detections by descending score.
        detections = sorted(detections, key=lambda x: x[0], reverse=True)
        keep = []

        def iou(boxA, boxB):
            xA = max(boxA[0], boxB[0])
            yA = max(boxA[1], boxB[1])
            xB = min(boxA[2], boxB[2])
            yB = min(boxA[3], boxB[3])
            interArea = max(0, xB - xA) * max(0, yB - yA)
            boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
            boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
            unionArea = boxAArea + boxBArea - interArea
            return interArea / unionArea if unionArea > 0 else 0

        while detections:
            best = detections.pop(0)
            keep.append(best)
            detections = [det for det in detections if iou(best[1], det[1]) < iou_threshold]
        return keep

    def image_callback(self, msg):
        # Increase frame counter.
        self.frame_counter += 1

        # Convert the ROS2 image message to a CV2 image (BGR).
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # If this is not the 5th frame, skip processing to save computational resources.
        if self.frame_counter % 10 != 0:
            return

        # Convert the CV2 image (BGR) to RGB and then to a PIL Image.
        frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)
        draw = ImageDraw.Draw(pil_image)

        # -------------------------------
        # Run OwlViT with All Queries (Synonyms)
        # -------------------------------
        inputs = self.processor(text=self.all_queries, images=pil_image, return_tensors="pt")
        # Move inputs to GPU
        inputs = {key: val.to(self.device) for key, val in inputs.items()}

        with torch.no_grad():
            outputs = self.model(**inputs)

        # Use a very low threshold (0.0) to retrieve all candidate detections.
        target_sizes = torch.tensor([pil_image.size[::-1]]).to(self.device)  # Note: PIL size is (width, height)
        results = self.processor.post_process_object_detection(
            outputs, threshold=0.0, target_sizes=target_sizes)[0]

        scores = results["scores"].cpu()
        boxes = results["boxes"].cpu()
        labels = results["labels"].cpu()

        # -------------------------------
        # Group Detections by Canonical Label
        # -------------------------------
        grouped_detections = {}
        for score, box, label in zip(scores, boxes, labels):
            score_val = score.item()
            box_val = box.tolist()
            label_int = label.item()
            if label_int not in self.query_to_group:
                # If there's an unknown label, skip it.
                continue
            group_label = self.query_to_group[label_int]
            if group_label not in grouped_detections:
                grouped_detections[group_label] = []
            grouped_detections[group_label].append((score_val, box_val))

        # -------------------------------
        # (Optional) Filter and Merge Detections per Group
        # -------------------------------
        final_detections = {}
        for group_label, det_list in grouped_detections.items():
            custom_threshold = 0.2  # Adjust per group if necessary.
            filtered = [det for det in det_list if det[0] >= custom_threshold]
            after_nms = self.simple_nms(filtered, iou_threshold=0.5)
            final_detections[group_label] = after_nms

        # -------------------------------
        # Create and Populate Detection2DArray message
        # -------------------------------
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = Header()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = "camera_frame"  # Replace with your camera frame

        for group_label, det_list in final_detections.items():
            color = self.color_mapping.get(group_label, "red")
            for score_val, box_val in det_list:
                x_min, y_min, x_max, y_max = [int(b) for b in box_val]

                # Create Detection2D message for each detection
                detection_msg = Detection2D()
                detection_msg.header = detection_array_msg.header

                # Use Point2D instead of Pose2D
                detection_msg.bbox.center.position.x = (x_min + x_max) / 2.0
                detection_msg.bbox.center.position.y = (y_min + y_max) / 2.0
                detection_msg.bbox.size_x = float(x_max - x_min)
                detection_msg.bbox.size_y = float(y_max - y_min)

                # Add object hypothesis (label and score)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = group_label
                hypothesis.hypothesis.score = float(score_val)
                detection_msg.results.append(hypothesis)

                # Append the detection message to the array
                detection_array_msg.detections.append(detection_msg)
                
                # Draw bounding boxes and labels on the image
                draw.rectangle([x_min, y_min, x_max, y_max], outline=color, width=3)
                text = f"{group_label} ({score_val:.2f})"
                draw.text((x_min, max(0, y_min - 15)), text, fill=color, font=self.font)
                
        # Publish the Detection2DArray message
        self.detection_publisher.publish(detection_array_msg)

        # -------------------------------
        # Display & Publish the Processed Frame
        # -------------------------------
        processed_frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

        # Publish compressed image
        _, buffer = cv2.imencode('.jpg', processed_frame)
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.format = "jpeg"
        compressed_image.data = np.array(buffer).tobytes()
        self.img_publisher.publish(compressed_image)

        # Store the processed frame for display
        with self.lock:
            self.processed_frame = processed_frame

    def display_image(self):
        cv2.namedWindow("Drone OwlViT", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                if self.processed_frame is not None:
                    cv2.imshow("Drone OwlViT", self.processed_frame)
                    cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = OwlViTDetectionNode()
    try:
        # Use spin_once to allow OpenCV to update
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OwlViTDetectionNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
