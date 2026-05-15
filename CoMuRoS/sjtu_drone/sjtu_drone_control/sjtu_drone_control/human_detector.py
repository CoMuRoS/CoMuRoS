#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.models as models
import torchvision.transforms as transforms
import torch.nn.functional as F
from PIL import Image as PILImage

class NewHumanDetector(Node):
    def __init__(self):
        super().__init__('new_human_detector')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, '/simple_drone/bottom/image_raw', self.image_callback, 10)

        self.det_sub = self.create_subscription(
            Detection2DArray, '/simple_drone/owlvit_detections', self.detection_callback, 10)

        self.curr_image = None
        self.embeddings = []
        self.similarity_threshold = 0.85

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = models.resnet18(pretrained=True)
        self.model.fc = torch.nn.Identity()
        self.model.eval().to(self.device)

        self.transform = transforms.Compose([
            transforms.Resize((128, 128)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
        ])

        self.get_logger().info("New Human Detector is ready!")

    def image_callback(self, msg):
        self.curr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_callback(self, msg):
        if self.curr_image is None:
            return

        for detection in msg.detections:
            if not detection.results:
                continue

            label = detection.results[0].hypothesis.class_id
            if 'human' not in label.lower():
                continue

            x = int(detection.bbox.center.position.x - detection.bbox.size_x / 2)
            y = int(detection.bbox.center.position.y - detection.bbox.size_y / 2)
            w = int(detection.bbox.size_x)
            h = int(detection.bbox.size_y)
            crop = self.curr_image[y:y + h, x:x + w]

            if crop.size == 0:
                continue

            if self.is_new_human(crop):
                self.get_logger().info("🚨 New human detected!")
            else: 
                print("Trying")

    def is_new_human(self, crop_img):
        try:
            pil_img = PILImage.fromarray(cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB))
            tensor_img = self.transform(pil_img).unsqueeze(0).to(self.device)
            with torch.no_grad():
                embedding = self.model(tensor_img)
        except Exception as e:
            self.get_logger().warn(f"Error during embedding: {e}")
            return False

        for emb in self.embeddings:
            sim = F.cosine_similarity(embedding, emb).item()
            if sim > self.similarity_threshold:
                return False  # Already seen

        self.embeddings.append(embedding)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = NewHumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
