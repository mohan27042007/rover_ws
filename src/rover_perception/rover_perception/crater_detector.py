#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
import os

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

class CraterDetector(Node):
    def __init__(self):
        super().__init__('crater_detector')
        self.declare_parameter('model_path', 'models/crater_model.pt')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.model_path = self.get_parameter('model_path').value
        self.bridge = CvBridge()

        if YOLO is None:
            self.get_logger().error("ultralytics not available. Install ultralytics in your env.")
            raise RuntimeError("ultralytics required")

        self.model = YOLO(self.model_path)

        self.pub_det = self.create_publisher(Detection2DArray, '/perception/crater_detections', 10)
        self.pub_img = self.create_publisher(Image, '/perception/crater_annotated', 10)
        self.sub = self.create_subscription(Image, self.get_parameter('camera_topic').value, self.image_cb, 10)
        self.get_logger().info(f"Crater detector loaded model: {self.model_path}")

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_img, conf=0.25)
        dets_msg = Detection2DArray()
        dets_msg.header = msg.header
        annotated = cv_img.copy()

        if len(results) > 0:
            res = results[0]
            if hasattr(res, 'boxes') and len(res.boxes) > 0:
                boxes = res.boxes.xyxy.cpu().numpy()
                scores = res.boxes.conf.cpu().numpy()
                classes = res.boxes.cls.cpu().numpy().astype(int)
                for (x1, y1, x2, y2), score, cls in zip(boxes, scores, classes):
                    bb = BoundingBox2D()
                    # center is Pose2D -> use position.x / position.y
                    bb.center.position.x = float((x1 + x2) / 2.0)
                    bb.center.position.y = float((y1 + y2) / 2.0)
                    bb.size_x = float(x2 - x1)
                    bb.size_y = float(y2 - y1)

                    det = Detection2D()
                    det.header = msg.header
                    det.bbox = bb

                    hypo = ObjectHypothesisWithPose()
                    hypo.hypothesis.class_id = "crater"
                    hypo.hypothesis.score = float(score)
                    det.results.append(hypo)

                    dets_msg.detections.append(det)

                    cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    label = f"crater {score:.2f}"
                    cv2.putText(annotated, label, (int(x1), int(max(y1 - 6, 0))),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

        self.pub_det.publish(dets_msg)
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.pub_img.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CraterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
