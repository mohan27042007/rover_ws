#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

# Try to use ultralytics (preferred). If not available, node will error with clear message.
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

class BoulderDetector(Node):
    def __init__(self):
        super().__init__('boulder_detector')
        self.declare_parameter('model_path', 'models/boulder_model.pt')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.model_path = self.get_parameter('model_path').value
        self.bridge = CvBridge()

        if YOLO is None:
            self.get_logger().error("ultralytics not available. Install ultralytics in your env.")
            raise RuntimeError("ultralytics required")

        self.model = YOLO(self.model_path)

        self.pub_det = self.create_publisher(Detection2DArray, '/perception/boulder_detections', 10)
        self.pub_img = self.create_publisher(Image, '/perception/boulder_annotated', 10)
        self.sub = self.create_subscription(Image, self.get_parameter('camera_topic').value, self.image_cb, 10)
        self.get_logger().info(f"Boulder detector loaded model: {self.model_path}")

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # run inference (returns list-like)
        results = self.model(cv_img, conf=0.25)  # returns Results object
        if len(results) == 0:
            dets_msg = Detection2DArray()
            dets_msg.header = msg.header
            self.pub_det.publish(dets_msg)
            return

        res = results[0]
        dets_msg = Detection2DArray()
        dets_msg.header = msg.header

        # draw boxes on image for annotated output
        annotated = cv_img.copy()
        h, w = cv_img.shape[:2]

        if hasattr(res, 'boxes') and len(res.boxes) > 0:
            boxes = res.boxes.xyxy.cpu().numpy()  # N x 4 (x1,y1,x2,y2)
            scores = res.boxes.conf.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy().astype(int)

            for (x1, y1, x2, y2), score, cls in zip(boxes, scores, classes):
                # prepare Detection2D
                bb = BoundingBox2D()
                # center, size in pixels (Pose2D -> position.x/position.y)
                bb.center.position.x = float((x1 + x2) / 2.0)
                bb.center.position.y = float((y1 + y2) / 2.0)
                bb.size_x = float(x2 - x1)
                bb.size_y = float(y2 - y1)

                det = Detection2D()
                det.header = msg.header
                det.bbox = bb

                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = "boulder"
                hypo.hypothesis.score = float(score)
                det.results.append(hypo)

                dets_msg.detections.append(det)

                # draw box
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                label = f"boulder {score:.2f}"
                cv2.putText(annotated, label, (int(x1), int(max(y1 - 6, 0))),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        dets_msg.header = msg.header
        self.pub_det.publish(dets_msg)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.pub_img.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoulderDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
