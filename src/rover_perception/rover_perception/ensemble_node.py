#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torchvision.ops as ops
import numpy as np
import threading
import cv2

class EnsembleNode(Node):
    def __init__(self):
        super().__init__('perception_ensemble')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.crater_dets = None
        self.boulder_dets = None
        self.latest_image = None

        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.img_cb, 10)
        self.sub_crater = self.create_subscription(Detection2DArray, '/perception/crater_detections', self.crater_cb, 10)
        self.sub_boulder = self.create_subscription(Detection2DArray, '/perception/boulder_detections', self.boulder_cb, 10)

        self.pub_dets = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.pub_img = self.create_publisher(Image, '/perception/annotated_image', 10)

        self.get_logger().info("Ensemble node ready.")

    def img_cb(self, msg: Image):
        with self.lock:
            self.latest_image = msg
        self.try_publish()

    def crater_cb(self, msg: Detection2DArray):
        with self.lock:
            self.crater_dets = msg
        self.try_publish()

    def boulder_cb(self, msg: Detection2DArray):
        with self.lock:
            self.boulder_dets = msg
        self.try_publish()

    def try_publish(self):
        with self.lock:
            if self.latest_image is None:
                return
            # Accept even if one detector is missing — merge what's available
            all_boxes = []
            all_scores = []
            all_labels = []

            # helper to convert Detection2D -> box coords
            def append_from(msg, label_name):
                if msg is None:
                    return
                for d in msg.detections:
                    # bounding box center/size in pixels stored earlier
                    # BoundingBox2D.center is a Pose2D (position.x, position.y)
                    cx = d.bbox.center.position.x
                    cy = d.bbox.center.position.y
                    sx = d.bbox.size_x
                    sy = d.bbox.size_y
                    if sx <= 0 or sy <= 0:
                        continue
                    x1 = cx - sx/2.0
                    y1 = cy - sy/2.0
                    x2 = cx + sx/2.0
                    y2 = cy + sy/2.0
                    # best score from results (first result)
                    score = 0.0
                    if len(d.results) > 0:
                        score = d.results[0].hypothesis.score
                    all_boxes.append([x1, y1, x2, y2])
                    all_scores.append(float(score))
                    all_labels.append(label_name)

            append_from(self.crater_dets, "crater")
            append_from(self.boulder_dets, "boulder")

            if len(all_boxes) == 0:
                # publish empty
                out = Detection2DArray()
                out.header = self.latest_image.header
                self.pub_dets.publish(out)
                # annotated image = raw image
                self.pub_img.publish(self.latest_image)
                return

            boxes = torch.tensor(all_boxes, dtype=torch.float32)
            scores = torch.tensor(all_scores, dtype=torch.float32)
            labels = np.array(all_labels)

            # Run NMS class-wise
            keep_indices = []
            unique_labels = np.unique(labels)
            for lab in unique_labels:
                idxs = np.where(labels == lab)[0]
                if idxs.size == 0:
                    continue
                sub_boxes = boxes[idxs]
                sub_scores = scores[idxs]
                keep = ops.nms(sub_boxes, sub_scores, iou=0.5)
                for k in keep.cpu().numpy().tolist():
                    keep_indices.append(int(idxs[k]))

            # Build output Detection2DArray
            out = Detection2DArray()
            out.header = self.latest_image.header

            # sort keep indices so deterministic
            for idx in sorted(keep_indices):
                x1, y1, x2, y2 = boxes[idx].tolist()
                score = float(scores[idx].item())
                label = labels[idx]

                d = Detection2D()
                d.header = self.latest_image.header
                # bbox center & size
                d.bbox.center.x = float((x1 + x2) / 2.0)
                d.bbox.center.y = float((y1 + y2) / 2.0)
                d.bbox.size_x = float(x2 - x1)
                d.bbox.size_y = float(y2 - y1)

                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = str(label)
                hypo.hypothesis.score = score
                d.results.append(hypo)
                out.detections.append(d)

            self.pub_dets.publish(out)

            # Annotate image
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            for d in out.detections:
                cx = d.bbox.center.position.x; cy = d.bbox.center.position.y
                sx = d.bbox.size_x; sy = d.bbox.size_y
                x1 = int(cx - sx/2.0); y1 = int(cy - sy/2.0); x2 = int(cx + sx/2.0); y2 = int(cy + sy/2.0)
                label = d.results[0].hypothesis.class_id
                score = d.results[0].hypothesis.score
                color = (0, 0, 255) if label == 'boulder' else (255, 0, 0)
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(cv_img, f"{label} {score:.2f}", (x1, max(y1-6,0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            annotated_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            annotated_msg.header = self.latest_image.header
            self.pub_img.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnsembleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
