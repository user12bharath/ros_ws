#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2 as cv
import os

class YoloDetectorPy(Node):
    def __init__(self):
        super().__init__('yolo_detector_py')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # uses PyTorch directly, no OpenCV DNN

        self.declare_parameter('target_classes', ['person'])
        self.declare_parameter('confidence_threshold', 0.5)
        self.target_classes = self.get_parameter('target_classes').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_count = self.create_publisher(Int32, '/vision/sim_face_count', 10)
        self.pub_alert = self.create_publisher(String, '/vision/sim_alert', 10)

        self.get_logger().info(
            f'YoloDetectorPy started — target classes: {self.target_classes}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=self.conf_threshold, verbose=False)

        detected = False
        for r in results:
            for box in r.boxes:
                cls_name = self.model.names[int(box.cls)]
                if cls_name in self.target_classes:
                    detected = True
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv.putText(frame, f'{cls_name} {float(box.conf):.2f}',
                               (x1, max(y1 - 10, 0)),
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        alert = String()
        alert.data = 'TARGET DETECTED' if detected else 'CLEAR'
        self.pub_alert.publish(alert)

        count = Int32()
        count.data = 1 if detected else 0
        self.pub_count.publish(count)

        cv.imshow('YOLO view', frame)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorPy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()