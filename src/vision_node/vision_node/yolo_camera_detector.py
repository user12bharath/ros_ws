import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2 as cv
import torch
from ultralytics import YOLO

class YoloCameraDetector(Node):
    def __init__(self):
        super().__init__('yolo_camera_detector')

        self.bridge = CvBridge()

        self.declare_parameter('target_classes', ['person'])
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_filename', 'yolov8n.pt')

        self.target_classes = self.get_parameter('target_classes').value
        self.confidence_threshold =self.get_parameter('confidence_threshold').value
        self.model_filename = self.get_parameter('model_filename').value

        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading {self.model_filename} on device={self.device}...')
        self.model = YOLO(self.model_filename)
        self.get_logger().info(f'Model loaded. Watching for classes: {self.target_classes} with confidence threshold: {self.confidence_threshold}')

        self.subscriber_1 = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.publisher_1 = self.create_publisher(Int32, '/vision/sim_face_count', 10)
        self.publisher_2 = self.create_publisher(String, '/vision/sim_alert', 10)

        self.get_logger().info('Yolocameradetector node has been started. subscribed to /camera/image_raw')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting ROS Image to Opencv: {e}')
            return
        
        result = self.model.predict(cv_image, device=self.device, verbose=False, conf=self.confidence_threshold)

        detected = False
        largest_area = 0
        target_box = None

        boxes = result[0].boxes
        if boxes is not None:
            for box in boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                if class_name not in self.target_classes:
                    continue

                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                area = (x2 - x1) * (y2 - y1)


                if area > 300 and area > largest_area:
                    largest_area = area
                    target_box = (x1, y1, x2-x1, y2-y1)
                    detected = True

        if detected:
            x, y, w, h = target_box
            cv.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.putText(cv_image, f'Target area={int(largest_area)}', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
        alert_msg = String()
        alert_msg.data = 'TARGET DETECTED' if detected else 'CLEAR'
        self.publisher_2.publish(alert_msg)

        count_msg = Int32()
        count_msg.data = 1 if detected else 0
        self.publisher_1.publish(count_msg)

        self.get_logger().info(f'area of the detected target: {largest_area}, alert: {alert_msg.data}')

        cv.imshow('yolo camera view', cv_image)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()