import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2 as cv
import os

class SimCameraDetector(Node):
    def __init__(self):

        super().__init__('sim_camera_detector')
        
        self.bridge = CvBridge()

        self.subscriber_1 = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.publisher_1 = self.create_publisher(Int32, '/vision/sim_face_count', 10)
        self.publisher_2 = self.create_publisher(String, '/vision/sim_alert', 10)

        cascade_path = os.path.join(
            get_package_share_directory('vision_node'),
            'data',
            'haarcascade_frontalface_default.xml'
        )

        if not os.path.exists(cascade_path):
            cascade_path = cv.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv.CascadeClassifier(cascade_path)

        self.get_logger().info('SimCameraDetector node has been started. subscribed to /camera/image_raw')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV: {e}")
            return
        
        HSV = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        lower_red1 = (0, 120, 70)
        upper_red1 = (10, 255, 255)
        lower_red2 = (170, 120, 70)
        upper_red2 = (180, 255, 255)

        mask1 = cv.inRange(HSV, lower_red1, upper_red1)
        mask2 = cv.inRange(HSV, lower_red2, upper_red2)
        mask = cv.bitwise_or(mask1, mask2)

        contour, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        detected = False
        largest_area = 0
        target_box = None

        for c in contour:
            area =cv.contourArea(c)
            if area > 300 and area > largest_area:
                largest_area = area
                target_box = cv.boundingRect(c)
                detected = True

        if target_box:
            x, y, w, h = target_box
            cv.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv.putText(cv_image, f'Target area={int(largest_area)}', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


        alert_msg = String()
        alert_msg.data = 'TARGET DETECTED' if detected else 'CLEAR'
        self.publisher_2.publish(alert_msg)
        
        count_msg = Int32()
        count_msg.data = 1 if detected else 0
        self.publisher_1.publish(count_msg)

        cv.imshow('sim camera view', cv_image)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimCameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()