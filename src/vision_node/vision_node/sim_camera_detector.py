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
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5)
        face_count = len(faces)

        # draw bounding boxes on the frame
        for (x, y, w, h) in faces:
            cv.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # publish the face count with alert 
        face_count_msg = Int32()
        face_count_msg.data = face_count
        self.publisher_1.publish(face_count_msg)
        alert_msg = String()
        if face_count > 0:
            alert_msg.data = 'Face detected!'
        else:
            alert_msg.data = 'No face detected.'
        self.publisher_2.publish(alert_msg)

        #display the frames that robot sees with detection drawing
        cv.imshow('Sim Camera View', cv_image)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimCameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()