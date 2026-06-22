import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, String
from ament_index_python.packages import get_package_share_directory  
import time
import os
import cv2 as cv
import threading

class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector')

        self.publisher_1 = self.create_publisher(Int32, '/vision/face_count', 10)
        self.publisher_2 = self.create_publisher(Float64, '/vision/fps', 10)
        self.publisher_3 = self.create_publisher(String, '/vision/alerts', 10)

        #open webcam using opencv
        self.cap = cv.VideoCapture(0)
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        #capture thread runs independently
        self.capture_thread = threading.Thread(
            target=self.capture_loop, daemon=True)
        self.capture_thread.start()



        #load haar cascade from 'haarcascade_frontalface_default.xml'
        cascade_path = os.path.join(
            get_package_share_directory('vision_node'),
            'data',
            'haarcascade_frontalface_default.xml'
        )
        self.face_cascade = cv.CascadeClassifier(cascade_path)

        if self.face_cascade.empty():
            self.get_logger().error(
                f'Failed to load cascade from: {cascade_path}')
            raise RuntimeError('Cascade classifier failed to load. Check data_files in setup.py.')


        self.prev_time = time.time()

        # create timer at 30Hz (period = 1/30)
        self.timer =  self.create_timer(1/30, self.process_frame)

        self.get_logger().info('FaceDetector node started')

    def capture_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame

    def process_frame(self):
        #capture a frame from the webcam
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
        
        current_time = time.time()
        fps = 1.0/(current_time - self.prev_time)
        self.prev_time = current_time

        #convert the frame to grayscale for face detection
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #detect faces in the grayscale frame using detectMultiscale
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30,30))

        #draw rectangles around detected faces on the original frame
        for (x,y,w,h) in faces:
            cv.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 255), 2)
            #put fps text on the frame using cv.putText
        cv.putText(frame, 'FPS: %.2f' % fps, (10,30), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv.putText(frame, 'Faces: %d' % len(faces), (10,70), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        #face count (len of face array) as Int32
        face_count_msg = Int32()
        face_count_msg.data = len(faces)
        self.publisher_1.publish(face_count_msg)
            
        #publish fps as Float64
        fps_msg = Float64()
        fps_msg.data = fps
        self.publisher_2.publish(fps_msg)

        face_detection_alert_msg = String()
        if len(faces) > 0:
            face_detection_alert_msg.data = 'FACE DETECTED'
        else:
            face_detection_alert_msg.data = 'NO FACES DETECTED'
        self.publisher_3.publish(face_detection_alert_msg)

        cv.imshow('face detection', frame)

        cv.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()