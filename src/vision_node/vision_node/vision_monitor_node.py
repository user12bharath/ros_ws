import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, String

class VisionMonitor(Node):

    def __init__(self):
        super().__init__('vision_monitor')

        #initialize self.latest_fps, self.latest_face_count,elf.latest_alert with sensible defaults
        self.latest_fps = 0.0
        self.latest_face_count = 0
        self.latest_alert = 'No alerts'


        self.subscriber_1 = self.create_subscription(Int32, '/vision/face_count', self.face_count_callback, 10)
        self.subscriber_2 = self.create_subscription(Float64, '/vision/fps', self.fps_callback, 10)
        self.subscriber_3 =self.create_subscription(String, '/vision/alerts', self.alert_callback, 10)

        self.timer = self.create_timer(1.0, self.print_status)

    def fps_callback(self, msg):
        self.latest_fps = msg.data

    def face_count_callback(self, msg):
        self.latest_face_count = msg.data

    def alert_callback(self, msg):
        self.latest_alert = msg.data

    def print_status(self):
        self.get_logger().info(f' [VISION STATUS] FPS: {self.latest_fps:.2f} | Faces: {self.latest_face_count} | Status: {self.latest_alert}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
