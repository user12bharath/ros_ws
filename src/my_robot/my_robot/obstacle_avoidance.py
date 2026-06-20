import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.publisher_1 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_1 = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.safe_distance = 0.5
        self.get_logger().info('Obstacle Avoidance Node has been started.')

    def scan_callback(self, msg):
        ranges = msg.ranges

        max_range = msg.range_max

        clean_ranges = [r if (not math.isinf(r) and not math.isnan(r)) else max_range for r in ranges]
        front_ranges = clean_ranges[0:30] + clean_ranges[330:360]
        min_front_distance = min(front_ranges)


        Twist_msg = Twist()
        if min_front_distance < self.safe_distance:
            Twist_msg.linear.x = 0.0
            Twist_msg.angular.z = 0.5
        else:
            Twist_msg.linear.x = 0.2
            Twist_msg.angular.z = 0.0


        self.publisher_1.publish(Twist_msg)

        self.get_logger().info(f'MIN FRONT DISTANCE: {min_front_distance:.2f}m, CMD_VEL: linear.x={Twist_msg.linear.x:.2f}, angular.z={Twist_msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()