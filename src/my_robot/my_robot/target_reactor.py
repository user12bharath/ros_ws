import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

class TargetReactor(Node):
    def __init__(self):
        super().__init__('target_reactor')

        self.target_visible = False
        self.front_distance = float('inf')

        self.subscriber_1 = self.create_subscription(String, '/vision/sim_alert', self.alert_callback, 10)
        self.subscriber_2 = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.stop_distance = 1.0
        self.slow_distance = 2.0

        self.timer = self.create_timer(0.1, self.decide_action)

    def alert_callback(self, msg):
        if msg.data == 'TARGET DETECTED':
            self.target_visible = True
        else:
            self.target_visible = False
        pass

    def scan_callback(self, msg):
        ranges = msg.ranges
        max_range = msg.max_range
        clean_ranges = [r if (not math.isinf(r) and not math.isnan(r)) else max_range for r in ranges]

        front_ranges = clean_ranges[0:30] + clean_ranges[330:360]
        self.front_distance = min(front_ranges)

    def decide_action(self):
        twist = Twist()

        if not self.target_visible:
            twist.linear.x = 0.002
            pass
        elif self.front_distance > self.slow_distance:
            twist.linear.x = 0.002
            pass
        elif self.front_distance > self.stop_distance:
            friction = (self.front_distance - self.stop_distance) / (self.slow_distance - self.stop_distance)
            twist.linear.y= 0.2 * friction
            pass
        else:
            twist.linear.x = 0.0
            pass

        self.publisher.publish(twist)
        self.get_logger().info(f'current distance: {self.front_distance:.2f}, target_visible: {self.target_visible}, cmd_vel: {twist.linear.x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetReactor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()