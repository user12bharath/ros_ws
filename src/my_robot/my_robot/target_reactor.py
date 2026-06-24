import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import math

class TargetReactor(Node):
    def __init__(self):
        super().__init__('target_reactor')

        self.target_visible = False
        self.front_distance = float('inf')

        self.subscriber_1 = self.create_subscription(String, '/vision/sim_alert', self.alert_callback, 10)
        self.subscriber_2 = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel_safety', 10)

        self.stop_distance = 1.4
        self.slow_distance = 1.8
        self.target_recently_seen = False
        self.lost_count = 0
        self.lost_threshold = 5

        self.timer = self.create_timer(0.1, self.decide_action)

    def alert_callback(self, msg):
        if msg.data == 'TARGET DETECTED':
            self.target_visible = True
            self.lost_count = 0

        else:
            self.lost_count += 1
            if self.lost_count >= self.lost_threshold:
                self.target_visible = False

    def scan_callback(self, msg):
        ranges = msg.ranges
        max_range = msg.range_max
        clean_ranges = [
            r if (not math.isinf(r) and not math.isnan(r)) else max_range
            for r in ranges
        ]

        front_ranges = clean_ranges[0:30] + clean_ranges[330:360]
        self.front_distance = min(front_ranges)

    def decide_action(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        if not self.target_visible:
            #no target seen — drive forward at normal speed (0.2)
            twist.twist.linear.x = 0.2
            pass
        elif self.front_distance > self.slow_distance:
            #target visible but far — normal forward speed
            twist.twist.linear.x = 0.2
        elif self.front_distance > self.stop_distance:
            #target visible and getting close — slow down
            # scale speed proportionally between slow_distance and stop_distance
            scale = (self.front_distance - self.stop_distance) / (self.slow_distance - self.stop_distance)
            twist.twist.linear.x = 0.2 * scale
        else:
            #target visible and within stop_distance — full stop (0.0)
            twist.twist.linear.x = 0.0
        self.publisher.publish(twist)
        #log current state — target_visible, front_distance, and chosen linear.x
        self.get_logger().info(f'Target visible: {self.target_visible}, Front distance: {self.front_distance:.2f}, Speed: {twist.twist.linear.x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetReactor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()