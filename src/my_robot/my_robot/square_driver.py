import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        self.publisher_1 = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        time.sleep(5.0)
        self.get_logger().info('SquareDriver started - driving square pattern')

        #call self.drive_square() to start driving the square pattern
        self.drive_square()

    def make_twist(self, linear_x=0.0, angular_z=0.0):
        #create a TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.angular.z = angular_z
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        return twist_msg
    
    def drive_square(self):
        for side in range(4):
            self.get_logger().info(f'Side {side + 1} of 4')

            # YOUR CODE: publish forward velocity (linear.x = 0.2) for 3 seconds
            # use a loop: publish → sleep 0.1s → repeat for 3 seconds total
            start_time = time.time()
            while time.time() - start_time < 3.0:
                self.publisher_1.publish(self.make_twist(linear_x=0.2))
                time.sleep(0.1)

            self.publisher_1.publish(self.make_twist(linear_x=0.0))  # stop before turning
            time.sleep(0.5)  # pause before turning

            start_time = time.time()
            while time.time() - start_time < 2.8:
                self.publisher_1.publish(self.make_twist(angular_z=0.5))
                time.sleep(0.1)

            self.publisher_1.publish(self.make_twist(angular_z=0.0))  # stop after turning
            time.sleep(0.5)  # pause after turning

        self.get_logger().info('Square completed')

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()