#! /usr/bin/env python3

"""
        Description:
        This ROS 2 node subscribes to a topic and logs the received messages.

    ---------
    Publishing Topics:
        None

    --------
    Subscription Topics:
        /py_example_topic - std_msgs/String

    
    ----
    Author: Pavan
        Date: 2024-06-01

    """

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPySubscriber(Node):
    def __init__(self):
        super().__init__('minimal_py_subscriber')

        self.subscriber_1 = self.create_subscription(
            String,
            'py_example_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    minimal_py_subscriber = MinimalPySubscriber()

    rclpy.spin(minimal_py_subscriber)

    minimal_py_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

