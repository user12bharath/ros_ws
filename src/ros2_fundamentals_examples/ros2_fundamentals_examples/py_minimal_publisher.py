#! /usr/bin/env python3

"""Description:
This ROS @ node periodically publishes "Hello World" messages to a topic.

---------
Publishing Topics:
  This channel containing the "Hello World" messages
  /py_example_topic - std_msgs/String

Subscription Topics:
None
--------
Author: Pavan
Date: 2024-06-01
"""

import rclpy #import the ROS2 @ client library for Python
from rclpy.node import Node #Import the node class, used for creating nodes

from std_msgs.msg import String # import string msg type for ros2

class MinimalPyPublisher(Node):
    """ Create a minimal publisher node.add()
    """

    def __init__(self):
        """ Creating a custom node class for publishing messages.
        """
        #initialize the node with a name
        super().__init__('minimal_py_publisher')

        #create a publisher on the topic with a queue size of 10 messages
        self.publisher_1 = self.create_publisher(String, 'py_example_topic', 10)


        #create a timer with a period of 0.5 second to trigger publisher of message
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #initialize a counter variable for message content
        self.i = 0

    def timer_callback(self):
        """callback function executed periodically by the timer"""

        #create a new String message object
        msg = String()
        
        #message data with a counter value
        msg.data = 'Hello World: %d' % self.i

        #publish the message to the topic
        self.publisher_1.publish(msg)

        #log a message indicating the message has been published
        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.i = self.i + 1 #increment the counter for the next message

def main(args=None):
    """main function to start the ros2 node

Args:
    args (List, optional): command-line arguments. defualt to none.
    """

    rclpy.init(args=args) #initialize the ROS2 @ client library

    #create an instance of the minimal publisher node
    minimal_py_publisher = MinimalPyPublisher()

    rclpy.spin(minimal_py_publisher)

    #destroy the node explicitly
    minimal_py_publisher.destroy_node()

    #shutdown the ros2 communication

    rclpy.shutdown()

if __name__ == '__main__':
    #execute the main function if the script is run directly
    main()