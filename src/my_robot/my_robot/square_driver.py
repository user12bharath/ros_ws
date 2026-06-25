import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        
        # Sync with Gazebo simulation clock
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_out', 10)
        
        # 10Hz loop frequency (0.1 seconds per step)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.counter = 0
        self.side = 0
        
        # States: 
        # 0: Setup-Forward(3m) | 1: Setup-Turn | 2: Setup-Forward(3m) | 3: Setup-Turn
        # 4: Loop-Forward(6m)  | 5: Loop-Turn  | 6: Finished
        self.state = 0 
        self.get_logger().info('Starting outer perimeter mapping routing...')

    def make_twist(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def timer_callback(self):
        # PHASE 1: NAVIGATING TO THE PERIMETER
        if self.state == 0:  # Move 3.0m out from center
            if self.counter < 150:
                self.publisher_.publish(self.make_twist(linear_x=0.2))
                self.counter += 1
            else:
                self.state, self.counter = 1, 0

        elif self.state == 1:  # Turn 90 deg
            if self.counter < 39:
                self.publisher_.publish(self.make_twist(angular_z=0.4))
                self.counter += 1
            else:
                self.state, self.counter = 2, 0

        elif self.state == 2:  # Move 3.0m to reach the first corner link
            if self.counter < 150:
                self.publisher_.publish(self.make_twist(linear_x=0.2))
                self.counter += 1
            else:
                self.state, self.counter = 3, 0

        elif self.state == 3:  # Turn 90 deg to align with wall perimeter
            if self.counter < 39:
                self.publisher_.publish(self.make_twist(angular_z=0.4))
                self.counter += 1
            else:
                self.state, self.counter = 4, 0
                self.get_logger().info('Perimeter reached. Beginning 6m x 6m mapping loop.')

        # PHASE 2: RUNNING THE PERIMETER SQUARE
        elif self.state == 4:  # Drive 6.0m straight parallel to wall (2m clearance)
            if self.counter < 300:
                self.publisher_.publish(self.make_twist(linear_x=0.2))
                self.counter += 1
            else:
                self.state, self.counter = 5, 0

        elif self.state == 5:  # Turn 90 deg at corner
            if self.counter < 39:
                self.publisher_.publish(self.make_twist(angular_z=0.4))
                self.counter += 1
            else:
                self.side += 1
                self.counter = 0
                if self.side < 4:
                    self.state = 4  # Next side
                else:
                    self.state = 6  # Loop complete
                    self.get_logger().info('Map complete! Standing still for Map Saver.')

        elif self.state == 6:  # Final stop
            self.publisher_.publish(self.make_twist(linear_x=0.0, angular_z=0.0))

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
