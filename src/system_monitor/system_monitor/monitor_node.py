import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import psutil

class SystemMonitor(Node):

    def __init__(self):
        super().__init__('system_monitor')

        self.cpu_publisher = self.create_publisher(Float64, 'system/cpu', 10)
        self.ram_publisher = self.create_publisher(Float64, 'system/ram', 10)

        psutil.cpu_percent(interval=None)

        self.timer = self.create_timer(1.0, self.publish_stats)
        self.get_logger().info('SystemMonitor node started')

    def publish_stats(self):

        cpu_usage = psutil.cpu_percent(interval=None)

        ram_usage = psutil.virtual_memory().percent

        cpu_msg = Float64()
        cpu_msg.data = cpu_usage
        self.cpu_publisher.publish(cpu_msg)

        ram_msg = Float64()
        ram_msg.data = ram_usage
        self.ram_publisher.publish(ram_msg)

        self.get_logger().info(f'CPU: {cpu_usage:.2f}%, RAM: {ram_usage:.2f}%')

def main(args=None):
    rclpy.init(args=args)

    node = SystemMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
