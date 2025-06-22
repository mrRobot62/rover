import rclpy
from rclpy.node import Node

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.get_logger().info('OdomNode gestartet')

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
