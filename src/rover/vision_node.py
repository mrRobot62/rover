import rclpy
from rclpy.node import Node

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('VisionNode gestartet')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
