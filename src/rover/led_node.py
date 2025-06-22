import rclpy
from rclpy.node import Node
from .control.led_pattern import *

class LEDNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.get_logger().info('LEDNode gestartet')

        # Parameter auslesen
        self.declare_parameter('led_topic', "/leddddd")
        self.declare_parameter('led_num_pixel', 24)
        self.declare_parameter('led_type', "WS12345")

        led_topic = self.get_parameter('led_topic').get_parameter_value().string_value
        led_num_pixel = self.get_parameter('led_num_pixel').get_parameter_value().integer_value
        led_type = self.get_parameter('led_type').get_parameter_value().string_value

        self.get_logger().info(f"LEDNode CFG: Topic:{led_topic}, LEDType:{led_type}, Pixels:{led_num_pixel}")

def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
