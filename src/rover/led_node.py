import rclpy
from rclpy.node import Node
from .control.led_pattern import *

class LEDNode(Node):
    def __init__(self):
        super().__init__('led_node')

        # Parameter auslesen
        self.declare_parameters(
        namespace='',
        parameters=[
            ('led_topic', '/led_default'),
            ('led_num_pixels', 24),
            ('led_type', 'WS2812'),
            ('led_brightness', 0.3),
            ('led_default_timeout', 1000),
            ('led_default_duration_on', 500),
            ('led_default_duration_off', 500),
        ])
        led_topic = self.get_parameter('led_topic').get_parameter_value().string_value
        led_num_pixel = self.get_parameter('led_num_pixels').get_parameter_value().integer_value
        led_type = self.get_parameter('led_type').get_parameter_value().string_value
        led_brightness = self.get_parameter('led_brightness').get_parameter_value().double_value
        led_default_timeout = self.get_parameter('led_default_timeout').get_parameter_value().integer_value
        led_default_duration_on = self.get_parameter('led_default_duration_on').get_parameter_value().integer_value
        led_default_duration_off = self.get_parameter('led_default_duration_off').get_parameter_value().integer_value

        self.get_logger().info(
f"""
LEDNode config:\n\
--------------------------------
Topic:          {led_topic},
LEDType:        {led_type},
Pixels:         {led_num_pixel},
Brightness:     {led_brightness},
Timeout:        {led_default_timeout},
DurationON:     {led_default_duration_on},
DurationOFF:    {led_default_duration_off},
""")

        self.get_logger().info('LEDNode gestartet')



def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
