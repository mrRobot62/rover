import rclpy
from rclpy.node import Node
from .control.led_pattern import LEDPattern
from .hardware.ws2812_driver import WS2812
from rover_interfaces.msg import LEDMessage
# -------------------------------------------------------------------------------------------------------------------------------
class LEDPatternConfig:
    def __init__(self, pattern_id: int, pattern_name=str, led_type="WS2812", duration: int=0, timeout: int=0, callback=None, callback_param=None):
        """ 
        LEDPattern.
        @param pattern_id entspricht der ID aus der Enumeration
        @param pattern_name Name des Musters
        @param duration default=0 (für das Pattern irrelevant), wenn > 0 wird diese Zeit in ms als Pause genutzt für das nachfolgende Pattern (zB BLINK_xxx)#
        @param timeout default=0 (für das Pattern irrelevant), wenn > 0 die Zeit in ms, bis das Pattern deaktiviert (LED OFF) gesetzt wird
        """
        self.pattern_id = pattern_id
        self.pattern_name = pattern_name
        self.led_type = led_type
        self.duration = duration
        self.timeout = timeout
        self.callback = callback
        self.callback_param = callback_param
        if self.led_type == "WS2812":
            self.led = WS2812()


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
        self.pattern = {
            LEDPattern.RED: LEDPatternConfig(LEDPattern.RED, 
                "FILL RED", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={"color":(255, 0, 0),"timeout":led_default_timeout, "duration_on": led_default_duration_on, "duration_off": led_default_duration_off, "brightness": led_brightness}
            ),
            LEDPattern.GREEN: LEDPatternConfig(LEDPattern.GREEN, 
                "FILL GREEN", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={"color":(0, 255, 0),"timeout":led_default_timeout, "duration_on": led_default_duration_on, "duration_off": led_default_duration_off, "brightness": led_brightness}
            ),
            LEDPattern.BLUE: LEDPatternConfig(LEDPattern.BLUE, 
                "FILL BLUE", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={"color":(0, 0, 255),"timeout":led_default_timeout, "duration_on": led_default_duration_on, "duration_off": led_default_duration_off, "brightness": led_brightness}
            ),
            LEDPattern.ORANGE: LEDPatternConfig(LEDPattern.ORANGE, 
                "FILL ORANGE", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={"color":(255, 165, 0),"timeout":led_default_timeout, "duration_on": led_default_duration_on, "duration_off": led_default_duration_off, "brightness": led_brightness}
            ),
            LEDPattern.WHITE: LEDPatternConfig(LEDPattern.WHITE, 
                "FILL WHITE", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={"color":(255, 255, 255),"timeout":led_default_timeout, "duration_on": led_default_duration_on, "duration_off": led_default_duration_off, "brightness": led_brightness}
            ),            
            LEDPattern.BLINK_LEFT: LEDPatternConfig(LEDPattern.BLINK_LEFT, 
                "BLINK LEFT", 
                duration=0, 
                timeout=0, 
                callback='blink', 
                callback_param={"direction":LEDPattern.BLINK_LEFT, "duration_on":500, "duration_off":500, "timeout":0}
            ),             
            LEDPattern.BLINK_RIGHT: LEDPatternConfig(LEDPattern.BLINK_RIGHT,
                "BLINK RIGHT", 
                duration=0, 
                timeout=0, 
                callback='blink', 
                callback_param={"direction":LEDPattern.BLINK_RIGHT, "duration_on":500, "duration_off":500, "timeout":0}
            ),             
            LEDPattern.HAZARD_LIGHT: LEDPatternConfig(LEDPattern.HAZARD_LIGHT, 
                "BLINK LEFT", 
                duration=0, 
                timeout=0, 
                callback='blink', 
                callback_param={"direction":LEDPattern.HAZARD_LIGHT, "duration_on":1000, "duration_off":500, "timeout":0}
            ),             
        }

        self.subscription = self.create_subscription(
            LEDMessage,
            led_topic,
            self.led_callback,
            10
        )

        self.get_logger().info('LEDNode gestartet')

    def led_callback(self, msg):
        pattern_id = LEDPattern(msg.pattern)  # kommt vom Publisher
        config = self.pattern.get(pattern_id)

        if not config:
            self.get_logger().warn(f"Unbekanntes Pattern: {msg.pattern}")
            return

        #
        # Python-Trick
        # getattr liest aus, welche Attribute ein objekt besitzt. In unserem Fall benötigen wir den Inhalt
        # des Callback-Attributes.
        # Hier steht der Methodenname drin, der aufgerufen werden soll.
        method = getattr(config.led, config.callback, None)
        #
        if method is None:
            self.get_logger().error(f"Methode {config.callback} nicht gefunden")
            return
        self.get_logger().info(f"Aktiviere Pattern {config.pattern_name} → {config.callback} mit Parametern {config.callback_param}")

        # das ist der eigentliche Methoden-Aufruf mit übergabe der Parameter.
        method(**config.callback_param)


def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
