import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
import time  # am Anfang ergänzen
from rclpy.parameter import Parameter
#from .hardware.rover_driver import RoverDriver
from rover_interfaces.msg import I2CWrite
from rover_interfaces.msg import LEDMessage
from rover_interfaces.srv import I2CReadRequest
from .control.led_pattern import LEDPattern
from .control.ESP32Client import ESP32Client
from .control.ESP32CommandsV1 import CommandID, SubCommandID, ESP32PINS
from .control.utilities import Utilities

class ESP32_PORTS(Enum):
    LED1=18
    LED2=19

class BUTTONS(Enum):
    X=0
    A=1
    B=2
    Y=3
    LB=4
    RB=5
    LT=6
    RT=7
    BACK=8
    START=9

class JOYSTICKS(Enum):
    LJ_LR = 0      # Linker JoyStick links/rects
    LJ_UD = 1      # up/down
    RJ_LR = 2      # Rechter Joystick links/rechts
    RJ_UD = 3      # up/down
    PAD_LR = 4     # JoyPad
    PAD_UD = 5      # JoyPad

class DriverControllerNode(Node):
    """
    Der DriverControllNode steuert über den I2C_Node die Verbindung zum ESP indem er eine
    I2CWrite Topic-Nachricht generiert und published.

    Publish-Messagess: (OUT)
    - /i2c/I2CWrite: Message zu Steuerung des ESP32
    - /led : LEDMessage Anzeige von LEDPattern (z.B blinken, Warnblinker, ...)

    SUBSCRIBE-Messages: (IN)    
    - /joy : Daten des Gamepad-Controllers


    """


    def __init__(self):
        super().__init__('driver_controller_node')
        self.node_name = self.__class__.__name__
        # Parameter auslesen

        ct_tled = Utilities.get_common_topic('topic_led', 1, logger=self.get_logger())

        self.declare_parameters(
        namespace='',
        parameters=[
            ('cmd_vel_topic', '/joy'),
            ('topic_led', ct_tled),
            ('reverse_steering', True),
            ('reverse_velocity', False),
            ('map_js_steering', 0),
            ('map_js_velocity', 1),
            ('map_js_cam_turn', 2),
            ('map_js_cam_tilt', 3),
        ])

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.topic_led = self.get_parameter('topic_led').get_parameter_value().string_value
        self.reverse_steering = self.get_parameter('reverse_steering').get_parameter_value().bool_value
        self.reverse_velocity = self.get_parameter('reverse_velocity').get_parameter_value().bool_value
        self.map_js_steering = self.get_parameter('map_js_steering').get_parameter_value().integer_value
        self.map_js_velocity = self.get_parameter('map_js_velocity').get_parameter_value().integer_value
        self.map_js_cam_turn = self.get_parameter('map_js_cam_turn').get_parameter_value().integer_value
        self.map_js_cam_tilt = self.get_parameter('map_js_cam_tilt').get_parameter_value().integer_value

        self.get_logger().info(
        f"""
            DriverControllerNode(Node) config:\n\
            --------------------------------
            cmd_vel_topic:          {self.cmd_vel_topic}
            cmd_vel_topic:          {self.topic_led}
            reverse_steering:       {self.reverse_steering}
            reverse_velocity:       {self.reverse_velocity}
            map_js_steering:        {self.map_js_steering}
            map_js_velocity:        {self.map_js_velocity}
            map_js_cam_turn:        {self.map_js_cam_turn}
            map_js_cam_tilt:        {self.map_js_cam_tilt}
        """)
        
        self.get_logger().info(f"create publisher für I2CWrite")
        self.publisher = self.create_publisher(I2CWrite, '/i2c/write', 10)
        self.get_logger().info(f"create publisher für LEDMessages")
        self.led_pub = self.create_publisher(LEDMessage, '/led', 10)

        self.js_velocity = JOYSTICKS.LJ_UD.value
        self.js_steering = JOYSTICKS.LJ_LR.value

        #
        # teleop-Topic abonnieren
        self.cmd_vel_topic = self.cmd_vel_topic
        self.get_logger().info(f"create subscription für JoyStick-Commands")
        self.subscription = self.create_subscription(
            Joy,
            self.cmd_vel_topic,
            self.cmd_driver_callback,
            10
        )

        self.last_velocity = None
        self.last_steering = None
        self.last_axes = None
        self.last_buttons = None
        self.MODE = "MANUAL"
        self.toggle_active = False
        self.last_i2c_time = time.monotonic()
        self.min_interval = 0.05

        self.esp32client = ESP32Client(self, '/i2c/esp32_command')
        self.get_logger().info('ESP32Client Objekt erhalten')

        self.get_logger().info('DriverControllerNode gestartet.')

    def publish_led_pattern(self, pattern_id: int, timeout: int = 0, duration_on: int = 0, duration_off: int = 0, ledmask: int = 0x0fffffff):
        """
        """
        msg = LEDMessage()
        msg.pattern = pattern_id
        msg.ledtype = "WS2812"
        msg.timeout = timeout
        msg.duration = 0
        msg.duration_on = duration_on
        msg.duration_off = duration_off
        msg.brightness = 1.0
        msg.ledmask = ledmask
        self.led_pub.publish(msg)
        self.get_logger().info(f'LED Pattern {pattern_id} gesendet')

    def cmd_driver_callback(self, msg: Joy):
        axes = msg.axes.tolist()
        axes = [round(x, 3) for x in msg.axes]
        buttons = msg.buttons.tolist()

        #
        # für einfacheren Zugriff, buttons in Variablen ablegen
        x_button = buttons[BUTTONS.X.value]
        b_button = buttons[BUTTONS.B.value]
        lb_button = buttons[BUTTONS.LB.value]
        rb_button = buttons[BUTTONS.RB.value]

        if self.last_buttons is not None:

            #
            # Links blinken
            if lb_button and not self.last_buttons[BUTTONS.LB.value]:
                self.publish_led_pattern(
                    pattern_id=LEDPattern.BLINK_LEFT, 
                    duration_on=500, 
                    duration_off=500,
                    ledmask = 0b00011100001110
                )

            #
            # Rechts blinken
            if rb_button and not self.last_buttons[BUTTONS.RB.value]:
                self.publish_led_pattern(
                    pattern_id=LEDPattern.BLINK_RIGHT, 
                    duration_on=500, 
                    duration_off=500,
                    ledmask = 0b01110000111000
                )            #
            # Warnblink
            if rb_button and lb_button:
                self.publish_led_pattern(
                    pattern_id=LEDPattern.HAZARD_LIGHT,
                    duration_on=500, 
                    duration_off=500,
                    ledmask = 0xFFFFFFF
                )

        if x_button and b_button:
            if not self.toggle_active:
                self.MODE = "MANUAL" if self.MODE == "AUTO" else "AUTO"
                self.get_logger().info(f'Modus gewechselt auf: {self.MODE}')
                self.toggle_active = True
        else:
            self.toggle_active = False

        if axes != self.last_axes or buttons != self.last_buttons:
            self.last_axes = list(axes)
            self.last_buttons = list(buttons)

        if self.MODE == "MANUAL":
            velocity = axes[self.js_velocity]
            steering = axes[self.js_steering]
            now = time.monotonic()
            #
            # Nur dann Daten versenden, wenn sich zwischen jetzt und letzter Übertragung etwas geändert hat
            if (now - self.last_i2c_time >= self.min_interval and
                    (velocity != self.last_velocity or steering != self.last_steering)):
                self.publish_steering_velocity(steering, velocity)
                self.last_i2c_time = now
                self.last_velocity = velocity
                self.last_steering = steering
                self.esp32client.write_servo(
                    steering=steering,
                    velocity=velocity
                )
        else:
            self.get_logger().warn("AUTO-MODE noch nicht implementiert")


def main(args=None):
    rclpy.init(args=args)
    node = DriverControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()     