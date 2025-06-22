import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
import time  # am Anfang ergänzen
from rclpy.parameter import Parameter


#from src.rover.hardware.rover_driver import RoverDriver
from .hardware.rover_driver import RoverDriver

#from src.rover.hardware.i2c_driver import I2CTest;

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

    def __init__(self):
        super().__init__(
            'driver_controller_node',
            automatically_declare_parameters_from_overrides=True
        )
        self.js_velocity = JOYSTICKS.LJ_UD.value
        self.js_steering = JOYSTICKS.LJ_LR.value

        self.led_left = BUTTONS.LB.value
        self.led_right = BUTTONS.RB.value

        self.rover_driver = RoverDriver(self.get_logger())
        # Parameter deklarieren mit Default
        self.declare_parameter('cmd_vel_topic', '/joy')

        # Parameter auslesen
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        # Parameter aus YAML oder Default übernehmen
        self.reverse_steering = self.get_parameter_or(
            'reverse_steering',
            Parameter('reverse_steering', Parameter.Type.BOOL, False)
        ).value

        self.reverse_velocity = self.get_parameter_or(
            'reverse_velocity',
            Parameter('reverse_velocity', Parameter.Type.BOOL, False)
        ).value

        self.get_logger().info(
            f" - cmd_vel_topic:     {cmd_vel_topic}"
        )
        self.get_logger().info(
            f" - reverse_steering:  {self.reverse_steering}"
        )
        self.get_logger().info(
            f" - reverse_velocity:  {self.reverse_velocity}"
        )
        
        # Subscriber anlegen (hier für Twist Nachrichten)
        self.subscription = self.create_subscription(
            Joy,   # oder Joy, falls das dein Message-Typ ist
            cmd_vel_topic,
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
        self.min_interval = 0.05  # mindestens 50ms zwischen zwei I2C-Aufrufen

        self.rb_blinking = False
        self.lb_blinking = False
        self.rb_led_state = False
        self.lb_led_state = False

        # 500ms Timer für Blinken
        self.timer = self.create_timer(0.5, self.blink_timer_callback)

        self.get_logger().info(f'DriverControllerNode gestartet. Warte auf {cmd_vel_topic}...')

    def blink_timer_callback(self):
        """
        Callback des create_timer(self.blink_timer_callback)

        """

        if self.rb_blinking and self.lb_blinking:
            self.lb_led_state = not self.lb_led_state
            self.rb_led_state = not self.rb_led_state
            self.get_logger().info(f"LB-State: {self.lb_led_state} RB-STATE: {self.rb_led_state}")
            self.rover_driver.digitalWrite(
                [ESP32_PORTS.LED1.value, ESP32_PORTS.LED2.value],
                [self.lb_led_state, self.rb_led_state]
            )
            self.get_logger().info("HAZARD WARNING LIGHT")
            return

        # LED1 (LB) blinken
        if self.lb_blinking:
            self.lb_led_state = not self.lb_led_state
            self.rover_driver.digitalWrite(
                [ESP32_PORTS.LED1.value, 0],
                [self.lb_led_state, 0]
            )
            self.get_logger().info("LEFT TURN LIGHT")
            return
        
        # LED2 (RB) blinken
        if self.rb_blinking:
            self.rb_led_state = not self.rb_led_state
            self.rover_driver.digitalWrite(
                [0, ESP32_PORTS.LED2.value],
                [0, self.rb_led_state]
            )
            self.get_logger().info("RIGHT TURN LIGHT")
            return

    def cmd_driver_callback(self, msg:Joy):
        axes = msg.axes.tolist()
        axes = [round(x,3) for x in axes]
        buttons = msg.buttons.tolist()



        x_button = buttons[BUTTONS.X.value]
        b_button = buttons[BUTTONS.B.value]


        lb_button = buttons[BUTTONS.LB.value]
        rb_button = buttons[BUTTONS.RB.value]

        # Blinken toggeln bei Button-Press (Rising-Edge-Detection)
        if self.last_buttons is not None:
            if lb_button and not self.last_buttons[BUTTONS.LB.value]:
                self.lb_blinking = not self.lb_blinking
                self.get_logger().info(f'LB-Button: Blinken {"AN" if self.lb_blinking else "AUS"}')

                # sicherstellen das die LED auch aus ist
                if not self.lb_blinking:
                    self.rover_driver.set_led(ESP32_PORTS.LED1.value, False)
                    self.lb_led_state = False

            if rb_button and not self.last_buttons[BUTTONS.RB.value]:
                self.rb_blinking = not self.rb_blinking
                self.get_logger().info(f'RB-Button: Blinken {"AN" if self.rb_blinking else "AUS"}')
                # sicherstellen das die LED auch aus ist
                if not self.rb_blinking:
                    self.rover_driver.set_led(ESP32_PORTS.LED2.value, False)
                    self.rb_led_state = False
        
        if  x_button and  b_button:
            #
            # X+B am Controller toggelt von MANUAL-Mode in AUTO-MODE und wieder zurück
            #self.get_logger().info(f'MODE1 {self.MANUAL_MODE} vs {self.last_mode}')
            if not self.toggle_active:
                # Nur einmal toggeln
                self.MODE = "MANUAL" if self.MODE == "AUTO" else "AUTO"
                self.logger.info(f'*** Modus gewechselt auf: {self.MODE}')
                self.toggle_active = True  # merken: wurde bereits getoggelt
        else:
            # Reset: Mind. eine Taste losgelassen
            self.toggle_active = False

        if axes != self.last_axes or buttons != self.last_buttons:
            self.get_logger().debug(f'[Joystick] axes: {axes}, buttons: {buttons}')
            
            # Richtig: Werte speichern
            self.last_axes = list(axes)
            self.last_buttons = list(buttons)        #
            # Übergabe der Daten an den Hardware-Driver
        if self.MODE == "MANUAL":
            velocity = axes[self.js_velocity]
            steering = axes[self.js_steering]

            now = time.monotonic()
            if (now - self.last_i2c_time >= self.min_interval and
                (velocity != self.last_velocity or steering != self.last_steering)):

                self.get_logger().debug(f'[cmd_driver_callback] velo: {velocity}, steering: {steering}')
                rc = self.rover_driver.set_steeringAndVelocity(
                    steering=steering, 
                    velocity=velocity, 
                    reverse_steering=self.reverse_steering, 
                    reverse_velocity=self.reverse_velocity
                )

                self.last_i2c_time = now
                self.last_velocity = velocity
                self.last_steering = steering
                return rc
            else:
                self.get_logger().debug("I2C übersprungen: keine Änderung oder Rate-Limit aktiv")
        else:
            #
            # noch nicht implementiert
            #
            self.get_logger().warn("AUTO-MODE noch nicht implementiert")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DriverControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()