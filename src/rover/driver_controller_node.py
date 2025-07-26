import rclpy
from rclpy.lifecycle import LifecycleNode
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
import random

from sensor_msgs.msg import Joy
from enum import Enum
import time  # am Anfang ergänzen
from rover_interfaces.msg import I2CWrite
from rover_interfaces.msg import LEDMessage
from rover_interfaces.srv import I2CESP32Communication
from .control.led_pattern import LEDPattern
from .control.ESP32Client import ESP32Client
from .control.ESP32CommandsV1 import CommandID, SubCommandID, ESP32PINS
from .control.utilities import Utilities
from .control.ros_utilities import *

from .rover_exceptions import *
import os


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

class DriverControllerNode(LifecycleNode):
    """
    Der DriverControllNode steuert über den I2C_Node die Verbindung zum ESP indem er eine
    I2CESPCommunication Service-Nachricht versendet. Der Vorteil von Serivce-Nachrichten
    ist, das der ESP32 mit einem Response antwortet.
    Somit können auch Daten vom ESP32 empfangen werden

    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
    BEACHTEN: 
    in der Launch datei werden die Nodes mit dependencies gestartet
    der driver_controller_node hat eine dependency zum i2c_node. Erst wenn dieser on_active() erfolgreich
    durchlaufen hat, geht der driver_controller_node von inaktive auf active.
    Hintergrund ist: dieses Node ist davon abhängig das der i2c_node den service /i2c/esp32command gestartet hat

    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    Publish-Service-Message: (OUT)
    (Versenden von Steering/Velocity Nachrichten)
    - /i2c/esp32_command: Message zu Steuerung des ESP32
    - /led : LEDMessage Anzeige von LEDPattern (z.B blinken, Warnblinker, ...)

    SUBSCRIBE-Messages: (IN)    
    - /joy : Daten des Gamepad-Controllers

    I2CESPCommunication (Service) 
    ---------------------------------------------------------------------------
    # Request für den ESP32 verwendet wird
    #
    # command:              1=dynamixel, 2=esp32
    # subcommand:           1=write, 2=read, 3=status
    # fvalues               bei cmd=1, scmd=1, fvalues[0] = steering-wert, fvalues[1]=velocity-wert
    # ivalues               bei cmd=2, scmd=1, ivalues[0]=pin1, ivalues[1]=state_pin1, ivalues[2]=pin2, ivalues[2]=state_pin2, ...
    string device           # 'ESP32'
    int32 command           # Command für den Slave
    int32 subcommand        # ggf. SubCommand für den Slave (z.B. esp32)
    float32[] fvalues         # gefüllt je nach command/subcommand
    int32[] ivalues         # gefüllt je nach command/subcommand

    ---
    # Response
    float32[] fvalues         # gefüllt je nach command/subcommand
    int32[] ivalues         # gefüllt je nach command/subcommand
    

    
    I2CWrite (aktuell nicht verwendet)
    #-------------------------
    # I2C V01 Version
    #-------------------------
    # grundlegendes Kommando was an den ESP
    # verwendet wird
    string command      # Nur String zur Ausgabe, keine funktionaler Inhalt
    int32[] pins        # Liste an Pins die angesteuert werden können
    int32[] states      # Liste an States der in pins angegeben Pins
    int32 cmd           # CommandID - tatsächliche Funktionsaufruf
    int32 subcmd        # SubCommandID bezogen auf CommandID
    float64[] data      # bis zu 5 Floatwerte

    LEDMessage
    #-------------------------
    int32 pattern
    string ledtype
    int32 timeout
    int32 duration
    int32 duration_on
    int32 duration_off
    float32 brightness
    int32 ledmask
    int32[3] color     # RGB: z.B. [255, 100, 0]
    string callback    # Name des Musters oder Methode, z. B. "blink"    
    
    """

    def __init__(self):
        self.node_name = self.__class__.__name__
        super().__init__(self.node_name)
        self.pid = os.getpid()

        # Parameter deklarieren
        self.declare_parameter("log_level", "INFO")  # Default als Fallback
        level_str = self.get_parameter("log_level").get_parameter_value().string_value
        from rclpy.logging import LoggingSeverity
        log_level = getattr(LoggingSeverity, level_str.upper(), LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)

        self.get_logger().info(f"{self.node_name} instantiated")

        ct_tled = Utilities.get_common_topic('topic_led', '/led', logger=self.get_logger())
        ct_bat = Utilities.get_common_topic('topic_battery', '/battery', logger=self.get_logger())
        ct_twrite = Utilities.get_common_topic('i2c_write_topic','/i2c/write', logger=self.get_logger())
        ct_srv_esp = Utilities.get_common_topic('i2c_esp_command_srv','/i2c/esp32_command', logger=self.get_logger())


        self.declare_parameters(
        namespace='',
        parameters=[
            # Common config
            ('topic_led', ct_tled),
            ('topic_battery', ct_bat),
            ('i2c_write_topic', ct_twrite),
            ('i2c_esp_command_srv', ct_srv_esp),

            # Node config
            ('cmd_vel_topic', '/joy'),
            ('reverse_steering', True),
            ('reverse_velocity', False),
            ('map_js_steering', 0),
            ('map_js_velocity', 1),
            ('map_js_cam_turn', 2),
            ('map_js_cam_tilt', 3),
            ('service_wait_timeout', 10.0),


        ])


        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.topic_battery = self.get_parameter('topic_battery').get_parameter_value().string_value
        self.topic_led = self.get_parameter('topic_led').get_parameter_value().string_value
        self.i2c_write_topic = self.get_parameter('i2c_write_topic').get_parameter_value().string_value
        self.i2c_esp_command_srv = self.get_parameter('i2c_esp_command_srv').get_parameter_value().string_value

        self.reverse_steering = self.get_parameter('reverse_steering').get_parameter_value().bool_value
        self.reverse_velocity = self.get_parameter('reverse_velocity').get_parameter_value().bool_value
        self.map_js_steering = self.get_parameter('map_js_steering').get_parameter_value().integer_value
        self.map_js_velocity = self.get_parameter('map_js_velocity').get_parameter_value().integer_value
        self.map_js_cam_turn = self.get_parameter('map_js_cam_turn').get_parameter_value().integer_value
        self.map_js_cam_tilt = self.get_parameter('map_js_cam_tilt').get_parameter_value().integer_value
        self.service_wait_timeout = self.get_parameter('service_wait_timeout').get_parameter_value().double_value


        self.get_logger().info(
        f"""
        DriverControllerNode(Node) config:\n\
        PID:                    {self.pid}
        ---------------------------------------------------
        COMMON --------------------------------------------
        cmd_vel_topic:          {self.cmd_vel_topic}
        topic_led:              {self.topic_led}
        topic_battery:          {self.topic_battery}
        i2c_esp_command_srv:     {self.i2c_esp_command_srv}

        DriverControllerNode ------------------------------
        reverse_steering:       {self.reverse_steering}
        reverse_velocity:       {self.reverse_velocity}
        map_js_steering:        {self.map_js_steering}
        map_js_velocity:        {self.map_js_velocity}
        map_js_cam_turn:        {self.map_js_cam_turn}
        map_js_cam_tilt:        {self.map_js_cam_tilt}
        """)



        self.last_velocity = None
        self.last_steering = None
        self.last_axes = None
        self.last_buttons = None
        self.MODE = "MANUAL"
        self.toggle_active = False
        self.last_i2c_time = time.monotonic()
        self.min_interval = 0.05

    #-------------------------------------------------------------------------------------------------------
    #   🛠 on_configure()
	#	Zweck:      Initialisierung des Nodes (z. B. Parameter laden, Publisher/Subscribers erstellen – aber noch nicht aktivieren).
	#	Auslöser:   Übergang von unconfigured → inactive
	#	Typisch:    Ressourcen vorbereiten, aber noch keine Kommunikation starten.
    #-------------------------------------------------------------------------------------------------------
    def on_configure(self, state: State):
        self.get_logger().info("🚀  on_configure wurde betreten")
        self.get_logger().info(f'{self.node_name}: Konfiguriere...')
        #return super().on_configure(state)


        self.get_logger().info(f"\t📳 create publisher für I2CWrite")
        self.publisher = self.create_publisher(I2CWrite, '/i2c/write', 10)
        self.get_logger().info(f"\t📳 create publisher für LEDMessages")
        self.led_pub = self.create_publisher(LEDMessage, '/led', 10)


        periode_s = 5.0
        self.test_periodically_timer_active = False
        self.get_logger().info(f"\t⏱️ create periodical GameController Messages")
        self.test_periodical_timer = self.create_timer(periode_s, self.test_periodically_send_service_messsage)

        self.get_logger().info("✅ on_configure ready")
        return TransitionCallbackReturn.SUCCESS

    #-------------------------------------------------------------------------------------------------------
    #   ✅ on_activate()
	#	Zweck:      Aktivieren des Nodes – z. B. Publisher freischalten, Timer starten.
	#	Auslöser:   Übergang von inactive → active
	#	Typisch:    Start der eigentlichen Funktionalität.
    #-------------------------------------------------------------------------------------------------------
    def on_activate(self, state: State):
        self.get_logger().info("🚀🚀  on_activate wurde betreten")

        try:
            #
            # der ESP32Client ist faktisch die Schnittstelle zwischen dem
            # driver_controller und dem I2CNode und versendet die Serivce-nachricht
            self.get_logger().info('\t📳 setup ESP32Client')
            self.esp32client = ESP32Client( node=self, 
                                            channel=self.i2c_esp_command_srv,
                                            timeout=self.service_wait_timeout
                                        )
            self.get_logger().info('\t📳 ESP32Client Objekt erhalten')

            #self.get_logger().info(f"create publisher für I2CWrite")
            #self.publisher = self.create_publisher(I2CWrite, '/i2c/write', 10)
            self.get_logger().info(f"\t🔴🟡🟢 create publisher für LEDMessages")
            self.led_pub = self.create_publisher(LEDMessage, '/led', 10)

            self.js_velocity = JOYSTICKS.LJ_UD.value
            self.js_steering = JOYSTICKS.LJ_LR.value

            #
            # teleop-Topic abonnieren
            self.cmd_vel_topic = self.cmd_vel_topic
            self.get_logger().info(f"\t📳 create subscription für JoyStick-Commands")
            self.subscription = self.create_subscription(
                Joy,
                self.cmd_vel_topic,
                self.cmd_driver_callback,
                10
            )


        except RoverException as err:
            raise RoverException()

        self.get_logger().info("✅✅ on_activate ready")
        #return super().on_activate(state)
        self.test_periodically_timer_active = True
        return TransitionCallbackReturn.SUCCESS

    #-------------------------------------------------------------------------------------------------------
    #   ⏸ on_deactivate()
	#	Zweck:      Temporäres Pausieren – Publisher deaktivieren, aber Ressourcen behalten.
    #	Auslöser:   Übergang von active → inactive
    #	Typisch:    Nützlich für Systemwechsel oder geplante Pausen.
    #-------------------------------------------------------------------------------------------------------
    def on_deactivate(self, state: State):

        self.get_logger().info(f'🧼🧼🧼 on_deactivate()')
        return super().on_deactivate(state)

    #-------------------------------------------------------------------------------------------------------
    #   ❌ on_shutdown()
	#	Zweck:      Endgültiges Herunterfahren, z. B. bei ROS-Abbruch oder manuellem Stop.
	#	Auslöser:   Übergang aus jedem Zustand → finalized
	#	Typisch:    Letzte Aufräumarbeiten, Logging etc.
    #-------------------------------------------------------------------------------------------------------
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        try:
            if rclpy.ok():
                self.get_logger().info(f"🧼🧼🧼🧼 [{self.node_name}] on_shutdown")
        except Exception as e:
            self.get_logger().error(f'❌❌❌❌❌❌[{self.node_name}] Fehler in on_shutdown(): {e}❌❌❌❌❌❌')
            import traceback
            self.get_logger().error(traceback.format_exc())    

        self._destroy_resources()

        self.get_logger().info(f'[{self.node_name}] Shutdown erfolgreich abgeschlossen.')
 
        self._current_state = LifecycleState.PRIMARY_STATE_INACTIVE
        self.get_logger().info(f"[{self.node_name}] Node im Status '{LIFECYCLE_STATE_LABELS[self._current_state]}'")
 
        return super().on_shutdown(state)

    #-------------------------------------------------------------------------------------------------------
	#   🔁 on_cleanup()
    #	Zweck: Aufräumen aller Ressourcen, Rücksetzen in den Ursprungszustand.
	#	Auslöser: Übergang von inactive → unconfigured
	#	Typisch: Alles schließen, als ob der Node frisch gestartet wurde.    
    #-------------------------------------------------------------------------------------------------------


    #-------------------------------------------------------------------------------------------------------
    # ⚠️ on_error()
	#	Zweck: Fehlerbehandlung bei fehlgeschlagenem Übergang.
	#	Auslöser: Fehler in anderen Transitions (z. B. on_activate schlägt fehl)
	#	Typisch: Logging, Ressourcenfreigabe, ggf. Rückkehr in sicheren Zustand.    
    #-------------------------------------------------------------------------------------------------------


    def _destroy_resources(self):
        self.get_logger().info(f"[_destroy_resources] Node sauber runter fahren")
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
        if self.led_pub is not None:
            self.destroy_publisher(self.led_pub)
        if self.esp32client is not None:
            self.esp32client.shutdown()
            self.esp32client = None
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
        if self.test_periodical_timer is not None:
            self.destroy_timer(self.test_periodical_timer)


    def publish_led_pattern(self, pattern_id: int, timeout: int = 0, duration_on: int = 0, duration_off: int = 0, ledmask: int = 0x0fffffff):
        """
        Das gewünsche LEDPattern publizieren.

        Wichtig:
        Topic-Messages haben seitens Ros immer einen Default wert, bei Interger & Float ist das 0 bzw. 0.0, bei Strings ist das ''
        Sendet man nun aber im Timeout=0 an den LEDNode, geht der Node davon aus, das es keinen Timeout geben soll und überschreibt den
        eigentlichen Default-Wert des Patterns. Das Ergebnis ist, das das Pattern anders reagiert als gedacht. Gleiches gitlt für duration_on/_off usw.

        Daher ist es wichtig das das die werte auf -1 gesetzt werden. der LEDNode prüft auf diesen Wert und wenn er vorhanden ist, wird dieser Wert ignoriert
        und mit dem DefaultWert überschrieben.

        Somit ist es explizit möglich, das ein Publisher entweder mit Default-Werten arbeiten kann und sie mit neuen Werten überschreibt.
        """
        msg = LEDMessage()
        msg.pattern = pattern_id
        msg.ledtype = "WS2812"
        msg.timeout = timeout
        msg.duration_on = duration_on
        msg.duration_off = duration_off
        msg.ledmask = ledmask
        self.led_pub.publish(msg)
        self.get_logger().info(f"🔴🟡🟢 Published LEDMessage() '{msg}'")            


    def cmd_driver_callback(self, msg: Joy):
        axes = msg.axes.tolist()
        axes = [round(x, 3) for x in msg.axes]
        buttons = msg.buttons.tolist()

        #self.get_logger().debug(f"cmd_driver_callback({msg})")
        #
        # für einfacheren Zugriff, buttons in Variablen ablegen
        x_button = buttons[BUTTONS.X.value]
        b_button = buttons[BUTTONS.B.value]
        lb_button = buttons[BUTTONS.LB.value]
        rb_button = buttons[BUTTONS.RB.value]

        if self.last_buttons is not None:
            # Warnblink
            if rb_button and lb_button:
                self.publish_led_pattern(
                    pattern_id=LEDPattern.OFF.value,
                    timeout=-1,             # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_on=-1,         # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_off=-1,        # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    ledmask = 0             # 0 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                )
                self.publish_led_pattern(
                    pattern_id=LEDPattern.HAZARD.value,
                    timeout=-1,             # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_on=-1,         # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_off=-1,        # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    ledmask = 0             # 0 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                )
            else:
                #
                # Links blinken
                if lb_button and not self.last_buttons[BUTTONS.LB.value]:
                    self.publish_led_pattern(
                    pattern_id=LEDPattern.BLINK_LEFT.value,
                    timeout=-1,             # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_on=-1,         # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_off=-1,        # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    ledmask = 0             # 0 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                )

                #
                # Rechts blinken
                if rb_button and not self.last_buttons[BUTTONS.RB.value]:
                    self.publish_led_pattern(
                    pattern_id=LEDPattern.BLINK_RIGHT.value, 
                    timeout=-1,             # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_on=-1,         # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    duration_off=-1,        # -1 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
                    ledmask = 0             # 0 muss explizit gesetzt werden, wenn man default werte NICHT überschreiben möchte
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
            #self.get_logger().info(f"MANUAL-MODE......")            
            velocity = axes[self.js_velocity]
            steering = axes[self.js_steering]
            now = time.monotonic()
            #
            # Nur dann Daten versenden, wenn sich zwischen jetzt und letzter Übertragung etwas geändert hat
            if (now - self.last_i2c_time >= self.min_interval and
                    (velocity != self.last_velocity or steering != self.last_steering)):
                self.last_i2c_time = now
                self.last_velocity = velocity
                self.last_steering = steering
                self.get_logger().debug(f"[cmd_driver_callback] WRITE_SERVO {steering:.5f} | {velocity:.5f}")
                self.esp32client.write_servo(
                        steering=steering,
                        velocity=velocity
                )
                self.get_logger().debug(f"successfully send to service")

        else:
            self.get_logger().warn("AUTO-MODE noch nicht implementiert")

    def test_periodically_send_service_messsage(self):
        """
        NUR ZUM TEST
        sende periodisch über /i2c/esp32_command eine Nachricht und simulliert
        einen GameController Eingabe
        """
        if not self.test_periodically_timer_active :
            return 
        steering = Utilities.random_step_value(-1.0, 1.0, 5, 5)
        velocity = Utilities.random_step_value(-1.0, 1.0, 5, 5)
        self.get_logger().debug(f"➡️ TEST GameController :  {steering:.5f} | {velocity:.5f}")
        self.esp32client.write_servo(
                steering=steering,
                velocity=velocity
        )

def main(args=None):
    rclpy.init(args=args)
    node = DriverControllerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Node wird beendet...")
        node.destroy_node()
        # ⚠️ Shutdown nur, wenn Kontext nicht schon heruntergefahren!
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

