import rclpy
from rclpy.node import Node
from .control.led_pattern import LEDPattern
from .hardware.ws2812_driver import WS2812SPI
from rover_interfaces.msg import LEDMessage
# -------------------------------------------------------------------------------------------------------------------------------
# mit ros2 interface show rover_interfaces/msg/LEDMessage
# kann man sich anzeigen lassen ob das topic LEDMessage den korrekten aufbau hat
# -------------------------------------------------------------------------------------------------------------------------------

def generate_bitmask(start: int, end: int) -> int:
    """Setze Bits von start bis end (inklusiv)."""
    if not (0 <= start <= end < 32):
        raise ValueError("start und end müssen zwischen 0 und 31 liegen.")
    return ((1 << (end - start + 1)) - 1) << start

class LEDUtils:
    """
    Lediglich eine Hilfsklasse die genutzt wird um einzelene BITs (LEDs) anzusprechen und um Kombinationen von
    Mustern zur Verfügung zu stellen.

    Davon ausgehend, das wir vier Ringe (á 7 LEDs) am Rover montiert haben.

    
    """
    # Definition der Ringe (0-basierte Indizes, inklusiv)
    RINGS = {
        'LH': (0, 6),    # Links hinten (LED1–7)
        'LV': (7, 13),   # Links vorne (LED8–14)
        'RV': (14, 20),  # Rechts vorne (LED15–21)
        'RH': (21, 27),  # Rechts hinten (LED22–28)
    }

    # Definition für vordefinierte Kombinationen
    COMBINATIONS = {
        'LEFT_ALL': ['LH', 'LV'],
        'RIGHT_ALL': ['RV', 'RH'],
        'ALL' : ['LH','LV','RV','RH'],
    }

    @staticmethod
    def generate_bitmask(first: int, last: int) -> int:
        """Erzeugt eine Maske mit Bits first...last gesetzt."""
        if not (0 <= first <= last < 32):
            raise ValueError("first / last must be between 0 and 31")
        return ((1 << (last - first + 1)) - 1) << first

    @classmethod
    def ring_mask(cls, ring: str, bitmask: int = None) -> int:
        """Maske für einzelne LEDs innerhalb eines Rings."""
        if ring not in cls.RINGS:
            raise ValueError(f"Ungültiger Ring '{ring}'")
        first, last = cls.RINGS[ring]
        full = cls.generate_bitmask(first, last)
        if bitmask is None:
            return full
        width = last - first + 1
        if bitmask >> width:
            raise ValueError(f"bitmask {bitmask:#x} zu groß für Ring '{ring}'")
        return (bitmask << first) & full

    @classmethod
    def combination_mask(cls, name: str, ring_bitmasks=None) -> int:
        """
        Kombiniert mehrere Ringe und erlaubt bitmasken pro Ring.
        :param name: Ringname oder Combination-Key (z.B. "LEFT_ALL")
        :param ring_bitmasks: dict z.B. {"LH": 0b00101, "LV": 0b01000}
        :return: Kombinierte 32-Bit-Maske.
        """
        if name in cls.COMBINATIONS:
            rings = cls.COMBINATIONS[name]
        elif name in cls.RINGS:
            rings = [name]
        else:
            raise ValueError(f"Kein Ring oder Kombination '{name}'")

        result = 0
        ring_bitmasks = ring_bitmasks or {}

        for r in rings:
            bm = ring_bitmasks.get(r, None)
            result |= cls.ring_mask(r, bm)
        return result

class LEDPatternConfig:
    def __init__(self, pattern_id: int, pattern_name=str, led_type="WS2812", duration: int=0, timeout: int=0, callback=None, callback_param=None):
        """ 
        LEDPattern.
        Im Prinzip ist die Klasse nur ein Platzhalter für die Konfiguration. Die eigentlichen Paremter und die Callbackfunktion die das
        tatsächliche LED-Muster darstellt werden im callback und callback_param übergeben.


        @param pattern_id entspricht der ID aus der Enumeration
        @param pattern_name Name des Musters
        @param duration default=0 (für das Pattern irrelevant), wenn > 0 wird diese Zeit in ms als Pause genutzt für das nachfolgende Pattern (zB BLINK_xxx)#
        @param timeout default=0 (für das Pattern irrelevant), wenn > 0 die Zeit in ms, bis das Pattern deaktiviert (LED OFF) gesetzt wird
        """
        self.pattern_id = pattern_id
        self.pattern_name = pattern_name
        self.led_type = led_type
        self.duration = duration
        # self.duration_on = duration_on
        # self.duration_off = duration_off
        # self.ledmask = ledmask
        # self.timeout = timeout
        self.callback = callback
        self.callback_param = callback_param
        if self.led_type == "WS2812":
            self.led = WS2812SPI()

#------------------------------------------------------------------------------------------------------------# der eigentliche LEDNode
# generischer Aufbau, der Publisher für ein LEDMuster, kann entweder sich auf 
# default Werte beziehen oder explizit Default-Werte überschreiben die dann zur Anzeige kommen
#------------------------------------------------------------------------------------------------------------

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
            ('led_default_ledmask', 0b01010101), # auffallendes LED-Muster
        ])
        self.led_topic = self.get_parameter('led_topic').get_parameter_value().string_value
        self.led_num_pixel = self.get_parameter('led_num_pixels').get_parameter_value().integer_value
        self.led_type = self.get_parameter('led_type').get_parameter_value().string_value
        self.led_default_brightness = self.get_parameter('led_brightness').get_parameter_value().double_value
        self.led_default_timeout = self.get_parameter('led_default_timeout').get_parameter_value().integer_value
        self.led_default_duration_on = self.get_parameter('led_default_duration_on').get_parameter_value().integer_value
        self.led_default_duration_off = self.get_parameter('led_default_duration_off').get_parameter_value().integer_value
        self.led_default_ledmask = self.get_parameter('led_default_ledmask').get_parameter_value().integer_value

        self.get_logger().info(
f"""
LEDNode config:\n\
--------------------------------
Topic:          {self.led_topic},
LEDType:        {self.led_type},
Pixels:         {self.led_num_pixel},
Brightness:     {self.led_default_brightness},
Timeout:        {self.led_default_timeout},
DurationON:     {self.led_default_duration_on},
DurationOFF:    {self.led_default_duration_off},
""")
        self.pattern = {
            LEDPattern.STOP: LEDPatternConfig(LEDPattern.STOP, 
                "STOP", 
                duration=0, 
                timeout=0, 
                callback='stop', 
                callback_param={
                    "color":(0, 0, 0),
                    "timeout":0, 
                    "duration_on": 0, 
                    "duration_off": 0, 
                    "brightness": 0,
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),
            LEDPattern.RED: LEDPatternConfig(LEDPattern.RED, 
                "FILL RED", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={
                    "color":(255, 0, 0),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),
            LEDPattern.GREEN: LEDPatternConfig(LEDPattern.GREEN, 
                "FILL GREEN", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={
                    "color":(0, 255, 0),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),
            LEDPattern.BLUE: LEDPatternConfig(LEDPattern.BLUE, 
                "FILL BLUE", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={
                    "color":(0, 0, 255),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),
            LEDPattern.ORANGE: LEDPatternConfig(LEDPattern.ORANGE, 
                "FILL ORANGE", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={
                    "color":(250, 165, 0),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),
            LEDPattern.WHITE: LEDPatternConfig(LEDPattern.WHITE, 
                "FILL WHITE", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={
                    "color":(255, 255, 255),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),            
            LEDPattern.BLINK_LEFT: LEDPatternConfig(LEDPattern.BLINK_LEFT, 
                "BLINK LEFT", 
                duration=0, 
                timeout=0, 
                callback='blink', 
                callback_param={
                    "color":(255, 165, 0),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    # cooler Trick, ich steuer die linken Ringe komplett an schränke aber über den zweiten Parameter
                    # die tatsächlcihen LEDs ein. in dem Fall sind es jeweils LED2,3,7
                    "ledmask" : LEDUtils.combination_mask('LEFT_ALL', {'LH':0b001000110, 'LV':0b001000110})
                }
            ),             
            LEDPattern.BLINK_RIGHT: LEDPatternConfig(LEDPattern.BLINK_RIGHT,
                "BLINK RIGHT", 
                duration=0, 
                timeout=0, 
                callback='blink', 
                callback_param={
                    "color":(255, 165, 0),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    # cooler Trick, ich steuer die linken Ringe komplett an schränke aber über den zweiten Parameter
                    # die tatsächlcihen LEDs ein. in dem Fall sind es jeweils LED3,4,5
                    "ledmask" : LEDUtils.combination_mask('RIGHT_ALL', {'RV':0b00111000, 'RH':0b00111000})
                }
            ),             
            LEDPattern.HAZARD_LIGHT: LEDPatternConfig(LEDPattern.HAZARD_LIGHT, 
                "HAZARD", 
                duration=0, 
                timeout=0, 
                callback='blink', 
                callback_param={
                    "color":(255, 165, 0),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    # cooler Trick, ich steuer die linken Ringe komplett an schränke aber über den zweiten Parameter
                    # die tatsächlcihen LEDs ein. in dem Fall sind es jeweils LED3,4,5
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),             
            LEDPattern.ROVER_BOOT1: LEDPatternConfig(LEDPattern.ROVER_BOOT1, 
                "ROVER_BOOT1", 
                duration=0, 
                timeout=0, 
                callback='circle', 
                callback_param={
                    "color":(10, 10, 255),
                    "timeout":self.led_default_timeout, 
                    "duration_on": 50, 
                    "duration_off": 25, 
                    "brightness": self.led_default_brightness,
                    # cooler Trick, ich steuer die linken Ringe komplett an schränke aber über den zweiten Parameter
                    # die tatsächlcihen LEDs ein. in dem Fall sind es jeweils LED3,4,5
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ), 
            LEDPattern.ROVER_BOOT2: LEDPatternConfig(LEDPattern.ROVER_BOOT2, 
                "ROVER_BOOT2", 
                duration=0, 
                timeout=0, 
                callback='circle', 
                callback_param={
                    "color":(50, 255, 70),
                    "timeout":self.led_default_timeout, 
                    "duration_on": 50, 
                    "duration_off": 25, 
                    "brightness": self.led_default_brightness,
                    "ledmask" : 0b0 # bei circle wird keine ledmask genutzt
                }
            ), 
        }

        self.subscription = self.create_subscription(
            LEDMessage,
            self.led_topic,
            self.led_callback,
            10
        )

        self.get_logger().info('LEDNode gestartet')

    def _param_override(self, msg_value, default_value):
        return msg_value if msg_value >= 0 else default_value

    def validate_led_pattern(self, pattern_id: int) -> int:
        """
        Prüft, ob pattern_id ein gültiger Wert der LEDPattern-Enum ist.
        
        :param pattern_id: Integer-Wert, der überprüft werden soll
        :return: Gültige pattern_id oder 0, falls ungültig
        """
        try:
            LEDPattern(pattern_id)
            return pattern_id
        except ValueError:
            return 0

    def led_callback(self, msg):
        #
        # ist die empfangene patternID eine valide ID? Wenn nein wird 0 angenommen
        pattern_id = self.validate_led_pattern(msg.pattern)
        pattern_id = LEDPattern(pattern_id)  # kommt vom Publisher

        # das LEDPatternConfog-Object wird benötigt um die tatsächlich konfigurierten Parameter
        # auszulesen. diese werden dann im eigentlichen Callback übergeben
        obj = self.pattern[pattern_id]
        
        config = self.pattern.get(pattern_id)

        if not config:
            self.get_logger().warn(f"Unbekanntes Pattern: {msg.pattern}")
            return

        #
        # Python-Trick
        # getattr liest aus, welche Attribute ein objekt besitzt. In unserem Fall benötigen wir den Inhalt
        # des Callback-Attributes.
        # Hier steht der Methodenname drin, der aufgerufen werden soll.
        # config.led entspricht dem LED-Type (z.B WS2812)

        method = getattr(config.led, config.callback, None)
        #
        if method is None:
            self.get_logger().error(f"Methode {config.callback} nicht gefunden")
            return
        
        user_params = {
            "timeout": self._param_override(msg.timeout, obj.callback_param["timeout"]),
            "duration_on": self._param_override(msg.duration_on, obj.callback_param["duration_on"]),
            "duration_off": self._param_override(msg.duration_off, obj.callback_param["duration_off"]),
            "brightness": self._param_override(msg.brightness,obj.callback_param["brightness"]),
            "ledmask": self._param_override(msg.ledmask, obj.callback_param["ledmask"])
        }

        # überschreibt config.callback_param
        params = {**config.callback_param, **user_params}
        self.get_logger().info(
            f"Aktiviere Pattern {config.pattern_name} \n\t{config.callback} mit Parametern {params} LEDMaskBitPattern: {bin(params['ledmask'])}"
        )
        # das ist der eigentliche Methoden-Aufruf mit übergabe der Parameter.

        method(**params)
        self.get_logger().info(
            f"[method(**params)]: Call {config.led} Mask:{bin(params['ledmask'])}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
