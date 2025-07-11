import rclpy
from rclpy.node import Node
from .control.led_pattern import LEDPattern
from .hardware.ws2812_driver import WS2812SPI
from rover_interfaces.msg import LEDMessage
from enum import Enum  # falls nicht schon vorhanden

import yaml
from ament_index_python.packages import get_package_share_directory
import os
import pprint

# -------------------------------------------------------------------------------------------------------------------------------
# mit ros2 interface show rover_interfaces/msg/LEDMessage
# kann man sich anzeigen lassen ob das topic LEDMessage den korrekten aufbau hat
# -------------------------------------------------------------------------------------------------------------------------------

# def generate_bitmask(start: int, end: int) -> int:
#     """Setze Bits von start bis end (inklusiv)."""
#     if not (0 <= start <= end < 32):
#         raise ValueError("start und end müssen zwischen 0 und 31 liegen.")
#     return ((1 << (end - start + 1)) - 1) << start

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
        'CLH': (0,0),    # Center LH
        'CLV': (7,7),    # Center LH
        'CRV': (14,14),    # Center RV
        'CRH': (21,21),    # Center RH

    }

    # Definition für vordefinierte Kombinationen
    COMBINATIONS = {
        'LEFT_ALL': ['LH', 'LV'],
        'RIGHT_ALL': ['RV', 'RH'],
        'CENTERH' : ['CLH','CRH'],
        'CENTERA' : ['CLV','CRV','CLH','CRH'],
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
        Im Prinzip ist die Klasse nur ein Platzhalter für die Konfiguration. Die eigentlichen Paramemter und die Callbackfunktion die das
        tatsächliche LED-Muster darstellt werden im callback und callback_param übergeben.

        
        LEDPattern=1 = DEFAULT
        das ist ein Pattern das genutzt werden kann, wenn der Publisher im Prinzip alle Attribute selber setzen möchte

        @param pattern_id entspricht der ID aus der Enumeration
        @param pattern_name Name des Musters
        @param duration default=0 (für das Pattern irrelevant), wenn > 0 wird diese Zeit in ms als Pause genutzt für das nachfolgende Pattern (zB BLINK_xxx)#
        @param timeout default=0 (für das Pattern irrelevant), wenn > 0 die Zeit in ms, bis das Pattern deaktiviert (LED OFF) gesetzt wird
        """
        self.pattern_id = pattern_id
        self.pattern_name = pattern_name
        self.led_type = led_type
        self.duration = duration
        self.callback = callback
        self.callback_param = callback_param
        if self.led_type == "WS2812":
            self.led = WS2812SPI()
        else:
            # wenn alle Stricke reißen ;-)
            self.led = WS2812SPI()


class LEDPatternLoader:
    """ liest eine YAML Patterndatei
    Der Key entspricht dem Wert der in LEDPattern(Enum) definiert wurde
    Zurückgegeben Struktur als Beispiel:
    {
        0: {
            'callback': 'fill',
            'timeout': 0,
            'duration_on': 0,
            'duration_off': 0,
            'brightness': 0.3,
            'ledmask': 0x3FFFFF,           # int-Wert (nicht String)
            'name': 'NONE',
            'color': [0, 0, 0]
        },
        50: {
            'callback': 'fill',
            'timeout': 0,
            'duration_on': 0,
            'duration_off': 0,
            'brightness': 0.3,
            'ledmask': 0x3FFFFF,
            'name': 'FILL RED',
            'color': [255, 0, 0]
        },
        61: {
            'callback': 'blink',
            'timeout': 0,
            'duration_on': 125,
            'duration_off': 125,
            'brightness': 0.3,
            'ledmask': 0b0000001000000100000010000001,
            'name': 'BAT100',
            'color': [51, 229, 0]
        },
        ...
    }
    
    
    """

    def __init__(self, path:str, file:str, logger):
        self.logger = logger
        self.pattern_file = os.path.join(
            get_package_share_directory('rover'),
            path,
            file
        )
        self.logger.info(f"[LEDPatternLoader] {self.pattern_file}")

    def load_yaml(self, led_type="WS2812"):
        with open( self.pattern_file, 'r') as f:
            raw_data = yaml.safe_load(f)

        pattern_dict = {}

        for pattern_id, entry in raw_data.items():
            try:
                pattern_id = int(pattern_id)  # YAML keys sind Strings
                name = entry.get("name", f"PATTERN_{pattern_id}")
                callback = entry.get("callback", "fill")  # fallback auf "fill"
                timeout = entry.get("timeout", 0)
                duration_on = entry.get("duration_on", 0)
                duration_off = entry.get("duration_off", 0)
                duration = entry.get("duration", 0)  # optional für spätere Erweiterung
                brightness = entry.get("brightness", 0.3)
                ledmask = entry.get("ledmask", 0x3FFFFF)
                color = entry.get("color", [0, 0, 0])

                callback_param = {
                    "color": color,
                    "timeout": timeout,
                    "duration_on": duration_on,
                    "duration_off": duration_off,
                    "brightness": brightness,
                    "ledmask": ledmask
                }

                pattern_obj = LEDPatternConfig(
                    pattern_id=pattern_id,
                    pattern_name=name,
                    led_type=led_type,
                    duration=duration,
                    timeout=timeout,
                    callback=callback,
                    callback_param=callback_param
                )

                pattern_dict[pattern_id] = pattern_obj
            except ValueError:
                # Kein int → ignoriere z.B. "HR", "defaults", etc.
                print(f"[YAML Load] Überspringe Eintrag: '{pattern_id}' (kein numerischer Pattern-Key)")

            except Exception as e:
                print(f"[YAML Load] Fehler bei Pattern-ID {pattern_id}: {e}")

        return pattern_dict


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
            ('led_default_ledmask',0x555555), # auffallendes LED-Muster
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

        patternLoader = LEDPatternLoader("config", "ledpatterns.yaml", self.get_logger())
        self.yamlPattern = patternLoader.load_yaml()
        self.get_logger().info(f"Verfügbare Pattern-IDs: {list(self.yamlPattern.keys())}")
  
        self.subscription = self.create_subscription(
            LEDMessage,
            self.led_topic,
            self.led_callback,
            10
        )

        self.get_logger().info('LEDNode gestartet')

    def resolve_value(self, msg_value, default_value):
        """
        Entscheidet, ob der vom Publisher gesendete Wert (msg_value)
        benutzt werden soll, oder ob der Default-Wert verwendet wird.

        Die Logik:
        - Für Zahlen (int, float): alles außer None wird verwendet, auch 0
        - Für Listen/Tupel (z. B. color): wird verwendet, wenn nicht exakt [0,0,0]
        - Für Strings: wird verwendet, wenn nicht leer
        - Für None: default wird genommen
        """
        if msg_value is None:
            return default_value

        if isinstance(msg_value, (int, float)):
            return msg_value

        if isinstance(msg_value, (list, tuple)):
            if msg_value == [0, 0, 0] or msg_value == (0, 0, 0):
                return default_value
            return msg_value

        if isinstance(msg_value, str):
            return msg_value if msg_value.strip() else default_value

        # fallback für alles andere
        return msg_value


    def validate_led_pattern(self, pattern_id: int) -> bool:
        """
        Prüft, ob pattern_id ein gültiger Wert der LEDPattern-Enum ist.
        
        :param pattern_id: Integer-Wert, der überprüft werden soll
        :return: True/False
        """
        try:
            LEDPattern(pattern_id)
            return True
        except ValueError:
            return False

    def led_callback(self, msg):
        #
        # ist die empfangene patternID eine valide ID? Wenn nein wird 0 angenommen
        self.get_logger().info(f"Subscribed Message: {msg}")

        #
        # Ist die PatternID eine valide ID?
        pattern_id = msg.pattern
        if self.validate_led_pattern(pattern_id) == False:
            self.get_logger().warn(f"Publisher nutzt eine ungültige LEDPatternID: {pattern_id}")
            return
       
        #
        # in yamlPattern wurde für jedes Pattern ein Objekt generiert vom Type LEDPatternConfig
        # diese Objekt enthält nun alle notwendigen Konfiguraitonsdaten für das Pattern als auch
        # die callback funktion die letztendlich die LEDs ansteuert.

        patternObj = self.yamlPattern.get(pattern_id)
        if patternObj is None:
            self.get_logger().warn(f"PatternID wurde nicht in yamlPattern gefunden: {pattern_id}")
            return
        self.get_logger().debug(f"---- [LEDNode] MSG.pattern: {pattern_id}; Pattern: {patternObj.pattern_name}")

        #
        # nun wird der callback funktionsname benötigt. dieser ist im patternObj schon enthalten
        # des weiteren steckt im Objekt auch ein Klassenverweis zum LEDType (z.B WS2812()) drin
        # über diese Klasse wird dann der callback aufgerufen
        # WS2812.fill()
        callbackFnc = patternObj.callback

        #
        # Python-Hack
        # mit der Funktion getattr liest man aus, welche Attribute ein Objekt hat (z.B. WS2812())
        # Da wir ja den LEDDriver (WS2812) nutzen, benötigen wir nun die Methoden die diese Klasse zur
        # Verfügung stellt (fill(), blink(), run(), circle()) im Prinzip sind das die callback funktionen
        callback = getattr(patternObj.led, callbackFnc, None)
        self.get_logger().debug(f"---- [LEDNode] callbackFnc: {callbackFnc} Object: {callback}")

        #
        # gibts die funktion überhaupt im LED_Driver?
        if callback is None:
            self.get_logger().error(f"Methode {callbackFnc} nicht gefunden")
            return        

        self.get_logger().debug(f"---- [LEDNode] callbackParam: \n{pprint.pformat(patternObj.callback_param)}")

        #
        # Nun müssen wir die korrekten Parameter noch setzen
        # in der YAML-Datei wurden für das aktuelle Pattern alle notwendigen Parameter per default 
        # gesetzt. Der Publish kann diese aber überschreiben
        #
        # resolve_value() setzt entweder den wert aus der Message oder sonst den default wert aus dem PatternObj
        user_params = {
            "timeout": self.resolve_value(msg.timeout, patternObj.callback_param["timeout"]),
            "duration_on": self.resolve_value(msg.duration_on, patternObj.callback_param["duration_on"]),
            "duration_off": self.resolve_value(msg.duration_off, patternObj.callback_param["duration_off"]),
            "brightness": self.resolve_value(msg.brightness,patternObj.callback_param["brightness"]),
            "ledmask": self.resolve_value(msg.ledmask, patternObj.callback_param["ledmask"]),
            "color" : self.resolve_value(msg.color, patternObj.callback_param["color"]),
        }

        user_params = {
            "timeout": patternObj.callback_param["timeout"],
            "duration_on": patternObj.callback_param["duration_on"],
            "duration_off": patternObj.callback_param["duration_off"],
            "brightness": patternObj.callback_param["brightness"],
            "ledmask": patternObj.callback_param["ledmask"],
            "color" : patternObj.callback_param["color"],
        }

        self.get_logger().info(f"---- [LEDNode] user_params: \n{pprint.pformat(user_params)}")

        # überschreibt config.callback_param
        params = {**patternObj.callback_param, **user_params}
        self.get_logger().info(
            f"Aktiviere Pattern {patternObj.pattern_name} \n\t{patternObj.callback} mit Parametern {params} LEDMaskBitPattern: {bin(params['ledmask'])}"
        )
        # das ist der eigentliche Methoden-Aufruf mit übergabe der Parameter.
        # es wird der Platzhalter callback genutzt um die tatsächliche Methode aufzurufen
        callback(**params)
        self.get_logger().info(
            f"[method(**params)]: Call {patternObj.led} Mask:{bin(params['ledmask'])}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
